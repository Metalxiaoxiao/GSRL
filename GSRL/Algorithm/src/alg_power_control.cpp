/**
 ******************************************************************************
 * @file           : alg_power_control.cpp
 * @brief          : power control algorithm for mecanum chassis
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 GMaster
 * All rights reserved.
 *
 ******************************************************************************
 */

#include "alg_power_control.hpp"
#include "dvc_power_meter.hpp"
#include "alg_filter.hpp"  // RLSFilter
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>

// 静态辅助函数
static inline fp32 clampRange(fp32 value, fp32 minValue, fp32 maxValue)
{
    if (value < minValue) {
        return minValue;
    }
    if (value > maxValue) {
        return maxValue;
    }
    return value;
}

static inline fp32 clampAbs(fp32 value, fp32 limit)
{
    if (value > limit) {
        return limit;
    }
    if (value < -limit) {
        return -limit;
    }
    return value;
}

static inline bool floatEqual(fp32 a, fp32 b)
{
    return fabsf(a - b) < 1e-5f;
}

static inline fp32 rpm2av(fp32 rpm)
{
    return rpm * (fp32)MATH_PI / 30.0f;
}

// 静态包装函数，用于FreeRTOS任务
static void powerDaemonWrapper(void *pvParam)
{
    PowerManager::powerDaemon(pvParam);
}

// PowerManager 构造函数
PowerManager::PowerManager(Motor *motorRf,
                           Motor *motorLf,
                           Motor *motorLb,
                           Motor *motorRb,
                           fp32 powerUpperLimit_,
                           fp32 k1_,
                           fp32 k2_,
                           fp32 k3_,
                           fp32 minMaxPower_,
                           uint8_t rlsEnabled_)
    : powerUpperLimit(powerUpperLimit_),
      minMaxPower(minMaxPower_),
      userConfiguredMaxPower(0.0f),
      measuredPower(0.0f),
      estimatedPower(0.0f),
      lastUpdateTick(0),
      rlsEnabled(rlsEnabled_),
      callback(nullptr),
      torqueConst(0.0f),
      k1(k1_),
      k2(k2_),
      k3(k3_),
      defaultK1(k1_),
      defaultK2(k2_),
      rlsLambda(0.99999f),
      rlsDelta(1e-5f),
      powerMeter(nullptr),
      taskCreated(false)
{
    motors[0] = motorRf;
    motors[1] = motorLf;
    motors[2] = motorLb;
    motors[3] = motorRb;

    // 初始化电机断开计数器
    for (int i = 0; i < 4; i++) {
        motorDisconnectCounter[i] = 0U;
    }

    // 初始化电源状态
    powerStatus.userConfiguredMaxPower = 0.0f;
    powerStatus.maxPowerLimited        = 0.0f;
    powerStatus.sumPowerCmdBeforeClamp = 0.0f;
    powerStatus.effectivePower         = 0.0f;
    powerStatus.powerLoss              = 0.0f;
    powerStatus.efficiency             = 0.0f;

    // 初始化RLS滤波器，使用工厂函数
    rlsFilter = createFirstOrderRLS(rlsLambda, rlsDelta);
    // 设置初始参数
    RLSFilter2D::ParameterVector initialParams;
    initialParams << k1, k2;
    rlsFilter.setInitialParameters(initialParams);

    // 如果用户未配置最大功率，使用上限
    if (userConfiguredMaxPower <= 0.0f) {
        setMaxPowerConfigured(powerUpperLimit);
    }

}

void PowerManager::init()
{
    if (taskCreated) {
        return;
    }

    // 如用户未配置最大功率，使用上限
    if (userConfiguredMaxPower <= 0.0f) {
        setMaxPowerConfigured(powerUpperLimit);
    }

    // 创建电源守护任务
    static StackType_t xPowerTaskStack[1024];
    static StaticTask_t uxPowerTaskTCB;
    xTaskCreateStatic(powerDaemonWrapper, "power", 1024, this, 10, xPowerTaskStack, &uxPowerTaskTCB);
    taskCreated = true;
}

void PowerManager::attachPowerMeter(PowerMeter *meter)
{
    powerMeter = meter;
}


// 设置配置的最大功率
void PowerManager::setMaxPowerConfigured(fp32 maxPower)
{
    userConfiguredMaxPower = clampRange(maxPower, minMaxPower, powerUpperLimit);
}

// 设置RLS使能
void PowerManager::setRLSEnabled(uint8_t enable)
{
    rlsEnabled = enable;
}

// 注册电源回调函数
void PowerManager::registerPowerCallbackFunc(fp32 (*callback)(void))
{
    this->callback = callback;
}

// 获取电源状态
const PowerManager::PowerStatus &PowerManager::getPowerStatus()
{
    static PowerStatus snapshot;
    taskENTER_CRITICAL();
    snapshot = powerStatus;
    taskEXIT_CRITICAL();
    return snapshot;
}

fp32 PowerManager::getK1() const
{
    fp32 value;
    taskENTER_CRITICAL();
    value = k1;
    taskEXIT_CRITICAL();
    return value;
}

fp32 PowerManager::getK2() const
{
    fp32 value;
    taskENTER_CRITICAL();
    value = k2;
    taskEXIT_CRITICAL();
    return value;
}

// 获取控制输出
float *PowerManager::getControlledOutput(PowerObj *objs[4])
{
    static fp32 newTorqueCurrent[4];
    fp32 k0[4]       = {0.0f, 0.0f, 0.0f, 0.0f};
    bool objValid[4] = {false, false, false, false};
    bool k0Valid[4]  = {false, false, false, false};
    fp32 k1Local     = 0.0f;
    fp32 k2Local     = 0.0f;
    fp32 k3Local     = 0.0f;

    // 读取功率模型参数，避免并发更新导致不一致
    taskENTER_CRITICAL();
    k1Local = k1;
    k2Local = k2;
    k3Local = k3;
    taskEXIT_CRITICAL();

    for (int i = 0; i < 4; i++) {
        newTorqueCurrent[i] = 0.0f;
        objValid[i]         = (objs[i] != nullptr);
        if (motors[i] != nullptr &&
            motors[i]->getOutputLimit() > 0.0f &&
            motors[i]->getCurrentLimit() > 0.0f &&
            motors[i]->getKA() > 0.0f) {
            k0[i]      = motors[i]->getKA() * motors[i]->getGearboxRatio() *
                     motors[i]->getCurrentLimit() / motors[i]->getOutputLimit();
            k0Valid[i] = k0[i] > 0.0f;
        }
    }

    fp32 sumCmdPower = 0.0f;
    fp32 cmdPower[4];

    fp32 sumError = 0.0f;
    fp32 error[4];

    fp32 maxPower = clampRange(userConfiguredMaxPower, minMaxPower, powerUpperLimit);

    fp32 allocatablePower = maxPower;
    fp32 sumPowerRequired = 0.0f;

    for (int i = 0; i < 4; i++) {
        if (motors[i] != nullptr && motors[i]->isMotorConnected() == true && objValid[i] && k0Valid[i]) {
            PowerObj *p = objs[i];
            cmdPower[i] = p->pidOutput * k0[i] * p->currentAngularVelocity +
                          fabsf(p->currentAngularVelocity) * k1Local +
                          p->pidOutput * k0[i] * p->pidOutput * k0[i] * k2Local +
                          k3Local / 4.0f;
            sumCmdPower += cmdPower[i];
            error[i] = fabsf(p->setPointAngularVelocity - p->currentAngularVelocity);
            if (floatEqual(cmdPower[i], 0.0f) || cmdPower[i] < 0.0f) {
                allocatablePower += -cmdPower[i];
            } else {
                sumError += error[i];
                sumPowerRequired += cmdPower[i];
            }
        } else if (motors[i] != nullptr && motorDisconnectCounter[i] < 1000U) {
            fp32 torque = motors[i]->getTorqueFeedback();
            fp32 av     = rpm2av(motors[i]->getRPMFeedback());
            cmdPower[i] = torque * av + fabsf(av) * k1Local + torque * torque * k2Local + k3Local / 4.0f;
            error[i]    = 0.0f;
        } else {
            cmdPower[i] = 0.0f;
            error[i]    = 0.0f;
        }
    }

    taskENTER_CRITICAL();
    powerStatus.maxPowerLimited        = maxPower;
    powerStatus.sumPowerCmdBeforeClamp = sumCmdPower;
    taskEXIT_CRITICAL();

    if (sumCmdPower > maxPower) {
        fp32 errorConfidence = 0.0f;
        if (sumError > ERROR_POWER_DISTRIBUTION_SET) {
            errorConfidence = 1.0f;
        } else if (sumError > PROP_POWER_DISTRIBUTION_SET) {
            errorConfidence =
                clampRange((sumError - PROP_POWER_DISTRIBUTION_SET) / (ERROR_POWER_DISTRIBUTION_SET - PROP_POWER_DISTRIBUTION_SET), 0.0f, 1.0f);
        }

        for (int i = 0; i < 4; i++) {
            PowerObj *p = objs[i];
            if (motors[i] != nullptr && motors[i]->isMotorConnected() == true && objValid[i] && k0Valid[i]) {
                if (floatEqual(cmdPower[i], 0.0f) || cmdPower[i] < 0.0f) {
                    newTorqueCurrent[i] = p->pidOutput;
                    continue;
                }
                fp32 powerWeightError = (sumError > 1e-5f) ? (fabsf(p->setPointAngularVelocity - p->currentAngularVelocity) / sumError) : 0.0f;
                fp32 powerWeightProp  = (sumPowerRequired > 1e-5f) ? (cmdPower[i] / sumPowerRequired) : 0.0f;
                fp32 powerWeight      = errorConfidence * powerWeightError + (1.0f - errorConfidence) * powerWeightProp;
                fp32 delta            = p->currentAngularVelocity * p->currentAngularVelocity -
                             4.0f * k2Local * (k1Local * fabsf(p->currentAngularVelocity) + k3Local / 4.0f - powerWeight * allocatablePower);
                if (floatEqual(delta, 0.0f)) {
                    newTorqueCurrent[i] = -p->currentAngularVelocity / (2.0f * k2Local) / k0[i];
                } else if (delta > 0.0f) {
                    newTorqueCurrent[i] =
                        p->pidOutput > 0.0f ? (-p->currentAngularVelocity + sqrtf(delta)) / (2.0f * k2Local) / k0[i]
                                            : (-p->currentAngularVelocity - sqrtf(delta)) / (2.0f * k2Local) / k0[i];
                } else {
                    newTorqueCurrent[i] = -p->currentAngularVelocity / (2.0f * k2Local) / k0[i];
                }
                newTorqueCurrent[i] = clampAbs(newTorqueCurrent[i], p->pidMaxOutput);
            } else if (objValid[i]) {
                // 未提供完整电机参数时，保持原输出
                newTorqueCurrent[i] = objs[i]->pidOutput;
            } else {
                // 未提供PowerObj时，防止空指针异常
                newTorqueCurrent[i] = 0.0f;
            }
        }
    } else {
        for (int i = 0; i < 4; i++) {
            if (motors[i] != nullptr && motors[i]->isMotorConnected() == true && objValid[i]) {
                newTorqueCurrent[i] = objs[i]->pidOutput;
            } else {
                newTorqueCurrent[i] = 0.0f;
            }
        }
    }

    return newTorqueCurrent;
}

// 电源守护任务
void PowerManager::powerDaemon(void *pvParam)
{
    PowerManager *self  = static_cast<PowerManager *>(pvParam);
    fp32 effectivePower = 0.0f;

    vTaskDelay(pdMS_TO_TICKS(1000));
    self->lastUpdateTick = (uint32_t)xTaskGetTickCount();

    while (true) {
        if (self->callback != nullptr) {
            self->setMaxPowerConfigured(self->callback());
        }

        effectivePower = 0.0f;
        fp32 sampleAv  = 0.0f;
        fp32 sampleT2  = 0.0f;

        for (int i = 0; i < 4; i++) {
            if (self->motors[i] != nullptr && self->motors[i]->isMotorConnected() == true) {
                self->motorDisconnectCounter[i] = 0U;
            } else {
                self->motorDisconnectCounter[i]++;
            }

            if (self->motors[i] != nullptr && self->motorDisconnectCounter[i] < 1000U) {
                fp32 torque = self->motors[i]->getTorqueFeedback();
                fp32 av     = rpm2av(self->motors[i]->getRPMFeedback());
                effectivePower += torque * av;
                sampleAv += fabsf(av);
                sampleT2 += torque * torque;
            } else {
                self->motorDisconnectCounter[i] = 1000U;
            }
        }

        self->estimatedPower = self->k1 * sampleAv + self->k2 * sampleT2 + effectivePower + self->k3;
        bool meterConnected = (self->powerMeter != nullptr && self->powerMeter->isConnected());
        // 根据功率计在线状态自动启用RLS更新
        self->rlsEnabled = meterConnected ? 1U : 0U;

        if (meterConnected) {
            self->measuredPower = self->powerMeter->getPower();
        } else {
            // 功率计断连时回退到默认参数
            self->measuredPower = self->estimatedPower;
            if (self->rlsEnabled != 0U) {
                taskENTER_CRITICAL();
                self->k1 = self->defaultK1;
                self->k2 = self->defaultK2;
                // 重新创建RLS滤波器
                self->rlsFilter = createFirstOrderRLS(self->rlsLambda, self->rlsDelta);
                // 设置初始参数
                RLSFilter2D::ParameterVector initialParams;
                initialParams << self->k1, self->k2;
                self->rlsFilter.setInitialParameters(initialParams);
                taskEXIT_CRITICAL();
            }
        }

        if (self->rlsEnabled != 0U && meterConnected && fabsf(self->measuredPower) > 5.0f) {
            // 使用功率计实测功率更新k1/k2
            fp32 output = self->measuredPower - effectivePower - self->k3;
            // 创建输入向量
            RLSFilter2D::InputVector inputVec;
            inputVec << sampleAv, sampleT2;
            // 更新RLS滤波器
            self->rlsFilter.update(inputVec, output);
            // 获取更新后的参数
            taskENTER_CRITICAL();
            const auto &params = self->rlsFilter.getParameters();
            self->k1 = fmaxf(params(0), 1e-5f);
            self->k2 = fmaxf(params(1), 1e-5f);
            taskEXIT_CRITICAL();
        }

        taskENTER_CRITICAL();
        self->powerStatus.userConfiguredMaxPower = self->userConfiguredMaxPower;
        self->powerStatus.effectivePower         = effectivePower;
        self->powerStatus.powerLoss              = self->measuredPower - effectivePower;
        if (self->measuredPower > 1e-5f) {
            self->powerStatus.efficiency = clampRange(effectivePower / self->measuredPower, 0.0f, 1.0f);
        } else {
            self->powerStatus.efficiency = 0.0f;
        }
        taskEXIT_CRITICAL();

        self->lastUpdateTick = (uint32_t)xTaskGetTickCount();
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

