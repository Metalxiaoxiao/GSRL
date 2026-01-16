/**
 ******************************************************************************
 * @file           : alg_power_control.hpp
 * @brief          : power control algorithm for mecanum chassis
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 GMaster
 * All rights reserved.
 *
 ******************************************************************************
 */

#pragma once

#include "gsrl_common.h"
#include "dvc_motor.hpp"
<<<<<<< Updated upstream
=======
#include "alg_filter.hpp"  // RLSFilter

>>>>>>> Stashed changes

#define DEFAULT_K1                   0.22f
#define DEFAULT_K2                   1.2f
#define DEFAULT_K3                   2.78f
#define DEFAULT_MIN_MAX_POWER        30.0f
#define ERROR_POWER_DISTRIBUTION_SET 20.0f
#define PROP_POWER_DISTRIBUTION_SET  15.0f

class PowerMeter;

class PowerManager
{
public:
    PowerManager(Motor *motorRf,
                 Motor *motorLf,
                 Motor *motorLb,
                 Motor *motorRb,
                 fp32 powerUpperLimit_,
                 fp32 k1_            = DEFAULT_K1,
                 fp32 k2_            = DEFAULT_K2,
                 fp32 k3_            = DEFAULT_K3,
                 fp32 minMaxPower_   = DEFAULT_MIN_MAX_POWER,
                 uint8_t rlsEnabled_ = 0);
    /**
     * @brief 初始化功率控制任务
     */
    void init();
    /**
     * @brief 绑定功率计设备
     * @param meter 功率计指针
     */
    void attachPowerMeter(PowerMeter *meter);
    Motor *motors[4];
    fp32 powerUpperLimit;
    fp32 minMaxPower;
    fp32 userConfiguredMaxPower;
    fp32 measuredPower;
    fp32 estimatedPower;
    uint32_t lastUpdateTick;
    uint8_t rlsEnabled; // TODO: 预留RLS自适应参数更新接口
    fp32 (*callback)(void);
    struct PowerStatus {
        fp32 userConfiguredMaxPower;
        fp32 maxPowerLimited;
        fp32 sumPowerCmdBeforeClamp;
        fp32 effectivePower;
        fp32 powerLoss;
        fp32 efficiency;
    };
    PowerStatus powerStatus;
    struct PowerObj {
        fp32 pidOutput;
        fp32 currentAngularVelocity;
        fp32 setPointAngularVelocity;
        fp32 pidMaxOutput;
    };

    void setMaxPowerConfigured(fp32 maxPower);
    void setRLSEnabled(uint8_t enable);
    void registerPowerCallbackFunc(fp32 (*callback)(void));
    float *getControlledOutput(PowerObj *objs[4]);
    const PowerStatus &getPowerStatus();
    fp32 getK1() const;
    fp32 getK2() const;
    static void powerDaemon(void *pvParam);

private:
    // 使用alg_filter.hpp中声明的RLSFilter2D类型别名
    // RLSFilter2D在alg_filter.hpp中声明为RLSFilter<fp32, 2>

    fp32 torqueConst;
    uint16_t motorDisconnectCounter[4];
    fp32 k1;
    fp32 k2;
    fp32 k3;
    fp32 defaultK1;
    fp32 defaultK2;
    fp32 rlsLambda;
    fp32 rlsDelta;
    PowerMeter *powerMeter;
    RLSFilter2D rlsFilter;  // 使用新的RLS滤波器
    bool taskCreated;
};
