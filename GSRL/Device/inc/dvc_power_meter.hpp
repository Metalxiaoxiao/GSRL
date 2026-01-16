/**
 ******************************************************************************
 * @file           : dvc_power_meter.hpp
 * @brief          : power meter device
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
#include "drv_can.h"

/**
 * @brief 功率计设备
 * @note  仅负责解析与缓存，不负责CAN初始化
 */
class PowerMeter
{
public:
    explicit PowerMeter(uint16_t canId = 0x212U);

    bool decodeCanRxMessageFromISR(const can_rx_message_t *rxMessage);
    bool decodeCanRxMessageFromQueue(const can_rx_message_t *rxMessage, uint8_t size);

    fp32 getVoltage() const;
    fp32 getCurrent() const;
    fp32 getPower() const;

    bool isConnected() const;
    uint32_t getLastUpdateTick() const;
    void setTimeoutMs(uint32_t timeoutMs);

private:
    uint16_t m_canId;
    fp32 m_voltage;
    fp32 m_current;
    fp32 m_power;
    uint32_t m_lastUpdateTick;
    uint32_t m_timeoutTicks;
};
