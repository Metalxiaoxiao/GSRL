/**
 ******************************************************************************
 * @file           : dvc_power_meter.cpp
 * @brief          : power meter device
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 GMaster
 * All rights reserved.
 *
 ******************************************************************************
 */

#include "dvc_power_meter.hpp"
#include "FreeRTOS.h"
#include "task.h"

#define POWER_METER_DEFAULT_TIMEOUT_MS 100U

PowerMeter::PowerMeter(uint16_t canId)
    : m_canId(canId),
      m_voltage(0.0f),
      m_current(0.0f),
      m_power(0.0f),
      m_lastUpdateTick(0U),
      m_timeoutTicks(pdMS_TO_TICKS(POWER_METER_DEFAULT_TIMEOUT_MS))
{
}

bool PowerMeter::decodeCanRxMessageFromISR(const can_rx_message_t *rxMessage)
{
    if (rxMessage == nullptr) {
        return false;
    }
    if (rxMessage->header.StdId != m_canId) {
        return false;
    }

    uint16_t vRaw = (static_cast<uint16_t>(rxMessage->data[1]) << 8) | rxMessage->data[0];
    uint16_t iRaw = (static_cast<uint16_t>(rxMessage->data[3]) << 8) | rxMessage->data[2];

    m_voltage        = static_cast<fp32>(vRaw) / 100.0f;
    m_current        = static_cast<fp32>(iRaw) / 100.0f;
    m_power          = m_voltage * m_current;
    m_lastUpdateTick = xTaskGetTickCountFromISR();
    return true;
}

bool PowerMeter::decodeCanRxMessageFromQueue(const can_rx_message_t *rxMessage, uint8_t size)
{
    if (rxMessage == nullptr || size == 0U) {
        return false;
    }
    for (uint8_t i = size; i > 0U; i--) {
        if (decodeCanRxMessageFromISR(&rxMessage[i - 1U])) {
            m_lastUpdateTick = xTaskGetTickCount();
            return true;
        }
    }
    return false;
}

fp32 PowerMeter::getVoltage() const
{
    return m_voltage;
}

fp32 PowerMeter::getCurrent() const
{
    return m_current;
}

fp32 PowerMeter::getPower() const
{
    return m_power;
}

bool PowerMeter::isConnected() const
{
    if (m_lastUpdateTick == 0U) {
        return false;
    }
    return (xTaskGetTickCount() - m_lastUpdateTick) <= m_timeoutTicks;
}

uint32_t PowerMeter::getLastUpdateTick() const
{
    return m_lastUpdateTick;
}

void PowerMeter::setTimeoutMs(uint32_t timeoutMs)
{
    m_timeoutTicks = pdMS_TO_TICKS(timeoutMs);
}
