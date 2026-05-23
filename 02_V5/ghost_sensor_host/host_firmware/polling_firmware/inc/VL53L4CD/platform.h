/**
 ******************************************************************************
 * @file    platform.h
 * @brief   VL53L4CD driver class for Raspberry Pi Pico SDK.
 *
 * Copyright (c) 2023 STMicroelectronics – original API contract.
 * Pico SDK adaptation: public domain / your project license.
 ******************************************************************************
 */

#pragma once

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include "I2CBus.h"

// ---- Driver version ---------------------------------------------------------
#define VL53L4CD_IMPLEMENTATION_VER_MAJOR       2
#define VL53L4CD_IMPLEMENTATION_VER_MINOR       2
#define VL53L4CD_IMPLEMENTATION_VER_BUILD       3
#define VL53L4CD_IMPLEMENTATION_VER_REVISION    0

// ---- Error codes ------------------------------------------------------------
typedef uint8_t VL53L4CD_Error;

#define VL53L4CD_ERROR_NONE             ((uint8_t)0U)
#define VL53L4CD_ERROR_XTALK_FAILED     ((uint8_t)253U)
#define VL53L4CD_ERROR_INVALID_ARGUMENT ((uint8_t)254U)
#define VL53L4CD_ERROR_TIMEOUT          ((uint8_t)255U)

// ---- Register addresses -----------------------------------------------------
#define VL53L4CD_SOFT_RESET                             ((uint16_t)0x0000)
#define VL53L4CD_I2C_SLAVE__DEVICE_ADDRESS              ((uint16_t)0x0001)
#define VL53L4CD_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND  ((uint16_t)0x0008)
#define VL53L4CD_XTALK_PLANE_OFFSET_KCPS                ((uint16_t)0x0016)
#define VL53L4CD_XTALK_X_PLANE_GRADIENT_KCPS            ((uint16_t)0x0018)
#define VL53L4CD_XTALK_Y_PLANE_GRADIENT_KCPS            ((uint16_t)0x001A)
#define VL53L4CD_RANGE_OFFSET_MM                        ((uint16_t)0x001E)
#define VL53L4CD_INNER_OFFSET_MM                        ((uint16_t)0x0020)
#define VL53L4CD_OUTER_OFFSET_MM                        ((uint16_t)0x0022)
#define VL53L4CD_GPIO_HV_MUX__CTRL                      ((uint16_t)0x0030)
#define VL53L4CD_GPIO__TIO_HV_STATUS                    ((uint16_t)0x0031)
#define VL53L4CD_SYSTEM__INTERRUPT                      ((uint16_t)0x0046)
#define VL53L4CD_RANGE_CONFIG_A                         ((uint16_t)0x005E)
#define VL53L4CD_RANGE_CONFIG_B                         ((uint16_t)0x0061)
#define VL53L4CD_RANGE_CONFIG__SIGMA_THRESH             ((uint16_t)0x0064)
#define VL53L4CD_MIN_COUNT_RATE_RTN_LIMIT_MCPS          ((uint16_t)0x0066)
#define VL53L4CD_INTERMEASUREMENT_MS                    ((uint16_t)0x006C)
#define VL53L4CD_THRESH_HIGH                            ((uint16_t)0x0072)
#define VL53L4CD_THRESH_LOW                             ((uint16_t)0x0074)
#define VL53L4CD_SYSTEM__INTERRUPT_CLEAR                ((uint16_t)0x0086)
#define VL53L4CD_SYSTEM_START                           ((uint16_t)0x0087)
#define VL53L4CD_RESULT__RANGE_STATUS                   ((uint16_t)0x0089)
#define VL53L4CD_RESULT__SPAD_NB                        ((uint16_t)0x008C)
#define VL53L4CD_RESULT__SIGNAL_RATE                    ((uint16_t)0x008E)
#define VL53L4CD_RESULT__AMBIENT_RATE                   ((uint16_t)0x0090)
#define VL53L4CD_RESULT__SIGMA                          ((uint16_t)0x0092)
#define VL53L4CD_RESULT__DISTANCE                       ((uint16_t)0x0096)
#define VL53L4CD_RESULT__OSC_CALIBRATE_VAL              ((uint16_t)0x00DE)
#define VL53L4CD_FIRMWARE__SYSTEM_STATUS                ((uint16_t)0x00E5)
#define VL53L4CD_IDENTIFICATION__MODEL_ID               ((uint16_t)0x010F)

// ---- Data types -------------------------------------------------------------

typedef struct {
    uint8_t  major;
    uint8_t  minor;
    uint8_t  build;
    uint32_t revision;
} VL53L4CD_Version_t;

typedef struct {
    uint8_t  range_status;
    uint16_t distance_mm;
    uint32_t ambient_rate_kcps;
    uint32_t ambient_per_spad_kcps;
    uint32_t signal_rate_kcps;
    uint32_t signal_per_spad_kcps;
    uint16_t number_of_spad;
    uint16_t sigma_mm;
} VL53L4CD_ResultsData_t;

// ---- Driver class -----------------------------------------------------------

class VL53L4CD {
public:
    VL53L4CD() = default;

    VL53L4CD_Error init(I2CBus *i2c, uint8_t address);

    // Sensor setup
    VL53L4CD_Error GetSWVersion(VL53L4CD_Version_t *pVersion);
    VL53L4CD_Error SetI2CAddress(uint8_t new_address);
    VL53L4CD_Error GetSensorId(uint16_t *p_id);
    VL53L4CD_Error SensorInit();

    // Ranging control
    VL53L4CD_Error ClearInterrupt();
    VL53L4CD_Error StartRanging();
    VL53L4CD_Error StopRanging();
    VL53L4CD_Error CheckForDataReady(uint8_t *p_is_data_ready);
    VL53L4CD_Error GetResult(VL53L4CD_ResultsData_t *p_result);

    // Configuration
    VL53L4CD_Error SetRangeTiming(uint32_t timing_budget_ms, uint32_t inter_measurement_ms);
    VL53L4CD_Error GetRangeTiming(uint32_t *p_timing_budget_ms, uint32_t *p_inter_measurement_ms);
    VL53L4CD_Error SetOffset(int16_t OffsetValueInMm);
    VL53L4CD_Error GetOffset(int16_t *p_offset);
    VL53L4CD_Error SetXtalk(uint16_t XtalkValueKcps);
    VL53L4CD_Error GetXtalk(uint16_t *p_xtalk_kcps);
    VL53L4CD_Error SetDetectionThresholds(uint16_t distance_low_mm, uint16_t distance_high_mm, uint8_t window);
    VL53L4CD_Error GetDetectionThresholds(uint16_t *p_distance_low_mm, uint16_t *p_distance_high_mm, uint8_t *p_window);
    VL53L4CD_Error SetSignalThreshold(uint16_t signal_kcps);
    VL53L4CD_Error GetSignalThreshold(uint16_t *p_signal_kcps);
    VL53L4CD_Error SetSigmaThreshold(uint16_t sigma_mm);
    VL53L4CD_Error GetSigmaThreshold(uint16_t *p_sigma_mm);
    VL53L4CD_Error StartTemperatureUpdate();

    // Calibration
    VL53L4CD_Error CalibrateOffset(int16_t TargetDistInMm, int16_t *p_measured_offset_mm, int16_t nb_samples);
    VL53L4CD_Error CalibrateXtalk(int16_t TargetDistInMm, uint16_t *p_measured_xtalk_kcps, int16_t nb_samples);

private:
    I2CBus *i2c_  = nullptr;
    uint16_t address_ = 0;

    uint8_t I2CWrite(uint16_t registerAddress, uint8_t *p_values, uint32_t size);
    uint8_t I2CRead(uint16_t registerAddress, uint8_t *p_values, uint32_t size);
    uint8_t RdDWord(uint16_t reg, uint32_t *value);
    uint8_t RdWord(uint16_t reg, uint16_t *value);
    uint8_t RdByte(uint16_t reg, uint8_t *value);
    uint8_t WrByte(uint16_t reg, uint8_t value);
    uint8_t WrWord(uint16_t reg, uint16_t value);
    uint8_t WrDWord(uint16_t reg, uint32_t value);
    uint8_t WaitMs(uint32_t TimeMs);
};
