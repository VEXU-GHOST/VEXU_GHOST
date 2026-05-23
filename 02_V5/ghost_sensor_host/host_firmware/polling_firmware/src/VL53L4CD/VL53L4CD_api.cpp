/**
  * Copyright (c) 2023 STMicroelectronics. All rights reserved.
  ******************************************************************************
  */

#include "platform.h"

static const uint8_t VL53L4CD_DEFAULT_CONFIGURATION[] = {
#ifdef VL53L4CD_I2C_FAST_MODE_PLUS
    0x12,
#else
    0x00,
#endif
    0x00, 0x00, 0x11, 0x02, 0x00, 0x02, 0x08, 0x00, 0x08, 0x10,
    0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0xff, 0x00, 0x0F, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x20, 0x0b, 0x00, 0x00, 0x02, 0x14,
    0x21, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0xc8, 0x00,
    0x00, 0x38, 0xff, 0x01, 0x00, 0x08, 0x00, 0x00, 0x01, 0xcc,
    0x07, 0x01, 0xf1, 0x05, 0x00, 0xa0, 0x00, 0x80, 0x08, 0x38,
    0x00, 0x00, 0x00, 0x00, 0x0f, 0x89, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x01, 0x07, 0x05, 0x06, 0x06, 0x00, 0x00,
    0x02, 0xc7, 0xff, 0x9B, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00,
};

VL53L4CD_Error VL53L4CD::GetSWVersion(VL53L4CD_Version_t *p_Version)
{
    p_Version->major    = VL53L4CD_IMPLEMENTATION_VER_MAJOR;
    p_Version->minor    = VL53L4CD_IMPLEMENTATION_VER_MINOR;
    p_Version->build    = VL53L4CD_IMPLEMENTATION_VER_BUILD;
    p_Version->revision = VL53L4CD_IMPLEMENTATION_VER_REVISION;
    return VL53L4CD_ERROR_NONE;
}

VL53L4CD_Error VL53L4CD::SetI2CAddress(uint8_t new_address)
{
    return WrByte(VL53L4CD_I2C_SLAVE__DEVICE_ADDRESS,
                  (uint8_t)(new_address >> (uint8_t)1));
}

VL53L4CD_Error VL53L4CD::GetSensorId(uint16_t *p_id)
{
    return RdWord(VL53L4CD_IDENTIFICATION__MODEL_ID, p_id);
}

VL53L4CD_Error VL53L4CD::SensorInit()
{
    VL53L4CD_Error status = VL53L4CD_ERROR_NONE;
    uint8_t tmp;
    uint8_t continue_loop = 1;
    uint16_t i = 0;

    do {
        status |= RdByte(VL53L4CD_FIRMWARE__SYSTEM_STATUS, &tmp);
        if (tmp == (uint8_t)0x3) {
            continue_loop = 0;
        } else if (i < (uint16_t)1000) {
            i++;
        } else {
            continue_loop = 0;
            status |= (uint8_t)VL53L4CD_ERROR_TIMEOUT;
        }
        WaitMs(1);
    } while (continue_loop == (uint8_t)1);

    for (uint8_t Addr = (uint8_t)0x2D; Addr <= (uint8_t)0x87; Addr++) {
        status |= WrByte(Addr, VL53L4CD_DEFAULT_CONFIGURATION[Addr - (uint8_t)0x2D]);
    }

    /* Start VHV */
    status |= WrByte(VL53L4CD_SYSTEM_START, (uint8_t)0x40);
    i = 0;
    continue_loop = 1;
    do {
        status |= CheckForDataReady(&tmp);
        if (tmp == (uint8_t)1) {
            continue_loop = 0;
        } else if (i < (uint16_t)1000) {
            i++;
        } else {
            continue_loop = 0;
            status |= (uint8_t)VL53L4CD_ERROR_TIMEOUT;
        }
        WaitMs(1);
    } while (continue_loop == (uint8_t)1);

    status |= ClearInterrupt();
    status |= StopRanging();
    status |= WrByte(VL53L4CD_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, (uint8_t)0x09);
    status |= WrByte(0x0B, (uint8_t)0);
    status |= WrWord(0x0024, 0x500);
    status |= SetRangeTiming(50, 0);

    return status;
}

VL53L4CD_Error VL53L4CD::init(I2CBus *i2c, uint8_t address)
{
    i2c_     = i2c;
    address_ = address;

    uint16_t sensor_id = 0;
    VL53L4CD_Error status = GetSensorId(&sensor_id);
    if (status != VL53L4CD_ERROR_NONE)
        return status;
    if (sensor_id != (uint16_t)0xEBAA)
        return VL53L4CD_ERROR_TIMEOUT;

    return SensorInit();
}

VL53L4CD_Error VL53L4CD::ClearInterrupt()
{
    return WrByte(VL53L4CD_SYSTEM__INTERRUPT_CLEAR, 0x01);
}

VL53L4CD_Error VL53L4CD::StartRanging()
{
    VL53L4CD_Error status = VL53L4CD_ERROR_NONE;
    uint32_t tmp;

    status |= RdDWord(VL53L4CD_INTERMEASUREMENT_MS, &tmp);
    if (tmp == (uint32_t)0)
        status |= WrByte(VL53L4CD_SYSTEM_START, 0x21);
    else
        status |= WrByte(VL53L4CD_SYSTEM_START, 0x40);

    return status;
}

VL53L4CD_Error VL53L4CD::StopRanging()
{
    return WrByte(VL53L4CD_SYSTEM_START, 0x80);
}

VL53L4CD_Error VL53L4CD::CheckForDataReady(uint8_t *p_is_data_ready)
{
    VL53L4CD_Error status = VL53L4CD_ERROR_NONE;
    uint8_t temp, int_pol;

    status |= RdByte(VL53L4CD_GPIO_HV_MUX__CTRL, &temp);
    temp = temp & (uint8_t)0x10;
    temp = temp >> 4;
    int_pol = (temp == (uint8_t)1) ? (uint8_t)0 : (uint8_t)1;

    status |= RdByte(VL53L4CD_GPIO__TIO_HV_STATUS, &temp);
    *p_is_data_ready = ((temp & (uint8_t)1) == int_pol) ? (uint8_t)1 : (uint8_t)0;

    return status;
}

VL53L4CD_Error VL53L4CD::SetRangeTiming(uint32_t timing_budget_ms,
                                         uint32_t inter_measurement_ms)
{
    VL53L4CD_Error status = VL53L4CD_ERROR_NONE;
    uint16_t clock_pll, osc_frequency, ms_byte;
    uint32_t macro_period_us = 0, timing_budget_us = 0, ls_byte, tmp;
    float_t inter_measurement_factor = (float_t)1.055;

    status |= RdWord(0x0006, &osc_frequency);
    if (osc_frequency != (uint16_t)0) {
        timing_budget_us = timing_budget_ms * (uint32_t)1000;
        macro_period_us = (uint32_t)((uint32_t)2304 *
            ((uint32_t)0x40000000 / (uint32_t)osc_frequency)) >> 6;
    } else {
        status |= (uint8_t)VL53L4CD_ERROR_INVALID_ARGUMENT;
    }

    if ((timing_budget_ms < (uint32_t)10)
            || (timing_budget_ms > (uint32_t)200) || (status != (uint8_t)0)) {
        status |= VL53L4CD_ERROR_INVALID_ARGUMENT;
    } else if (inter_measurement_ms == (uint32_t)0) {
        status |= WrDWord(VL53L4CD_INTERMEASUREMENT_MS, 0);
        timing_budget_us -= (uint32_t)2500;
    } else if (inter_measurement_ms > timing_budget_ms) {
        status |= RdWord(VL53L4CD_RESULT__OSC_CALIBRATE_VAL, &clock_pll);
        clock_pll = clock_pll & (uint16_t)0x3FF;
        inter_measurement_factor = inter_measurement_factor
            * (float_t)inter_measurement_ms
            * (float_t)clock_pll;
        status |= WrDWord(VL53L4CD_INTERMEASUREMENT_MS,
                          (uint32_t)inter_measurement_factor);
        timing_budget_us -= (uint32_t)4300;
        timing_budget_us /= (uint32_t)2;
    } else {
        status |= (uint8_t)VL53L4CD_ERROR_INVALID_ARGUMENT;
    }

    if (status != (uint8_t)VL53L4CD_ERROR_INVALID_ARGUMENT) {
        ms_byte = 0;
        timing_budget_us = timing_budget_us << 12;
        tmp = macro_period_us * (uint32_t)16;
        ls_byte = ((timing_budget_us + ((tmp >> 6) >> 1)) / (tmp >> 6)) - (uint32_t)1;
        while ((ls_byte & 0xFFFFFF00U) > 0U) { ls_byte >>= 1; ms_byte++; }
        ms_byte = (uint16_t)(ms_byte << 8) + (uint16_t)(ls_byte & (uint32_t)0xFF);
        status |= WrWord(VL53L4CD_RANGE_CONFIG_A, ms_byte);

        ms_byte = 0;
        tmp = macro_period_us * (uint32_t)12;
        ls_byte = ((timing_budget_us + ((tmp >> 6) >> 1)) / (tmp >> 6)) - (uint32_t)1;
        while ((ls_byte & 0xFFFFFF00U) > 0U) { ls_byte >>= 1; ms_byte++; }
        ms_byte = (uint16_t)(ms_byte << 8) + (uint16_t)(ls_byte & (uint32_t)0xFF);
        status |= WrWord(VL53L4CD_RANGE_CONFIG_B, ms_byte);
    }

    return status;
}

VL53L4CD_Error VL53L4CD::GetRangeTiming(uint32_t *p_timing_budget_ms,
                                          uint32_t *p_inter_measurement_ms)
{
    VL53L4CD_Error status = VL53L4CD_ERROR_NONE;
    uint16_t osc_frequency = 1, range_config_macrop_high, clock_pll = 1;
    uint32_t tmp, ls_byte, ms_byte, macro_period_us;
    float_t clock_pll_factor = (float_t)1.065;

    status |= RdDWord(VL53L4CD_INTERMEASUREMENT_MS, &tmp);
    status |= RdWord(VL53L4CD_RESULT__OSC_CALIBRATE_VAL, &clock_pll);
    clock_pll = clock_pll & (uint16_t)0x3FF;
    clock_pll_factor = clock_pll_factor * (float_t)clock_pll;
    clock_pll = (uint16_t)clock_pll_factor;
    *p_inter_measurement_ms = (uint16_t)(tmp / (uint32_t)clock_pll);

    status |= RdWord(0x0006, &osc_frequency);
    status |= RdWord(VL53L4CD_RANGE_CONFIG_A, &range_config_macrop_high);

    macro_period_us = (uint32_t)((uint32_t)2304 *
        ((uint32_t)0x40000000 / (uint32_t)osc_frequency)) >> 6;
    ls_byte = (range_config_macrop_high & (uint32_t)0x00FF) << 4;
    ms_byte = (range_config_macrop_high & (uint32_t)0xFF00) >> 8;
    ms_byte = (uint32_t)0x04 - (ms_byte - (uint32_t)1) - (uint32_t)1;

    macro_period_us = macro_period_us * (uint32_t)16;
    *p_timing_budget_ms = (((ls_byte + (uint32_t)1) * (macro_period_us >> 6))
        - ((macro_period_us >> 6) >> 1)) >> 12;

    if (ms_byte < (uint8_t)12)
        *p_timing_budget_ms = (uint32_t)(*p_timing_budget_ms >> (uint8_t)ms_byte);

    if (tmp == (uint32_t)0) {
        *p_timing_budget_ms += (uint32_t)2500;
    } else {
        *p_timing_budget_ms *= (uint32_t)2;
        *p_timing_budget_ms += (uint32_t)4300;
    }
    *p_timing_budget_ms = *p_timing_budget_ms / (uint32_t)1000;

    return status;
}

VL53L4CD_Error VL53L4CD::GetResult(VL53L4CD_ResultsData_t *p_result)
{
    VL53L4CD_Error status = VL53L4CD_ERROR_NONE;
    uint16_t temp_16;
    uint8_t  temp_8;
    uint16_t raw_spads;

    static const uint8_t status_rtn[24] = {
        255, 255, 255, 5, 2, 4, 1, 7, 3, 0,
        255, 255, 9, 13, 255, 255, 255, 255, 10, 6,
        255, 255, 11, 12
    };

    status |= RdByte(VL53L4CD_RESULT__RANGE_STATUS, &temp_8);
    temp_8 = temp_8 & (uint8_t)0x1F;
    if (temp_8 < (uint8_t)24)
        temp_8 = status_rtn[temp_8];
    p_result->range_status = temp_8;

    status |= RdWord(VL53L4CD_RESULT__SPAD_NB, &temp_16);
    raw_spads = temp_16;
    p_result->number_of_spad = temp_16 / (uint16_t)256;

    status |= RdWord(VL53L4CD_RESULT__SIGNAL_RATE, &temp_16);
    p_result->signal_rate_kcps = (uint32_t)temp_16 * 8;

    status |= RdWord(VL53L4CD_RESULT__AMBIENT_RATE, &temp_16);
    p_result->ambient_rate_kcps = (uint32_t)temp_16 * 8;

    status |= RdWord(VL53L4CD_RESULT__SIGMA, &temp_16);
    p_result->sigma_mm = temp_16 / (uint16_t)4;

    status |= RdWord(VL53L4CD_RESULT__DISTANCE, &temp_16);
    p_result->distance_mm = temp_16;

    p_result->signal_per_spad_kcps  = p_result->signal_rate_kcps  * 256 / (uint32_t)raw_spads;
    p_result->ambient_per_spad_kcps = p_result->ambient_rate_kcps * 256 / (uint32_t)raw_spads;

    return status;
}

VL53L4CD_Error VL53L4CD::SetOffset(int16_t OffsetValueInMm)
{
    VL53L4CD_Error status = VL53L4CD_ERROR_NONE;
    uint16_t temp = (uint16_t)((uint16_t)OffsetValueInMm * (uint16_t)4);
    status |= WrWord(VL53L4CD_RANGE_OFFSET_MM, temp);
    status |= WrWord(VL53L4CD_INNER_OFFSET_MM, (uint8_t)0x0);
    status |= WrWord(VL53L4CD_OUTER_OFFSET_MM, (uint8_t)0x0);
    return status;
}

VL53L4CD_Error VL53L4CD::GetOffset(int16_t *p_offset)
{
    VL53L4CD_Error status = VL53L4CD_ERROR_NONE;
    uint16_t temp;

    status |= RdWord(VL53L4CD_RANGE_OFFSET_MM, &temp);
    temp = temp << 3;
    temp = temp >> 5;
    *p_offset = (int16_t)temp;
    if (*p_offset > 1024)
        *p_offset = *p_offset - 2048;

    return status;
}

VL53L4CD_Error VL53L4CD::SetXtalk(uint16_t XtalkValueKcps)
{
    VL53L4CD_Error status = VL53L4CD_ERROR_NONE;
    status |= WrWord(VL53L4CD_XTALK_X_PLANE_GRADIENT_KCPS, 0x0000);
    status |= WrWord(VL53L4CD_XTALK_Y_PLANE_GRADIENT_KCPS, 0x0000);
    status |= WrWord(VL53L4CD_XTALK_PLANE_OFFSET_KCPS, (XtalkValueKcps << 9));
    return status;
}

VL53L4CD_Error VL53L4CD::GetXtalk(uint16_t *p_xtalk_kcps)
{
    VL53L4CD_Error status = VL53L4CD_ERROR_NONE;
    status |= RdWord(VL53L4CD_XTALK_PLANE_OFFSET_KCPS, p_xtalk_kcps);
    float_t tmp_xtalk = (float_t)*p_xtalk_kcps / (float_t)512.0;
    *p_xtalk_kcps = (uint16_t)(round(tmp_xtalk));
    return status;
}

VL53L4CD_Error VL53L4CD::SetDetectionThresholds(uint16_t distance_low_mm,
                                                  uint16_t distance_high_mm,
                                                  uint8_t window)
{
    VL53L4CD_Error status = VL53L4CD_ERROR_NONE;
    status |= WrByte(VL53L4CD_SYSTEM__INTERRUPT, window);
    status |= WrWord(VL53L4CD_THRESH_HIGH, distance_high_mm);
    status |= WrWord(VL53L4CD_THRESH_LOW, distance_low_mm);
    return status;
}

VL53L4CD_Error VL53L4CD::GetDetectionThresholds(uint16_t *p_distance_low_mm,
                                                  uint16_t *p_distance_high_mm,
                                                  uint8_t *p_window)
{
    VL53L4CD_Error status = VL53L4CD_ERROR_NONE;
    status |= RdWord(VL53L4CD_THRESH_HIGH, p_distance_high_mm);
    status |= RdWord(VL53L4CD_THRESH_LOW, p_distance_low_mm);
    status |= RdByte(VL53L4CD_SYSTEM__INTERRUPT, p_window);
    *p_window = (*p_window & (uint8_t)0x7);
    return status;
}

VL53L4CD_Error VL53L4CD::SetSignalThreshold(uint16_t signal_kcps)
{
    return WrWord(VL53L4CD_MIN_COUNT_RATE_RTN_LIMIT_MCPS, signal_kcps >> 3);
}

VL53L4CD_Error VL53L4CD::GetSignalThreshold(uint16_t *p_signal_kcps)
{
    VL53L4CD_Error status = VL53L4CD_ERROR_NONE;
    uint16_t tmp = 0;
    status |= RdWord(VL53L4CD_MIN_COUNT_RATE_RTN_LIMIT_MCPS, &tmp);
    *p_signal_kcps = tmp << 3;
    return status;
}

VL53L4CD_Error VL53L4CD::SetSigmaThreshold(uint16_t sigma_mm)
{
    if (sigma_mm > (uint16_t)((uint16_t)0xFFFF >> 2))
        return (uint8_t)VL53L4CD_ERROR_INVALID_ARGUMENT;
    return WrWord(VL53L4CD_RANGE_CONFIG__SIGMA_THRESH, sigma_mm << 2);
}

VL53L4CD_Error VL53L4CD::GetSigmaThreshold(uint16_t *p_sigma_mm)
{
    VL53L4CD_Error status = VL53L4CD_ERROR_NONE;
    status += RdWord(VL53L4CD_RANGE_CONFIG__SIGMA_THRESH, p_sigma_mm);
    *p_sigma_mm = *p_sigma_mm >> 2;
    return status;
}

VL53L4CD_Error VL53L4CD::StartTemperatureUpdate()
{
    VL53L4CD_Error status = VL53L4CD_ERROR_NONE;
    uint8_t tmp = 0, continue_loop = 1;
    uint16_t i = 0;

    status |= WrByte(VL53L4CD_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, (uint8_t)0x81);
    status |= WrByte(0x0B, (uint8_t)0x92);
    status |= StartRanging();

    do {
        status |= CheckForDataReady(&tmp);
        if (tmp == (uint8_t)1) {
            continue_loop = 0;
        } else if (i < (uint16_t)1000) {
            i++;
        } else {
            continue_loop = 0;
            status = (uint8_t)VL53L4CD_ERROR_TIMEOUT;
        }
        WaitMs(1);
    } while (continue_loop == (uint8_t)1);

    status |= ClearInterrupt();
    status |= StopRanging();
    status += WrByte(VL53L4CD_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, 0x09);
    status += WrByte(0x0B, 0);

    return status;
}
