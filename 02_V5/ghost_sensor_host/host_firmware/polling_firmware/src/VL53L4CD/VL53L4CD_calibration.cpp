/**
  * Copyright (c) 2023 STMicroelectronics. All rights reserved.
  ******************************************************************************
  */

#include "platform.h"

VL53L4CD_Error VL53L4CD::CalibrateOffset(int16_t TargetDistInMm,
                                           int16_t *p_measured_offset_mm,
                                           int16_t nb_samples)
{
    VL53L4CD_Error status = VL53L4CD_ERROR_NONE;
    uint8_t i, tmp, continue_loop;
    uint16_t j, tmpOff;
    int16_t AvgDistance = 0;
    VL53L4CD_ResultsData_t results;

    if (((nb_samples < (int16_t)5) || (nb_samples > (int16_t)255))
            || ((TargetDistInMm < (int16_t)10) || (TargetDistInMm > (int16_t)1000))) {
        return (uint8_t)VL53L4CD_ERROR_INVALID_ARGUMENT;
    }

    status |= WrWord(VL53L4CD_RANGE_OFFSET_MM, 0x0);
    status |= WrWord(VL53L4CD_INNER_OFFSET_MM, 0x0);
    status |= WrWord(VL53L4CD_OUTER_OFFSET_MM, 0x0);

    /* Device heat loop (10 samples) */
    status |= StartRanging();
    for (i = 0; i < (uint8_t)10; i++) {
        tmp = 0; j = 0; continue_loop = 1;
        do {
            status |= CheckForDataReady(&tmp);
            if (tmp == (uint8_t)1)             { continue_loop = 0; }
            else if (j < (uint16_t)5000)       { j++; }
            else { continue_loop = 0; status |= (uint8_t)VL53L4CD_ERROR_TIMEOUT; }
            WaitMs(1);
        } while (continue_loop == (uint8_t)1);
        status |= GetResult(&results);
        status |= ClearInterrupt();
    }
    status |= StopRanging();

    /* Device ranging */
    status |= StartRanging();
    for (i = 0; i < (uint8_t)nb_samples; i++) {
        tmp = 0; j = 0; continue_loop = 1;
        do {
            status |= CheckForDataReady(&tmp);
            if (tmp == (uint8_t)1)             { continue_loop = 0; }
            else if (j < (uint16_t)5000)       { j++; }
            else { continue_loop = 0; status |= (uint8_t)VL53L4CD_ERROR_TIMEOUT; }
            WaitMs(1);
        } while (continue_loop == (uint8_t)1);
        status |= GetResult(&results);
        status |= ClearInterrupt();
        AvgDistance += (int16_t)results.distance_mm;
    }
    status |= StopRanging();

    AvgDistance = AvgDistance / nb_samples;
    *p_measured_offset_mm = (int16_t)TargetDistInMm - AvgDistance;
    tmpOff = (uint16_t)*p_measured_offset_mm * (uint16_t)4;
    status |= WrWord(VL53L4CD_RANGE_OFFSET_MM, tmpOff);

    return status;
}

VL53L4CD_Error VL53L4CD::CalibrateXtalk(int16_t TargetDistInMm,
                                          uint16_t *p_measured_xtalk_kcps,
                                          int16_t nb_samples)
{
    VL53L4CD_Error status = VL53L4CD_ERROR_NONE;
    uint8_t i, tmp, continue_loop;
    float_t AverageSignal    = (float_t)0.0;
    float_t AvgDistance      = (float_t)0.0;
    float_t AverageSpadNb    = (float_t)0.0;
    float_t TargetDistance   = (float_t)TargetDistInMm;
    float_t tmp_xtalk, CounterNbSamples = (float_t)0.0;
    VL53L4CD_ResultsData_t results;
    uint16_t calXtalk, j;

    *p_measured_xtalk_kcps = 0;
    if (((nb_samples < (int16_t)5) || (nb_samples > (int16_t)255))
            || ((TargetDistInMm < (int16_t)10) || (TargetDistInMm > (int16_t)5000))) {
        return (uint8_t)VL53L4CD_ERROR_INVALID_ARGUMENT;
    }

    status |= WrWord(VL53L4CD_XTALK_PLANE_OFFSET_KCPS, *p_measured_xtalk_kcps);

    /* Device heat loop (10 samples) */
    status |= StartRanging();
    for (i = 0; i < (uint8_t)10; i++) {
        tmp = 0; j = 0; continue_loop = 1;
        do {
            status |= CheckForDataReady(&tmp);
            if (tmp == (uint8_t)1)             { continue_loop = 0; }
            else if (j < (uint16_t)5000)       { j++; }
            else { continue_loop = 0; status |= (uint8_t)VL53L4CD_ERROR_TIMEOUT; }
            WaitMs(1);
        } while (continue_loop == (uint8_t)1);
        status |= GetResult(&results);
        status |= ClearInterrupt();
    }
    status |= StopRanging();

    /* Device ranging loop */
    status |= StartRanging();
    for (i = 0; i < (uint8_t)nb_samples; i++) {
        tmp = 0; j = 0; continue_loop = 1;
        do {
            status |= CheckForDataReady(&tmp);
            if (tmp == (uint8_t)1)             { continue_loop = 0; }
            else if (j < (uint16_t)5000)       { j++; }
            else { continue_loop = 0; status |= (uint8_t)VL53L4CD_ERROR_TIMEOUT; }
            WaitMs(1);
        } while (continue_loop == (uint8_t)1);
        status |= GetResult(&results);
        status |= ClearInterrupt();

        if (results.range_status == (uint8_t)0 && i > (uint8_t)0) {
            AvgDistance      += (float_t)results.distance_mm;
            AverageSpadNb    += (float_t)results.number_of_spad;
            AverageSignal    += (float_t)results.signal_rate_kcps;
            CounterNbSamples++;
        }
    }
    status |= StopRanging();

    if (CounterNbSamples == 0) {
        return VL53L4CD_ERROR_XTALK_FAILED;
    }

    AvgDistance   /= CounterNbSamples;
    AverageSpadNb /= CounterNbSamples;
    AverageSignal /= CounterNbSamples;

    tmp_xtalk  = (float_t)1.0 - (AvgDistance / TargetDistance);
    tmp_xtalk *= (AverageSignal / AverageSpadNb);

    if (tmp_xtalk > (uint16_t)127) {
        status = VL53L4CD_ERROR_XTALK_FAILED;
    } else {
        *p_measured_xtalk_kcps = (uint16_t)(round(tmp_xtalk));
        calXtalk = (uint16_t)(tmp_xtalk * (float_t)512.0);
        status |= WrWord(VL53L4CD_XTALK_PLANE_OFFSET_KCPS, calXtalk);
    }

    return status;
}
