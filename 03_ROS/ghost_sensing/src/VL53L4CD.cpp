#include "VL53L4CD.h"
#include <string.h>
#include <cstdint>
#include <memory>
#include "I2C_interfacing.h"

namespace ghost_sensing {

    VL53L4CD::VL53L4CD(std::shared_ptr<I2C_interfacing> iface, uint8_t address)
    : m_i2c_communication(iface), m_i2c_address(address)
    {m_i2c_communication->init();
    }


    uint8_t VL53L4CD::init() {
         // Set I2C address

    	VL53L4CD_WrByte(VL53L4CD_I2C_SLAVE__DEVICE_ADDRESS,
			//(uint8_t)(m_i2c_address >> (uint8_t)1));
            (uint8_t)(m_i2c_address));
        
         //Initialize with defaults
         //... (additional initialization code as needed)

         uint8_t status = VL53L4CD_ERROR_NONE;
        uint8_t Addr, tmp;
        uint8_t continue_loop = 1;
        uint16_t i = 0;

        do{
            status |= VL53L4CD_RdByte(
                    VL53L4CD_FIRMWARE__SYSTEM_STATUS, &tmp);

            if(tmp == (uint8_t)0x3) /* Sensor booted */
            {
                continue_loop = (uint8_t)0;
            }
            else if(i < (uint16_t)1000)       /* Wait for boot */
            {
                i++;
            }
            else /* Timeout 1000ms reached */
            {
                continue_loop = (uint8_t)0;
                status |= (uint8_t)VL53L4CD_ERROR_TIMEOUT;
            }
            VL53L4CD_WaitMs(1);
        }while(continue_loop == (uint8_t)1);

        /* Load default configuration */
        for (Addr = (uint8_t)0x2D; Addr <= (uint8_t)0x87; Addr++)
        {
            status |= VL53L4CD_WrByte(Addr,
                    VL53L4CD_DEFAULT_CONFIGURATION[
                                    Addr - (uint8_t)0x2D]);
        }

        /* Start VHV */
        status |= VL53L4CD_WrByte(VL53L4CD_SYSTEM_START, (uint8_t)0x40);
        i  = (uint8_t)0;
        continue_loop = (uint8_t)1;
        do{
            status |= VL53L4CD_CheckForDataReady(&tmp);
            if(tmp == (uint8_t)1) /* Data ready */
            {
                continue_loop = (uint8_t)0;
            }
            else if(i < (uint16_t)1000)       /* Wait for answer */
            {
                i++;
            }
            else /* Timeout 1000ms reached */
            {
                continue_loop = (uint8_t)0;
                status |= (uint8_t)VL53L4CD_ERROR_TIMEOUT;
            }
            VL53L4CD_WaitMs(1);
        }while(continue_loop == (uint8_t)1);

        status |= VL53L4CD_ClearInterrupt();
        status |= VL53L4CD_StopRanging();
        status |= VL53L4CD_WrByte(
                VL53L4CD_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, 
                            (uint8_t)0x09);
        status |= VL53L4CD_WrByte(0x0B, (uint8_t)0);
        status |= VL53L4CD_WrWord(0x0024, 0x500);

        status |= VL53L4CD_SetRangeTiming(50, 0);

        return status;
    }

    bool VL53L4CD::writeRegister(uint8_t reg, uint8_t data) { return (m_i2c_communication->write(reg, &data, 1) == 0); }

    bool VL53L4CD::readRegister(uint8_t reg, uint8_t &data) { return (m_i2c_communication->read(reg, &data, 1) == 0);}

    bool VL53L4CD::readRegisters(uint8_t reg, uint8_t *buf, uint16_t len) {
        return (m_i2c_communication->read(reg, buf, len) == 0);}

    bool VL53L4CD::writeRegisters(uint8_t reg, uint8_t *buf, uint16_t len) {
        return (m_i2c_communication->write(reg, buf, len) == 0);}
    
    uint8_t VL53L4CD::VL53L4CD_WrByte( uint16_t RegisterAdress, uint8_t value){
        if(!writeRegister((uint8_t)RegisterAdress, value)) {}
        return 0;      
    }
    uint8_t VL53L4CD::VL53L4CD_WrWord( uint16_t RegisterAdress, uint16_t value){
        uint8_t buf[2];
    	buf[0] = (uint8_t)(value & 0xFF);  buf[1] = (uint8_t)((value >> 8) & 0xFF);
        if(!writeRegister((uint8_t)RegisterAdress, buf[0])||!writeRegister((uint8_t)(RegisterAdress + 1), buf[1])) {
        }
        return 0;
    }
    uint8_t VL53L4CD::VL53L4CD_WrDWord( uint16_t RegisterAdress, uint32_t value){
        uint8_t buf[4];
    	buf[0] = (uint8_t)(value & 0xFF);
    	buf[1] = (uint8_t)((value >> 8) & 0xFF);
    	buf[2] = (uint8_t)((value >> 16) & 0xFF);
    	buf[3] = (uint8_t)((value >> 24) & 0xFF);
    	if(!writeRegister((uint8_t)RegisterAdress, buf[0])) {}
    	if(!writeRegister((uint8_t)(RegisterAdress + 1), buf[1])) {}
    	if(!writeRegister((uint8_t)(RegisterAdress + 2), buf[2])) {}
    	if(!writeRegister((uint8_t)(RegisterAdress + 3), buf[3])) {}
        return 0;
    }

    uint8_t VL53L4CD::VL53L4CD_WaitMs( uint32_t TimeMs){
        m_i2c_communication->delay_ms(TimeMs);
        return 0;
    }

    uint8_t VL53L4CD::VL53L4CD_RdDWord( uint16_t RegisterAdress, uint32_t *value){
    	uint8_t buf[4];  
         if (!readRegisters((uint8_t)RegisterAdress, buf, sizeof(buf))) {
             
         }
    	 uint32_t temp = ((uint32_t)buf[3] << 24) | ((uint32_t)buf[2] << 16) | ((uint32_t)buf[1] << 8) | ((uint32_t)buf[0]);
    	 *value = temp;
    	return 0;
    }

    uint8_t VL53L4CD::VL53L4CD_RdWord( uint16_t RegisterAdress, uint16_t *value){
    	uint8_t buf[2];  
         if (!readRegisters((uint8_t)RegisterAdress, buf, sizeof(buf))) {
             
         }
    	 uint16_t temp = ((uint16_t)buf[1] << 8) | ((uint16_t)buf[0]);
    	 *value = temp;
    	return 0;
    }

    uint8_t VL53L4CD::VL53L4CD_RdByte( uint16_t RegisterAdress, uint8_t *value){
    	uint8_t buf[1];  
         if (!readRegisters((uint8_t)RegisterAdress, buf, sizeof(buf))) {
             
         }
    	 uint8_t temp = buf[0];
    	 *value = temp;
    	return 0;
    }
    
    //actual stuff
    //2 invalid input
    uint8_t VL53L4CD::VL53L4CD_CalibrateOffset( int16_t TargetDistInMm, int16_t *p_measured_offset_mm,
            int16_t nb_samples)
      {
      uint8_t status = VL53L4CD_ERROR_NONE;
        uint8_t i, tmp, continue_loop;
        uint16_t j, tmpOff;
        int16_t AvgDistance = 0;
        VL53L4CD_ResultsData_t results;

        if(((nb_samples < (int16_t)5) || (nb_samples > (int16_t)255))
                || ((TargetDistInMm < (int16_t)10)
                    || (TargetDistInMm > (int16_t)1000)))
        {
            return 2;
        }
        else
        {
            status |= VL53L4CD_WrWord(VL53L4CD_T_MMRANGE_OFFSET, 0x0);
            status |= VL53L4CD_WrWord(VL53L4CD_INNER_OFFSET_MM, 0x0);
            status |= VL53L4CD_WrWord(VL53L4CD_OUTER_OFFSET_MM, 0x0);
            /* Device heat loop (10 samples) */
            status |= VL53L4CD_StartRanging();
            for (i = 0; i < (uint8_t)10; i++) {
                tmp = (uint8_t)0;
                j = (uint16_t)0;
                continue_loop = (uint8_t)1;
                do{
                    status |= VL53L4CD_CheckForDataReady(&tmp);
                    if(tmp == (uint8_t)1) /* Data ready */
                    {
                        continue_loop = (uint8_t)0;
                    }
                    else if(j < (uint16_t)5000) /* Wait for answer*/
                    {
                        j++;
                    }
                    else /* Timeout 5000ms reached */
                    {
                        continue_loop = (uint8_t)0;
                        status |= (uint8_t)VL53L4CD_ERROR_TIMEOUT;
                    }
                    VL53L4CD_WaitMs(1);
                }while(continue_loop == (uint8_t)1); //checks for data ready
                status |= VL53L4CD_GetResult(&results);
                status |= VL53L4CD_ClearInterrupt();
            }
            status |= VL53L4CD_StopRanging();

            /* Device ranging */
            status |= VL53L4CD_StartRanging();
            for (i = 0; i < (uint8_t)nb_samples; i++) {
                tmp = (uint8_t)0;
                j = (uint16_t)0;
                continue_loop = (uint8_t)1;
                do{
                    status |= VL53L4CD_CheckForDataReady(&tmp);
                    if(tmp == (uint8_t)1) /* Data ready */
                    {
                        continue_loop = (uint8_t)0;
                    }
                    else if(j < (uint16_t)5000) /* Wait for answer*/
                    {
                        j++;
                    }
                    else /* Timeout 5000ms reached */
                    {
                        continue_loop = (uint8_t)0;
                        status |= (uint8_t)VL53L4CD_ERROR_TIMEOUT;
                    }
                    VL53L4CD_WaitMs(1);
                }while(continue_loop == (uint8_t)1);

                status |= VL53L4CD_GetResult(&results);
                status |= VL53L4CD_ClearInterrupt();
                AvgDistance += (int16_t)results.distance_mm;
            }

            status |= VL53L4CD_StopRanging();
            AvgDistance = AvgDistance / nb_samples;
            *p_measured_offset_mm = (int16_t)TargetDistInMm - AvgDistance;
            tmpOff = (uint16_t) *p_measured_offset_mm * (uint16_t)4;
            status |= VL53L4CD_WrWord(VL53L4CD_T_MMRANGE_OFFSET, tmpOff); //write offset
        }

        return status;
    }


    uint8_t VL53L4CD::VL53L4CD_CalibrateXtalk(
            
            int16_t TargetDistInMm,
            uint16_t *p_measured_xtalk_kcps,
            int16_t nb_samples)
    {
        uint8_t status = VL53L4CD_ERROR_NONE;
        uint8_t i, tmp, continue_loop;
        float_t AverageSignal = (float_t)0.0;
        float_t AvgDistance = (float_t)0.0;
        float_t AverageSpadNb = (float_t)0.0;
        float_t TargetDistance = (float_t)TargetDistInMm;
        float_t tmp_xtalk, CounterNbSamples = (float_t)0.0;
        VL53L4CD_ResultsData_t results;

        uint16_t calXtalk, j;

        *p_measured_xtalk_kcps = 0;
        if(((nb_samples < (int16_t)5) || (nb_samples > (int16_t)255))
                || ((TargetDistInMm < (int16_t)10)
                    || (TargetDistInMm > (int16_t)5000)))
        {
            return 2;
        }
        else
        {
            /* Disable Xtalk compensation */
            status |= VL53L4CD_WrWord(
                VL53L4CD_XTALK_PLANE_OFFSET_KCPS, *p_measured_xtalk_kcps);

            /* Device heat loop (10 samples) */
            status |= VL53L4CD_StartRanging();
            for (i = 0; i < (uint8_t)10; i++) {
                tmp = (uint8_t)0;
                j = (uint16_t)0;
                continue_loop = (uint8_t)1;
                do{
                    status |= VL53L4CD_CheckForDataReady(&tmp);
                    if(tmp == (uint8_t)1) /* Data ready */
                    {
                        continue_loop = (uint8_t)0;
                    }
                    else if(j < (uint16_t)5000) /* Wait for answer*/
                    {
                        j++;
                    }
                    else /* Timeout 5000ms reached */
                    {
                        continue_loop = (uint8_t)0;
                        status |= (uint8_t)VL53L4CD_ERROR_TIMEOUT;
                    }
                    VL53L4CD_WaitMs(1);
                }while(continue_loop == (uint8_t)1);
                status |= VL53L4CD_GetResult(&results);
                status |= VL53L4CD_ClearInterrupt();
            }
            status |= VL53L4CD_StopRanging();

            /* Device ranging loop */
            status |= VL53L4CD_StartRanging();
            for (i = 0; i < (uint8_t)nb_samples; i++)
                {
                tmp = (uint8_t)0;
                j = (uint16_t)0;
                continue_loop = (uint8_t)1;
                do{
                    status |= VL53L4CD_CheckForDataReady(&tmp);
                    if(tmp == (uint8_t)1) /* Data ready */
                    {
                        continue_loop = (uint8_t)0;
                    }
                    else if(j < (uint16_t)5000) /* Wait for answer*/
                    {
                        j++;
                    }
                    else /* Timeout 5000ms reached */
                    {
                        continue_loop = (uint8_t)0;
                        status |= (uint8_t)VL53L4CD_ERROR_TIMEOUT;
                    }
                    VL53L4CD_WaitMs(1);
                }while(continue_loop == (uint8_t)1);

                status |= VL53L4CD_GetResult(&results);
                status |= VL53L4CD_ClearInterrupt();

                /* Discard invalid measurements and first frame */
                if (results.range_status == (uint8_t)0
                        && i > (uint8_t)0)
                {
                    AvgDistance += (float_t)results.distance_mm;
                    AverageSpadNb += (float_t)results.number_of_spad;
                    AverageSignal += (float_t)results.signal_rate_kcps;
                    CounterNbSamples++;
                }
            }
            status |= VL53L4CD_StopRanging();

            if (CounterNbSamples == 0)
            {
                status = VL53L4CD_ERROR_XTALK_FAILED;
            }
            else
            {
                AvgDistance /= CounterNbSamples;
                AverageSpadNb /= CounterNbSamples;
                AverageSignal /= CounterNbSamples;

                tmp_xtalk = (float_t)1.0 - (AvgDistance/TargetDistance);
                tmp_xtalk *= (AverageSignal/AverageSpadNb);

                /* 127kcps is the max Xtalk value (65536/512) */
                if(tmp_xtalk > (uint16_t)127)
                {
                    status = VL53L4CD_ERROR_XTALK_FAILED;
                }
                else
                {
                    *p_measured_xtalk_kcps = (uint16_t)(round(tmp_xtalk));

                    /* Send data to firmware */
                    calXtalk = (uint16_t)(tmp_xtalk * (float_t)512.0);
                    status |= VL53L4CD_WrWord(
                        VL53L4CD_XTALK_PLANE_OFFSET_KCPS, calXtalk); // sets xtalk, idk why its equal what it is, but surely its right
                }
            }
        }

        return status;
    }


    uint8_t VL53L4CD::VL53L4CD_GetSWVersion(
                VL53L4CD_Version_t *p_Version)
        {
            uint8_t Status = VL53L4CD_ERROR_NONE;

            p_Version->major = VL53L4CD_IMPLEMENTATION_VER_MAJOR;
            p_Version->minor = VL53L4CD_IMPLEMENTATION_VER_MINOR;
            p_Version->build = VL53L4CD_IMPLEMENTATION_VER_BUILD;
            p_Version->revision = VL53L4CD_IMPLEMENTATION_VER_REVISION;
            return Status;
        }

            uint8_t VL53L4CD::VL53L4CD_GetSensorId(
                
                uint16_t *p_id)
        {
            uint8_t status = VL53L4CD_ERROR_NONE;
            status |= VL53L4CD_RdWord(VL53L4CD_IDENTIFICATION__MODEL_ID, p_id);
            return status;
        }
    uint8_t VL53L4CD::VL53L4CD_ClearInterrupt(
           )
    {
        uint8_t status = VL53L4CD_ERROR_NONE;

        status |= VL53L4CD_WrByte(VL53L4CD_SYSTEM__INTERRUPT_CLEAR, 0x01);
        return status;
    }

        uint8_t VL53L4CD::VL53L4CD_StartRanging(
           )
    {
        uint8_t status = VL53L4CD_ERROR_NONE;
        uint32_t tmp;

        status |= VL53L4CD_RdDWord(VL53L4CD_INTERMEASUREMENT_MS, &tmp);

        /* Sensor runs in continuous mode */
        if(tmp == (uint32_t)0)
        {
            status |= VL53L4CD_WrByte(VL53L4CD_SYSTEM_START, 0x21);
        }
        /* Sensor runs in autonomous mode */
        else
        {
            status |= VL53L4CD_WrByte(VL53L4CD_SYSTEM_START, 0x40);
        }

        return status;
    }

        uint8_t VL53L4CD::VL53L4CD_StopRanging(
           )
    {
        uint8_t status = VL53L4CD_ERROR_NONE;

        status |= VL53L4CD_WrByte(VL53L4CD_SYSTEM_START, 0x80);
        return status;
    }

        uint8_t VL53L4CD::VL53L4CD_CheckForDataReady(
            
            uint8_t *p_is_data_ready)
    {
        uint8_t status = VL53L4CD_ERROR_NONE;
        uint8_t temp;
        uint8_t int_pol;

        status |= VL53L4CD_RdByte(VL53L4CD_GPIO_HV_MUX__CTRL, &temp);
        temp = temp & (uint8_t)0x10;
        temp = temp >> 4;

        if (temp == (uint8_t)1)
        {
            int_pol = (uint8_t)0;
        }
        else
        {
            int_pol = (uint8_t)1;
        }

        status |= VL53L4CD_RdByte(VL53L4CD_GPIO__TIO_HV_STATUS, &temp);

        if ((temp & (uint8_t)1) == int_pol)
        {
            *p_is_data_ready = (uint8_t)1;
        }
        else
        {
            *p_is_data_ready = (uint8_t)0;
        }

        return status;
    }

        uint8_t VL53L4CD::VL53L4CD_SetRangeTiming(
            
            uint32_t timing_budget_ms,
            uint32_t inter_measurement_ms)
    {
        uint8_t status = VL53L4CD_ERROR_NONE;
        uint16_t clock_pll, osc_frequency, ms_byte;
        uint32_t macro_period_us = 0, timing_budget_us = 0, ls_byte, tmp;
        float_t inter_measurement_factor = (float_t)1.055;

        status |= VL53L4CD_RdWord(0x0006, &osc_frequency);
        if(osc_frequency != (uint16_t)0)
        {
            timing_budget_us = timing_budget_ms*(uint32_t)1000;
            macro_period_us = (uint32_t)((uint32_t)2304 *
            ((uint32_t)0x40000000 / (uint32_t)osc_frequency)) >> 6;
        }
        else
        {
            status |= (uint8_t)VL53L4CD_ERROR_INVALID_ARGUMENT;
        }

        /* Timing budget check validity */
        if ((timing_budget_ms < (uint32_t)10) 
                || (timing_budget_ms > (uint32_t)200) || (status != (uint8_t)0))
        {
            status |= VL53L4CD_ERROR_INVALID_ARGUMENT;
        }
        /* Sensor runs in continuous mode */
        else if(inter_measurement_ms == (uint32_t)0)
        {
            status |= VL53L4CD_WrDWord(VL53L4CD_INTERMEASUREMENT_MS, 0);
            timing_budget_us -= (uint32_t)2500;
        }
        /* Sensor runs in autonomous low power mode */
        else if(inter_measurement_ms > timing_budget_ms)
        {
            status |= VL53L4CD_RdWord(
                    VL53L4CD_RESULT__OSC_CALIBRATE_VAL, &clock_pll);
            clock_pll = clock_pll & (uint16_t)0x3FF;
                    inter_measurement_factor = inter_measurement_factor
                    * (float_t)inter_measurement_ms
                    * (float_t)clock_pll;
            status |= VL53L4CD_WrDWord(VL53L4CD_INTERMEASUREMENT_MS,
                    (uint32_t)inter_measurement_factor);

            timing_budget_us -= (uint32_t)4300;
            timing_budget_us /= (uint32_t)2;

        }
        /* Invalid case */
        else
        {
            status |= (uint8_t)VL53L4CD_ERROR_INVALID_ARGUMENT;
        }

        if(status != (uint8_t)VL53L4CD_ERROR_INVALID_ARGUMENT)
        {
                    ms_byte = 0;
                    timing_budget_us = timing_budget_us << 12;
                    tmp = macro_period_us*(uint32_t)16;
                    ls_byte = ((timing_budget_us + ((tmp >> 6)>>1)) /(tmp>> 6))
                    - (uint32_t)1;

                    while ((ls_byte & 0xFFFFFF00U) > 0U) {
                            ls_byte = ls_byte >> 1;
                            ms_byte++;
                    }
                    ms_byte = (uint16_t)(ms_byte << 8)
                + (uint16_t) (ls_byte & (uint32_t)0xFF);
                    status |= VL53L4CD_WrWord(VL53L4CD_RANGE_CONFIG_A,ms_byte);

                    ms_byte = 0;
                    tmp = macro_period_us*(uint32_t)12;
                    ls_byte = ((timing_budget_us + ((tmp >> 6)>>1)) /(tmp>> 6))
                    - (uint32_t)1;

                    while ((ls_byte & 0xFFFFFF00U) > 0U) {
                            ls_byte = ls_byte >> 1;
                            ms_byte++;
                    }
                    ms_byte = (uint16_t)(ms_byte << 8)
                + (uint16_t) (ls_byte & (uint32_t)0xFF);
                    status |= VL53L4CD_WrWord(VL53L4CD_RANGE_CONFIG_B,ms_byte);
        }

        return status;
    }

        uint8_t VL53L4CD::VL53L4CD_GetRangeTiming(
            
            uint32_t *p_timing_budget_ms,
            uint32_t *p_inter_measurement_ms)
    {
        uint8_t status = VL53L4CD_ERROR_NONE;
        uint16_t osc_frequency = 1, range_config_macrop_high, clock_pll = 1;
        uint32_t tmp, ls_byte, ms_byte, macro_period_us;
        float_t clock_pll_factor = (float_t)1.065;

        /* Get InterMeasurement */
        status |= VL53L4CD_RdDWord(VL53L4CD_INTERMEASUREMENT_MS, &tmp);
        status |= VL53L4CD_RdWord(
                VL53L4CD_RESULT__OSC_CALIBRATE_VAL, &clock_pll);
        clock_pll = clock_pll & (uint16_t)0x3FF;
        clock_pll_factor = clock_pll_factor * (float_t)clock_pll;
        clock_pll = (uint16_t)clock_pll_factor;
        *p_inter_measurement_ms = (uint16_t)(tmp/(uint32_t)clock_pll);

        /* Get TimingBudget */
        status |= VL53L4CD_RdWord(0x0006, &osc_frequency);
        status |= VL53L4CD_RdWord(VL53L4CD_RANGE_CONFIG_A,
            &range_config_macrop_high);

        macro_period_us = (uint32_t)((uint32_t)2304 * ((uint32_t)0x40000000
                / (uint32_t)osc_frequency)) >> 6;
        ls_byte = (range_config_macrop_high & (uint32_t)0x00FF) << 4;
        ms_byte = (range_config_macrop_high & (uint32_t)0xFF00) >> 8;
        ms_byte = (uint32_t)0x04 - (ms_byte - (uint32_t)1) - (uint32_t)1;

        macro_period_us = macro_period_us * (uint32_t)16;
        *p_timing_budget_ms = (((ls_byte + (uint32_t)1)*(macro_period_us>> 6))
            - ((macro_period_us>> 6)>>1)) >> 12;

        if(ms_byte < (uint8_t)12)
        {
            *p_timing_budget_ms = (uint32_t)(*p_timing_budget_ms
                    >> (uint8_t)ms_byte);
        }
        
        /* Mode continuous */
        if(tmp == (uint32_t)0)
        {
            *p_timing_budget_ms += (uint32_t)2500;
        }
        /* Mode autonomous */
        else
        {
            *p_timing_budget_ms *= (uint32_t)2;
            *p_timing_budget_ms += (uint32_t)4300;
        }

        *p_timing_budget_ms = *p_timing_budget_ms/(uint32_t)1000;

        return status;
    }

        uint8_t VL53L4CD::VL53L4CD_GetResult(
            
            VL53L4CD_ResultsData_t *p_result)
    {
        uint8_t status = VL53L4CD_ERROR_NONE;
        uint16_t temp_16;
        uint8_t temp_8;
        uint16_t raw_spads;
        
        uint8_t status_rtn[24] = { 255, 255, 255, 5, 2, 4, 1, 7, 3,
                0, 255, 255, 9, 13, 255, 255, 255, 255, 10, 6,
                255, 255, 11, 12 };

        status |= VL53L4CD_RdByte(VL53L4CD_RESULT__RANGE_STATUS,
            &temp_8);
        temp_8 = temp_8 & (uint8_t)0x1F;
        if (temp_8 < (uint8_t)24)
        {
            temp_8 = status_rtn[temp_8];
        }
        p_result->range_status = temp_8;

        status |= VL53L4CD_RdWord(VL53L4CD_RESULT__SPAD_NB,
            &temp_16);
        raw_spads=temp_16;
        p_result->number_of_spad = temp_16 / (uint16_t) 256;

        status |= VL53L4CD_RdWord(VL53L4CD_RESULT__SIGNAL_RATE,
            &temp_16);
        p_result->signal_rate_kcps = (uint32_t)temp_16 *  8;

        status |= VL53L4CD_RdWord(VL53L4CD_RESULT__AMBIENT_RATE,
            &temp_16);
        p_result->ambient_rate_kcps = (uint32_t)temp_16 *  8;

        status |= VL53L4CD_RdWord(VL53L4CD_RESULT__SIGMA,
            &temp_16);
        p_result->sigma_mm = temp_16 / (uint16_t) 4;

        status |= VL53L4CD_RdWord(VL53L4CD_RESULT__DISTANCE,
            &temp_16);
        p_result->distance_mm = temp_16;

        p_result->signal_per_spad_kcps = p_result->signal_rate_kcps *256
                /(uint32_t) raw_spads;
        p_result->ambient_per_spad_kcps = p_result->ambient_rate_kcps *256
                /(uint32_t) raw_spads;																							  

        return status;
    }

        uint8_t VL53L4CD::VL53L4CD_SetOffset(
            
            int16_t OffsetValueInMm)
    {
        uint8_t status = VL53L4CD_ERROR_NONE;
        uint16_t temp;

        temp = (uint16_t)((uint16_t)OffsetValueInMm*(uint16_t)4);

        status |= VL53L4CD_WrWord(VL53L4CD_T_MMRANGE_OFFSET, temp);
        status |= VL53L4CD_WrWord(VL53L4CD_INNER_OFFSET_MM, (uint8_t)0x0);
        status |= VL53L4CD_WrWord(VL53L4CD_OUTER_OFFSET_MM, (uint8_t)0x0);
        return status;
    }

        uint8_t VL53L4CD::VL53L4CD_GetOffset(
            
            int16_t *p_offset)
    {
        uint8_t status = VL53L4CD_ERROR_NONE;
        uint16_t temp;

        status |= VL53L4CD_RdWord(VL53L4CD_T_MMRANGE_OFFSET, &temp);

        temp = temp<<3;
        temp = temp>>5;
        *p_offset = (int16_t)(temp);

        if(*p_offset > 1024)
        {
            *p_offset = *p_offset - 2048;
        }

        return status;
    }

        uint8_t VL53L4CD::VL53L4CD_SetXtalk(
            
            uint16_t XtalkValueKcps)
    {
        uint8_t status = VL53L4CD_ERROR_NONE;

        status |= VL53L4CD_WrWord(
            VL53L4CD_XTALK_X_PLANE_GRADIENT_KCPS, 0x0000);
        status |= VL53L4CD_WrWord(
            VL53L4CD_XTALK_Y_PLANE_GRADIENT_KCPS, 0x0000);
        status |= VL53L4CD_WrWord(
            VL53L4CD_XTALK_PLANE_OFFSET_KCPS,
            (XtalkValueKcps<<9));
            
        return status;
    }

        uint8_t VL53L4CD::VL53L4CD_GetXtalk(
            
            uint16_t *p_xtalk_kcps)
    {
        uint8_t status = VL53L4CD_ERROR_NONE;
        float_t tmp_xtalk;

        status |= VL53L4CD_RdWord(
            VL53L4CD_XTALK_PLANE_OFFSET_KCPS, p_xtalk_kcps);
            
        tmp_xtalk = (float_t)*p_xtalk_kcps / (float_t)512.0;
        *p_xtalk_kcps = (uint16_t)(round(tmp_xtalk));

        return status;
    }

        uint8_t VL53L4CD::VL53L4CD_SetDetectionThresholds(
            
            uint16_t distance_low_mm,
            uint16_t distance_high_mm,
            uint8_t window)
    {
        uint8_t status = VL53L4CD_ERROR_NONE;

        status |= VL53L4CD_WrByte(VL53L4CD_SYSTEM__INTERRUPT, window);
        status |= VL53L4CD_WrWord(VL53L4CD_THRESH_HIGH, distance_high_mm);
        status |= VL53L4CD_WrWord(VL53L4CD_THRESH_LOW, distance_low_mm);
        return status;
    }

    uint8_t VL53L4CD::VL53L4CD_GetDetectionThresholds(
            uint16_t *p_distance_low_mm,
            uint16_t *p_distance_high_mm,
            uint8_t *p_window)
    {
        uint8_t status = VL53L4CD_ERROR_NONE;

        status |= VL53L4CD_RdWord(VL53L4CD_THRESH_HIGH,p_distance_high_mm);
        status |= VL53L4CD_RdWord(VL53L4CD_THRESH_LOW, p_distance_low_mm);
        status |= VL53L4CD_RdByte(VL53L4CD_SYSTEM__INTERRUPT, p_window);
        *p_window = (*p_window & (uint8_t)0x7);

        return status;
    }

        uint8_t VL53L4CD::VL53L4CD_SetSignalThreshold(
            
            uint16_t signal_kcps)
    {
        uint8_t status = VL53L4CD_ERROR_NONE;
        status |= VL53L4CD_WrWord(
                VL53L4CD_MIN_COUNT_RATE_RTN_LIMIT_MCPS,signal_kcps>>3);
        return status;
    }

    uint8_t VL53L4CD::VL53L4CD_GetSignalThreshold(
            
            uint16_t 	*p_signal_kcps)
    {
        uint8_t status = VL53L4CD_ERROR_NONE;
        uint16_t tmp = 0;

        status |= VL53L4CD_RdWord(
                VL53L4CD_MIN_COUNT_RATE_RTN_LIMIT_MCPS, &tmp);
        *p_signal_kcps = tmp <<3;

        return status;
    }

        uint8_t VL53L4CD::VL53L4CD_SetSigmaThreshold(
            
            uint16_t 	sigma_mm)
    {
        uint8_t status = VL53L4CD_ERROR_NONE;
        if(sigma_mm>(uint16_t)((uint16_t)0xFFFF>>2))
        {
            status |= (uint8_t)VL53L4CD_ERROR_INVALID_ARGUMENT;
        }
        else
        {
            status |= VL53L4CD_WrWord(
                VL53L4CD_RANGE_CONFIG__SIGMA_THRESH, sigma_mm<<2);
        }

        return status;
    }

        uint8_t VL53L4CD::VL53L4CD_GetSigmaThreshold(
            
            uint16_t 	*p_sigma_mm)
    {
        uint8_t status = VL53L4CD_ERROR_NONE;

        status |= VL53L4CD_RdWord(
                VL53L4CD_RANGE_CONFIG__SIGMA_THRESH, p_sigma_mm);
        *p_sigma_mm = *p_sigma_mm >> 2;

        return status;
    }

        uint8_t VL53L4CD::VL53L4CD_StartTemperatureUpdate(
           )
    {
        uint8_t status = VL53L4CD_ERROR_NONE;
        uint8_t tmp = 0, continue_loop = 1;
        uint16_t i = 0;

        status |= VL53L4CD_WrByte(
            VL53L4CD_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, (uint8_t)0x81);
        status |= VL53L4CD_WrByte(0x0B, (uint8_t)0x92);
        status |= VL53L4CD_StartRanging();

        do{
                status |= VL53L4CD_CheckForDataReady(&tmp);
                if(tmp == (uint8_t)1) /* Data ready */
                {
                        continue_loop = (uint8_t)0;
                }
                else if(i < (uint16_t)1000)       /* Wait for answer */
                {
                        i++;
                }
                else /* Timeout 1000ms reached */
                {
                        continue_loop = (uint8_t)0;
                        status = (uint8_t)VL53L4CD_ERROR_TIMEOUT;
                }
                VL53L4CD_WaitMs(1);
        }while(continue_loop == (uint8_t)1);

        status |= VL53L4CD_ClearInterrupt();
        status |= VL53L4CD_StopRanging();

        status += VL53L4CD_WrByte(
            VL53L4CD_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, 0x09);
        status += VL53L4CD_WrByte(0x0B, 0);
        return status;
    }
}