#include <stdint.h>
#include <memory>
#include "I2C_interfacing.h"

namespace ghost_sensing {
class VL53L4CD {
    private:
    std::shared_ptr<I2C_interfacing> m_i2c_communication;
    uint8_t m_i2c_address;

    bool writeRegister(uint8_t reg, uint8_t data);
    bool writeRegisters(uint8_t reg, uint8_t *buf, uint16_t len);
    bool readRegister(uint8_t reg, uint8_t &data);   
    bool readRegisters(uint8_t reg, uint8_t *buf, uint16_t len);

    public:
    VL53L4CD(std::shared_ptr<I2C_interfacing> iface, uint8_t address);
    ~VL53L4CD() = default;

    uint8_t init();
    bool deinit();
    uint8_t VL53L4CD_RdDWord( uint16_t registerAddr, uint32_t *value);
    uint8_t VL53L4CD_RdWord( uint16_t registerAddr, uint16_t *value);
    uint8_t VL53L4CD_RdByte( uint16_t registerAddr, uint8_t *value);
    uint8_t VL53L4CD_WrByte( uint16_t registerAddr, uint8_t value);
    uint8_t VL53L4CD_WrWord( uint16_t RegisterAdress, uint16_t value);
    uint8_t VL53L4CD_WrDWord( uint16_t RegisterAdress, uint32_t value);
    uint8_t VL53L4CD_WaitMs( uint32_t TimeMs);

    // dev doesn't matter unless theres multiple of the same sensor
    //nb samples correlates with accuracy i.e. more samples = lower std dev
    //auto sets offsets in the sensor
   uint8_t VL53L4CD_CalibrateOffset( int16_t TargetDistInMm, int16_t *p_measured_offset_mm,
            int16_t nb_samples);
    
    uint8_t VL53L4CD_CalibrateXtalk(
            
            int16_t TargetDistInMm,
            uint16_t *p_measured_xtalk_kcps,
            int16_t nb_samples);

    



    	typedef struct {
		/* Status of measurements. If the status is equal to 0, the data are valid*/
		uint8_t range_status;
		/* Measured distance in millimeters */
		uint16_t distance_mm;
		/* Ambient noise in kcps */
		uint32_t ambient_rate_kcps;
		/* Ambient noise in kcps/SPAD */
		uint32_t ambient_per_spad_kcps;
		/* Measured signal of the target in kcps */
		uint32_t signal_rate_kcps;
		/* Measured signal of the target in kcps/SPAD */
		uint32_t signal_per_spad_kcps;
		/* Number of SPADs enabled */
		uint16_t number_of_spad;
		/* Estimated measurements std deviation in mm */
		uint16_t sigma_mm;
	} VL53L4CD_ResultsData_t;


    #define VL53L4CD_IMPLEMENTATION_VER_MAJOR       2
	#define VL53L4CD_IMPLEMENTATION_VER_MINOR       2
	#define VL53L4CD_IMPLEMENTATION_VER_BUILD       3
	#define VL53L4CD_IMPLEMENTATION_VER_REVISION  	0

    	typedef struct {
		uint8_t      major;    /*!< major number */
		uint8_t      minor;    /*!< minor number */
		uint8_t      build;    /*!< build number */
		uint32_t     revision; /*!< revision number */
	} VL53L4CD_Version_t;

    uint8_t VL53L4CD_GetSWVersion(
            VL53L4CD_Version_t *p_Version);
    
    uint8_t VL53L4CD_GetSensorId(
            
            uint16_t *p_id);

    uint8_t VL53L4CD_ClearInterrupt(
           );

    uint8_t VL53L4CD_StartRanging(
           ); 

    uint8_t VL53L4CD_StopRanging(
           );

    uint8_t VL53L4CD_CheckForDataReady(
            
            uint8_t *p_isReady);

    uint8_t VL53L4CD_SetRangeTiming(
            
            uint32_t timing_budget_ms,
            uint32_t inter_measurement_ms);    
    
    uint8_t VL53L4CD_GetRangeTiming(
              
            uint32_t *p_timing_budget_ms,
            uint32_t *p_inter_measurement_ms);

    uint8_t VL53L4CD_GetResult(
            
            VL53L4CD_ResultsData_t *p_result);

    uint8_t VL53L4CD_SetOffset(
            
            int16_t OffsetValueInMm);
            
    uint8_t VL53L4CD_GetOffset(
            
            int16_t *pOffsetValueInMm);
    
    uint8_t VL53L4CD_SetXtalk(
            
            uint16_t XtalkValueInKcps);
    
    uint8_t VL53L4CD_GetXtalk(
            
            uint16_t *pXtalkValueInKcps);
    
    uint8_t VL53L4CD_SetDetectionThresholds(
            
            uint16_t distance_low_mm,
            uint16_t distance_high_mm,
            uint8_t window);
    
    uint8_t VL53L4CD_GetDetectionThresholds(
            
            uint16_t *p_distance_low_mm,
            uint16_t *p_distance_high_mm,
            uint8_t *p_window);
    
    uint8_t VL53L4CD_SetInterruptThresholds(
            
            uint16_t distance_low_mm,
            uint16_t distance_high_mm,
            uint8_t window);
    
    uint8_t VL53L4CD_GetInterruptThresholds(
            
            uint16_t *p_distance_low_mm,
            uint16_t *p_distance_high_mm,
            uint8_t *p_window);

    uint8_t VL53L4CD_SetSignalThreshold(
            
            uint16_t signal_kcps);

    uint8_t VL53L4CD_GetSignalThreshold(
            
            uint16_t 	*p_signal_kcps);

    uint8_t VL53L4CD_SetSigmaThreshold(
            
            uint16_t 	sigma_mm);
    
    uint8_t VL53L4CD_GetSigmaThreshold(
            
            uint16_t 	*p_sigma_mm);
    
    uint8_t VL53L4CD_StartTemperatureUpdate(
           );

    

	/**
	 *  @brief Driver error type
	 */

	#define VL53L4CD_ERROR_NONE					((uint8_t)0U)
	#define VL53L4CD_ERROR_XTALK_FAILED			((uint8_t)253U)
	#define VL53L4CD_ERROR_INVALID_ARGUMENT		((uint8_t)254U)
	#define VL53L4CD_ERROR_TIMEOUT				((uint8_t)255U)


	/**
	 *  @brief Inner Macro for API. Not for user, only for development.
	 */

	#define VL53L4CD_SOFT_RESET     ((uint16_t)0x0000)
	#define VL53L4CD_I2C_SLAVE__DEVICE_ADDRESS      ((uint16_t)0x0001)
	#define VL53L4CD_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND  ((uint16_t)0x0008)
	#define VL53L4CD_XTALK_PLANE_OFFSET_KCPS ((uint16_t)0x0016)
	#define VL53L4CD_XTALK_X_PLANE_GRADIENT_KCPS     ((uint16_t)0x0018)
	#define VL53L4CD_XTALK_Y_PLANE_GRADIENT_KCPS     ((uint16_t)0x001A)
	#define VL53L4CD_T_MMRANGE_OFFSET     ((uint16_t)0x001E)
	#define VL53L4CD_INNER_OFFSET_MM     ((uint16_t)0x0020)
	#define VL53L4CD_OUTER_OFFSET_MM     ((uint16_t)0x0022)
	#define VL53L4CD_GPIO_HV_MUX__CTRL      ((uint16_t)0x0030)
	#define VL53L4CD_GPIO__TIO_HV_STATUS    ((uint16_t)0x0031)
	#define VL53L4CD_SYSTEM__INTERRUPT  ((uint16_t)0x0046)
	#define VL53L4CD_RANGE_CONFIG_A     ((uint16_t)0x005E)
	#define VL53L4CD_RANGE_CONFIG_B      ((uint16_t)0x0061)
	#define VL53L4CD_RANGE_CONFIG__SIGMA_THRESH     ((uint16_t)0x0064)
	#define VL53L4CD_MIN_COUNT_RATE_RTN_LIMIT_MCPS    ((uint16_t)0x0066)
	#define VL53L4CD_INTERMEASUREMENT_MS ((uint16_t)0x006C)
	#define VL53L4CD_THRESH_HIGH    ((uint16_t)0x0072)
	#define VL53L4CD_THRESH_LOW     ((uint16_t)0x0074)
	#define VL53L4CD_SYSTEM__INTERRUPT_CLEAR        ((uint16_t)0x0086)
	#define VL53L4CD_SYSTEM_START     ((uint16_t)0x0087)
	#define VL53L4CD_RESULT__RANGE_STATUS   ((uint16_t)0x0089)
	#define VL53L4CD_RESULT__SPAD_NB   ((uint16_t)0x008C)
	#define VL53L4CD_RESULT__SIGNAL_RATE   ((uint16_t)0x008E)
	#define VL53L4CD_RESULT__AMBIENT_RATE   ((uint16_t)0x0090)
	#define VL53L4CD_RESULT__SIGMA   ((uint16_t)0x0092)
	#define VL53L4CD_RESULT__DISTANCE   ((uint16_t)0x0096)


	#define VL53L4CD_RESULT__OSC_CALIBRATE_VAL      ((uint16_t)0x00DE)
	#define VL53L4CD_FIRMWARE__SYSTEM_STATUS        ((uint16_t)0x00E5)
	#define VL53L4CD_IDENTIFICATION__MODEL_ID       ((uint16_t)0x010F)


};
    static const uint8_t VL53L4CD_DEFAULT_CONFIGURATION[] = {
        #ifdef VL53L4CD_I2C_FAST_MODE_PLUS
	0x12, /* 0x2d : set bit 2 and 5 to 1 for fast plus mode (1MHz I2C),
	 else don't touch */
	 #else
	0x00, /* 0x2d : set bit 2 and 5 to 1 for fast plus mode (1MHz I2C),
	 else don't touch */
	 #endif
	0x00, /* 0x2e : bit 0 if I2C pulled up at 1.8V, else set bit 0 to 1
	 (pull up at AVDD) */
	0x00, /* 0x2f : bit 0 if GPIO pulled up at 1.8V, else set bit 0 to 1
	(pull up at AVDD) */
	0x11, /* 0x30 : set bit 4 to 0 for active high interrupt and 1 for active low
	(bits 3:0 must be 0x1), use SetInterruptPolarity() */
	0x02, /* 0x31 : bit 1 = interrupt depending on the polarity,
	use CheckForDataReady() */
	0x00, /* 0x32 : not user-modifiable */
	0x02, /* 0x33 : not user-modifiable */
	0x08, /* 0x34 : not user-modifiable */
	0x00, /* 0x35 : not user-modifiable */
	0x08, /* 0x36 : not user-modifiable */
	0x10, /* 0x37 : not user-modifiable */
	0x01, /* 0x38 : not user-modifiable */
	0x01, /* 0x39 : not user-modifiable */
	0x00, /* 0x3a : not user-modifiable */
	0x00, /* 0x3b : not user-modifiable */
	0x00, /* 0x3c : not user-modifiable */
	0x00, /* 0x3d : not user-modifiable */
	0xff, /* 0x3e : not user-modifiable */
	0x00, /* 0x3f : not user-modifiable */
	0x0F, /* 0x40 : not user-modifiable */
	0x00, /* 0x41 : not user-modifiable */
	0x00, /* 0x42 : not user-modifiable */
	0x00, /* 0x43 : not user-modifiable */
	0x00, /* 0x44 : not user-modifiable */
	0x00, /* 0x45 : not user-modifiable */
	0x20, /* 0x46 : interrupt configuration 0->level low detection, 1-> level high,
	2-> Out of window, 3->In window, 0x20-> New sample ready , TBC */
	0x0b, /* 0x47 : not user-modifiable */
	0x00, /* 0x48 : not user-modifiable */
	0x00, /* 0x49 : not user-modifiable */
	0x02, /* 0x4a : not user-modifiable */
	0x14, /* 0x4b : not user-modifiable */
	0x21, /* 0x4c : not user-modifiable */
	0x00, /* 0x4d : not user-modifiable */
	0x00, /* 0x4e : not user-modifiable */
	0x05, /* 0x4f : not user-modifiable */
	0x00, /* 0x50 : not user-modifiable */
	0x00, /* 0x51 : not user-modifiable */
	0x00, /* 0x52 : not user-modifiable */
	0x00, /* 0x53 : not user-modifiable */
	0xc8, /* 0x54 : not user-modifiable */
	0x00, /* 0x55 : not user-modifiable */
	0x00, /* 0x56 : not user-modifiable */
	0x38, /* 0x57 : not user-modifiable */
	0xff, /* 0x58 : not user-modifiable */
	0x01, /* 0x59 : not user-modifiable */
	0x00, /* 0x5a : not user-modifiable */
	0x08, /* 0x5b : not user-modifiable */
	0x00, /* 0x5c : not user-modifiable */
	0x00, /* 0x5d : not user-modifiable */
	0x01, /* 0x5e : not user-modifiable */
	0xcc, /* 0x5f : not user-modifiable */
	0x07, /* 0x60 : not user-modifiable */
	0x01, /* 0x61 : not user-modifiable */
	0xf1, /* 0x62 : not user-modifiable */
	0x05, /* 0x63 : not user-modifiable */
	0x00, /* 0x64 : Sigma threshold MSB (mm in 14.2 format for MSB+LSB),
	 use SetSigmaThreshold(), default value 90 mm  */
	0xa0, /* 0x65 : Sigma threshold LSB */
	0x00, /* 0x66 : Min count Rate MSB (MCPS in 9.7 format for MSB+LSB),
	 use SetSignalThreshold() */
	0x80, /* 0x67 : Min count Rate LSB */
	0x08, /* 0x68 : not user-modifiable */
	0x38, /* 0x69 : not user-modifiable */
	0x00, /* 0x6a : not user-modifiable */
	0x00, /* 0x6b : not user-modifiable */
	0x00, /* 0x6c : Intermeasurement period MSB, 32 bits register,
	 use SetIntermeasurementInMs() */
	0x00, /* 0x6d : Intermeasurement period */
	0x0f, /* 0x6e : Intermeasurement period */
	0x89, /* 0x6f : Intermeasurement period LSB */
	0x00, /* 0x70 : not user-modifiable */
	0x00, /* 0x71 : not user-modifiable */
	0x00, /* 0x72 : distance threshold high MSB (in mm, MSB+LSB),
	 use SetD:tanceThreshold() */
	0x00, /* 0x73 : distance threshold high LSB */
	0x00, /* 0x74 : distance threshold low MSB ( in mm, MSB+LSB),
	 use SetD:tanceThreshold() */
	0x00, /* 0x75 : distance threshold low LSB */
	0x00, /* 0x76 : not user-modifiable */
	0x01, /* 0x77 : not user-modifiable */
	0x07, /* 0x78 : not user-modifiable */
	0x05, /* 0x79 : not user-modifiable */
	0x06, /* 0x7a : not user-modifiable */
	0x06, /* 0x7b : not user-modifiable */
	0x00, /* 0x7c : not user-modifiable */
	0x00, /* 0x7d : not user-modifiable */
	0x02, /* 0x7e : not user-modifiable */
	0xc7, /* 0x7f : not user-modifiable */
	0xff, /* 0x80 : not user-modifiable */
	0x9B, /* 0x81 : not user-modifiable */
	0x00, /* 0x82 : not user-modifiable */
	0x00, /* 0x83 : not user-modifiable */
	0x00, /* 0x84 : not user-modifiable */
	0x01, /* 0x85 : not user-modifiable */
	0x00, /* 0x86 : clear interrupt, use ClearInterrupt() */
	0x00  /* 0x87 : start ranging, use StartRanging() or StopRanging(),
	 If you want an automatic start after VL53L4CD_init() call,
	  put 0x40 in location 0x87 */
    };
}
