// Pin definitions for the ESP32 controller to interface with DRV8xx2 devices

#ifndef PINS_H_
#define PINS_H_


// SPI Pin Definitions
#define SPI_SDI_PIN         21    // GPIO23: SPI Slave Data In
#define SPI_SDO_PIN         22    // GPIO22: SPI Slave Data Out
#define SPI_SCK_PIN         17    // GPIO33: SPI Clock
#define SPI_nSCS_PIN        23    // GPIO21: SPI Chip Select

// Control Pin Definitions
#define nSLEEP_PIN          33    // GPIO19: nSLEEP control
#define ENABLE_PIN          25    // GPIO18: ENABLE control
#define DIR_PIN             13    // GPIO17: DIR control
#define STEP_PIN            16    // GPIO16: STEP control


#define STEPS_PER_REVOLUTION 200 * 16// 1.8 degree step angle = 200 steps per revolution

// value from 0-255 to set the motor current
#define RUN_MOTOR_CURRENT 120 
#define HOLD_MOTOR_CURRENT 80 
#define STEPS_PER_REVOLUTION 200 * 16 // 1.8 degree step angle = 200 steps per revolution, 16x microstepping = 3200 steps per revolution

// Auto torque configuration
#define ATQ_ENABLE 1
#define ATQ_TRQ_MIN_CURRENT 20 
#define ATQ_TRQ_MAX_CURRENT 80

// Use previously learned ATQ constants (production mode)
#define ATQ_USE_LEARNED_PARAMS 1
#define ATQ_LEARNED_CONST1 1280
#define ATQ_LEARNED_CONST2 91
#define ATQ_LEARNED_MIN_CURRENT_CODE 15
#define ATQ_LEARNED_STEP_CODE 2
#define ATQ_LEARNED_CYCLE_SELECT_CODE 3

// Auto torque learning routine configuration
#if ATQ_USE_LEARNED_PARAMS
#define ATQ_RUN_LEARNING_ON_STARTUP 0
#else
#define ATQ_RUN_LEARNING_ON_STARTUP 1
#endif
#define ATQ_LRN_MIN_CURRENT_CODE (RUN_MOTOR_CURRENT / 8) // Initial learning current = code * 8
#define ATQ_LRN_STEP_CODE 2                              // 00:128, 01:16, 10:32, 11:64
#define ATQ_LRN_CYCLE_SELECT_CODE 3                      // 00:8, 01:16, 10:24, 11:32 half-cycles
#define ATQ_ERROR_TRUNCATE_CODE 0                        // bits truncated from ATQ error before PD loop
#define ATQ_LRN_MOTION_SETTLE_MS 250                     // wait for steady-state motion before LRN_START
#define ATQ_LRN_TIMEOUT_MS 1500


#endif // PINS_H_h