// Pin definitions for the ESP32 controller to interface with DRV8xx2 devices

#ifndef PINS_H_
#define PINS_H_


// SPI Pin Definitions
#define SPI_SDI_PIN         23    // GPIO23: SPI Slave Data In
#define SPI_SDO_PIN         22    // GPIO22: SPI Slave Data Out
#define SPI_SCK_PIN         33    // GPIO33: SPI Clock
#define SPI_nSCS_PIN        21    // GPIO21: SPI Chip Select

// Control Pin Definitions
#define nSLEEP_PIN          19    // GPIO19: nSLEEP control
#define ENABLE_PIN          18    // GPIO18: ENABLE control
#define DIR_PIN             17    // GPIO17: DIR control
#define STEP_PIN            16    // GPIO16: STEP control

#endif // PINS_H_