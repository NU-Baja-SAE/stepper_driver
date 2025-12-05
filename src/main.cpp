#include <Arduino.h>

#include "drv8xx2-RegMap.h"
#include "drv8xx2.h"
#include "pins.h"

#include <SPI.h>

// SPI Protocol
#define SPI_ADDRESS_MASK   0x3F00        // Mask for SPI register address bits
#define SPI_ADDRESS_POS    8             // Position for SPI register address bits
#define SPI_DATA_MASK      0x00FF        // Mask for SPI register data bits
#define SPI_DATA_POS       0             // Position for SPI register data bits
#define SPI_RW_BIT_MASK    0x4000        // Mask for SPI register read write indication bit

SPIClass * vspi = NULL;

static const int spiClk = 1000000;  // 1 MHz

void spiCommand(SPIClass *spi, byte data) {
  //use it as you would the regular arduino SPI API
  spi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
  digitalWrite(spi->pinSS(), LOW);  //pull SS low to prep other end for transfer
  spi->transfer(data);
  digitalWrite(spi->pinSS(), HIGH);  //pull ss high to signify end of data transfer
  spi->endTransaction();
}

// This SPI function is used to write the set device configurations and operating
// parameters of the device.
// Register format |R/W|A5|A4|A3|A2|A1|A0|*|D7|D6|D5|D4|D3|D2|D1|D0|
// Ax is address bit, Dx is data bits and R/W is read write bit.
// For write R/W bit should 0.
uint16_t spi_writeRegister(SPIClass *spi, uint8_t address, uint16_t data)
{
  volatile uint16_t reg_value = 0;

  reg_value |= ((address << SPI_ADDRESS_POS) & SPI_ADDRESS_MASK);         // Adding register address value
  reg_value |= ((data << SPI_DATA_POS) & SPI_DATA_MASK);                  // Adding data value


  
  spi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
  digitalWrite(spi->pinSS(), LOW);  //pull SS low to prep other end for transfer
  
  // spi->transfer((uint8_t)((reg_value>>8) & 0xFF));  // Transmit first Byte,MSB-Byte
  // spi->transfer((uint8_t)(reg_value & 0xFF));       // Transmit Second Byte, LSB-Byte
  spi->transfer16(reg_value);

  digitalWrite(spi->pinSS(), HIGH);  //pull ss high to signify end of data transfer
  spi->endTransaction();
  

  // if(address == SPI_CTRL2)
  //     DetermineMicrostepping(data);

  return 0;
}


void setup() {
  //initialise the SPIClass
  vspi = new SPIClass(VSPI);
  vspi->begin(SPI_SCK_PIN, SPI_SDI_PIN, SPI_SDO_PIN, SPI_nSCS_PIN); //SCLK, MISO, MOSI, SS

  pinMode(nSLEEP_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(SPI_nSCS_PIN, OUTPUT);



}

void loop() {
  // put your main code here, to run repeatedly:
}

