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
  spi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE1));
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


  
  spi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE1));
  digitalWrite(spi->pinSS(), LOW);  //pull SS low to prep other end for transfer
  
  spi->transfer16(reg_value);

  digitalWrite(spi->pinSS(), HIGH);  //pull ss high to signify end of data transfer
  spi->endTransaction();
  
  return 0;
}

// This SPI function is used to read the device configurations, parameters and
// status information for S version of device.
// Register format |R/W|A5|A4|A3|A2|A1|A0|*|D7|D6|D5|D4|D3|D2|D1|D0|
// Ax is address bit, Dx is data bits and R/W is read write bit.
// For read R/W bit should be 1.  
uint16_t spi_readRegister(SPIClass *spi, uint8_t address)
{
  volatile uint16_t reg_value = 0;

  reg_value |= ((address << SPI_ADDRESS_POS) & SPI_ADDRESS_MASK);         // Configure register address value
  reg_value |= SPI_RW_BIT_MASK;                                           // Set R/W bit

  spi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE1));
  digitalWrite(spi->pinSS(), LOW);  //pull SS low to prep other end for transfer
  
  uint16_t received = spi->transfer16(reg_value);
  digitalWrite(spi->pinSS(), HIGH);  //pull ss high to signify end of data transfer
  spi->endTransaction();

  
  uint8_t dataMSB = (received >> 8) & 0xFF;
  uint8_t dataLSB = received & 0xFF;
  
  // check that first 2 bits are set
  if ( (dataMSB & 0xC0) != 0xC0 ) {
    Serial.println("SPI read error: invalid response header!");
    Serial.print("Received MSB: 0x");
    Serial.println(dataMSB, HEX);
  }
  // check fault bits of MSB
  if (dataMSB & UVLO_MASK || dataMSB & CPUV_MASK || dataMSB & OCP_MASK || dataMSB & STL_MASK || dataMSB & OT_MASK) {
    Serial.println("Fault detected during SPI read!");
  }


  reg_value = ((((dataMSB<<8) | dataLSB) & SPI_DATA_MASK)>>SPI_DATA_POS); // complete data
  return (reg_value);
}


void setup() 
{
  Serial.begin(115200);

  //initialise the SPIClass
  vspi = new SPIClass(VSPI);
  vspi->begin(SPI_SCK_PIN, SPI_SDO_PIN, SPI_SDI_PIN, SPI_nSCS_PIN); //SCLK, MISO, MOSI, SS

  pinMode(nSLEEP_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(SPI_nSCS_PIN, OUTPUT);

  digitalWrite(nSLEEP_PIN, HIGH); // Wake up the driver
  digitalWrite(SPI_nSCS_PIN, HIGH); // Set SS high
  // wait t_wake = 1.5 ms
  delayMicroseconds(2000);
  digitalWrite(ENABLE_PIN, LOW);   // disable the driver


  // read fault register
  uint16_t faultReg = spi_readRegister(vspi, SPI_FAULT);
  Serial.print("Fault Register: 0x");
  Serial.println(faultReg, HEX);


  // read SPI_DIAG1
  uint16_t diag1Reg = spi_readRegister(vspi, SPI_DIAG1);
  Serial.print("DIAG1 Register: 0x");
  Serial.println(diag1Reg, HEX);

  // read SPI_DIAG2
  uint16_t diag2Reg = spi_readRegister(vspi, SPI_DIAG2);
  Serial.print("DIAG2 Register: 0x");
  Serial.println(diag2Reg, HEX);  
  
  // read SPI_DIAG3
  uint16_t diag3Reg = spi_readRegister(vspi, SPI_DIAG3);
  Serial.print("DIAG3 Register: 0x");
  Serial.println(diag3Reg, HEX);  

  // read CTRL 1
  uint16_t ctrl1Reg = spi_readRegister(vspi, SPI_CTRL1);
  Serial.print("CTRL1 Register: 0x");
  Serial.println(ctrl1Reg, HEX);

  // read CTRL2 
  uint16_t ctrl2Reg = spi_readRegister(vspi, SPI_CTRL2);
  Serial.print("CTRL2 Register: 0x");
  Serial.println(ctrl2Reg, HEX);

  // enable open load detection
  uint16_t ctrl9 = spi_readRegister(vspi, SPI_CTRL9);
  Serial.print("CTRL9 Register before OLD set: 0x");
  Serial.println(ctrl9, HEX);
  ctrl9 |= OLD_MASK; // set OLD bit
  spi_writeRegister(vspi, SPI_CTRL9, ctrl9);

  // read CTRL11 to make sure torque setting is correct
  uint16_t ctrl11Reg = spi_readRegister(vspi, SPI_CTRL11);
  Serial.print("CTRL11 Register: 0x");
  Serial.println(ctrl11Reg, HEX);

  // Use internal Vref
  uint16_t ctrl13 = spi_readRegister(vspi, SPI_CTRL13);
  Serial.print("CTRL13 Register before VREF set: 0x");
  Serial.println(ctrl13, HEX);
  ctrl13 |= VREF_MASK; // set VREF bit
  spi_writeRegister(vspi, SPI_CTRL13, ctrl13);



  // enable the driver
  uint16_t ctrl1 = spi_readRegister(vspi, SPI_CTRL1);
  Serial.print("CTRL1 Register before EN_OUT set: 0x");
  Serial.println(ctrl1, HEX);
  ctrl1 |= EN_OUT_MASK; // set EN_OUT bit
  spi_writeRegister(vspi, SPI_CTRL1, ctrl1);
  
  ctrl1 = spi_readRegister(vspi, SPI_CTRL1);
  Serial.print("CTRL1 Register after EN_OUT set: 0x");
  Serial.println(ctrl1, HEX);

  digitalWrite(ENABLE_PIN, HIGH);   // enable the driver



}

void loop() 
{
  // put your main code here, to run repeatedly:
  // // toggle step pin
  // digitalWrite(STEP_PIN, HIGH);
  // delayMicroseconds(500);
  // digitalWrite(STEP_PIN, LOW);
  // delayMicroseconds(500);
  // delay(100);
  // // read the fault register
  // uint16_t faultReg = spi_readRegister(vspi, SPI_FAULT);
  // // read the index register
  // Serial.print("Fault Register: 0x");
  // Serial.print(faultReg, HEX);

  // uint16_t indexReg = spi_readRegister(vspi, SPI_INDEX1);
  // Serial.print(" | Index1 Register: 0x");
  // Serial.println(indexReg, HEX);
}

