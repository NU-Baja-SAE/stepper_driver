#include <Arduino.h>
#include "DRV8462.h"

#include "drv8xx2-RegMap.h"
#include "pins.h"

DRV8462 stepper;

void setup()
{
    Serial.begin(115200);
    stepper.begin();

    uint16_t faultReg = stepper.readFault();
    Serial.printf("Initial Fault Register: 0x%X\n", faultReg);

    Serial.println("Starting stepper control...");
    stepper.enable();

    //   Write 1b to ATQ_EN
    // • Run the motor with no load
    // • Program ATQ_LRN_MIN_CURRENT
    // • Program ATQ_LRN_STEP
    // • Program ATQ_LRN_CYCLE_SELECT
    // • Write 1b to ATQ_LRN_START
    // • The algorithm runs the motor with initial current level for ATQ_LRN_CYCLE_SELECT number of electrical
    // half cycles
    // • Next, the algorithm runs the motor with final current level for ATQ_LRN_CYCLE_SELECT number of
    // electrical half cycles
    // • After learning is complete,
    // – ATQ_LRN_START bit is auto cleared to 0b
    // – ATQ_LRN_DONE bit becomes 1b
    // • ATQ_LRN_CONST1 and ATQ_LRN_CONST2 are populated in their respective registers
    // • Motor current goes to ATQ_TRQ_MAX

    uint16_t atqLrnCtrl10 = stepper.spiReadRegister(SPI_ATQ_CTRL10);
    atqLrnCtrl10 |= 0b10000000; // set ATQ_EN bit
    stepper.spiWriteRegister(SPI_ATQ_CTRL10, atqLrnCtrl10);

    uint16_t atqLrnCtrl15 = stepper.spiReadRegister(SPI_ATQ_CTRL15);
    atqLrnCtrl15 |= 0b00001000; // clear ATQ
    stepper.spiWriteRegister(SPI_ATQ_CTRL15, atqLrnCtrl15); // set ATQ_LRN_STEP to 16 and ATQ_LRN_CYCLE_SELECT to 8

    atqLrnCtrl10 = stepper.spiReadRegister(SPI_ATQ_CTRL10);
    atqLrnCtrl10 |= 0b01000000; // set ATQ_EN and LRN_START bits
    stepper.spiWriteRegister(SPI_ATQ_CTRL10, atqLrnCtrl10);

    stepper.spiWriteRegister(SPI_ATQ_CTRL15, 0x88); // set ATQ_EN bit to start auto torque learning
}

void loop()
{
    // read DIAG2 register to check for LRN_DONE bit
    uint16_t diag2Reg = stepper.spiReadRegister(SPI_DIAG2);
    if (diag2Reg & ATQ_LRN_DONE_MASK)
    {
        Serial.println("Auto Torque Learning complete!");
        // read ATQ_LRN_CONST1 and ATQ_LRN_CONST2 registers
        uint16_t atqLrnConst1 = stepper.spiReadRegister(SPI_ATQ_CTRL8);
        uint16_t atqLrnConst2 = stepper.spiReadRegister(SPI_ATQ_CTRL9);
        Serial.printf("ATQ_LRN_CONST1: 0x%X\n", atqLrnConst1);
        Serial.printf("ATQ_LRN_CONST2: 0x%X\n", atqLrnConst2);
        // stop the motor after learning is complete
        stepper.stop();
        while (true)
        {
        } // halt execution
    }
}
