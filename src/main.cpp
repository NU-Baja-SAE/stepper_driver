#include <Arduino.h>

#include "drv8xx2-RegMap.h"
#include "drv8xx2.h"
#include "pins.h"

#include <SPI.h>
#include "driver/rmt.h"  // for step pulse generation using RMT peripheral
#include "soc/rmt_reg.h" // for RMT register definitions

#define RMT_CHANNEL RMT_CHANNEL_0
#define MAX_PULSES 1000 // maximum number of pulses to send in one batch (adjust as needed)

// SPI Protocol
#define SPI_ADDRESS_MASK 0x3F00 // Mask for SPI register address bits
#define SPI_ADDRESS_POS 8       // Position for SPI register address bits
#define SPI_DATA_MASK 0x00FF    // Mask for SPI register data bits
#define SPI_DATA_POS 0          // Position for SPI register data bits
#define SPI_RW_BIT_MASK 0x4000  // Mask for SPI register read write indication bit

SPIClass *vspi = NULL;

rmt_item32_t pulse_buf[MAX_PULSES];

static const int spiClk = 1000000; // 1 MHz

void spiCommand(SPIClass *spi, byte data)
{
    // use it as you would the regular arduino SPI API
    spi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE1));
    digitalWrite(spi->pinSS(), LOW); // pull SS low to prep other end for transfer
    spi->transfer(data);
    digitalWrite(spi->pinSS(), HIGH); // pull ss high to signify end of data transfer
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

    reg_value |= ((address << SPI_ADDRESS_POS) & SPI_ADDRESS_MASK); // Adding register address value
    reg_value |= ((data << SPI_DATA_POS) & SPI_DATA_MASK);          // Adding data value

    spi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE1));
    digitalWrite(spi->pinSS(), LOW); // pull SS low to prep other end for transfer

    uint16_t received = spi->transfer16(reg_value);

    digitalWrite(spi->pinSS(), HIGH); // pull ss high to signify end of data transfer
    spi->endTransaction();

    uint8_t dataMSB = (received >> 8) & 0xFF;

    // check that first 2 bits are set
    if ((dataMSB & 0xC0) != 0xC0)
    {
        Serial.println("SPI write error: invalid response header!");
        Serial.print("Received MSB: 0x");
        Serial.println(dataMSB, HEX);
    }
    // check fault bits of MSB
    if (dataMSB & UVLO_MASK || dataMSB & CPUV_MASK || dataMSB & OCP_MASK || dataMSB & STL_MASK || dataMSB & OT_MASK)
    {
        Serial.println("Fault detected during SPI read!");
    }

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

    reg_value |= ((address << SPI_ADDRESS_POS) & SPI_ADDRESS_MASK); // Configure register address value
    reg_value |= SPI_RW_BIT_MASK;                                   // Set R/W bit

    spi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE1));
    digitalWrite(spi->pinSS(), LOW); // pull SS low to prep other end for transfer

    uint16_t received = spi->transfer16(reg_value);
    digitalWrite(spi->pinSS(), HIGH); // pull ss high to signify end of data transfer
    spi->endTransaction();

    uint8_t dataMSB = (received >> 8) & 0xFF;
    uint8_t dataLSB = received & 0xFF;

    // check that first 2 bits are set
    if ((dataMSB & 0xC0) != 0xC0)
    {
        Serial.println("SPI read error: invalid response header!");
        Serial.print("Received MSB: 0x");
        Serial.println(dataMSB, HEX);
    }
    // check fault bits of MSB
    if (dataMSB & UVLO_MASK || dataMSB & CPUV_MASK || dataMSB & OCP_MASK || dataMSB & STL_MASK || dataMSB & OT_MASK)
    {
        Serial.println("Fault detected during SPI read!");
    }

    reg_value = ((((dataMSB << 8) | dataLSB) & SPI_DATA_MASK) >> SPI_DATA_POS); // complete data
    return (reg_value);
}

void setup_rmt_stepper()
{
    rmt_config_t config;
    config.rmt_mode = RMT_MODE_TX;
    config.channel = RMT_CHANNEL;
    config.gpio_num = (gpio_num_t)STEP_PIN;
    config.mem_block_num = 1;
    config.clk_div = 80; // 80MHz / 80 = 1MHz resolution (1 tick = 1 microsecond)
    config.tx_config.loop_en = false;
    config.tx_config.carrier_en = false;
    config.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
    config.tx_config.idle_output_en = true;

    rmt_config(&config);
    rmt_driver_install(RMT_CHANNEL, 0, 0);
}

void move_steps_rmt(uint32_t steps, uint32_t speed_hz)
{
    if (steps == 0)
        return;
    if (steps > MAX_PULSES)
    {
        Serial.println("Warning: steps exceed MAX_PULSES, truncating to MAX_PULSES");
        steps = MAX_PULSES;
    }

    // Calculate pulse duration in microseconds
    // For a 50% duty cycle: Period = 1,000,000 / speed_hz
    uint32_t duration_us = 1000000 / speed_hz / 2;

    // Limit duration to RMT max (15-bit value, max 32767)
    if (duration_us > 32767)
        duration_us = 32767;

    // Create the pulse item: High for duration_us, Low for duration_us
    rmt_item32_t pulse = {{{(uint16_t)duration_us, 1, (uint16_t)duration_us, 0}}};

    // Allocate a buffer for the steps
    // rmt_item32_t* items = (rmt_item32_t*) malloc(sizeof(rmt_item32_t) * steps);

    for (int i = 0; i < steps; i++)
    {
        pulse_buf[i] = pulse;
    }

    // Send the items (this is non-blocking)
    rmt_write_items(RMT_CHANNEL, pulse_buf, steps, false);

    // // Wait for transmission to finish (optional, makes this function blocking)
    // rmt_wait_tx_done(RMT_CHANNEL, portMAX_DELAY);

    // free(items);
}

void setup()
{
    Serial.begin(115200);

    // initialise the SPIClass
    vspi = new SPIClass(VSPI);
    vspi->begin(SPI_SCK_PIN, SPI_SDO_PIN, SPI_SDI_PIN, SPI_nSCS_PIN); // SCLK, MISO, MOSI, SS

    pinMode(nSLEEP_PIN, OUTPUT);
    pinMode(ENABLE_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
    pinMode(STEP_PIN, OUTPUT);
    pinMode(SPI_nSCS_PIN, OUTPUT);

    digitalWrite(nSLEEP_PIN, HIGH);   // Wake up the driver
    digitalWrite(SPI_nSCS_PIN, HIGH); // Set SS high
    // wait t_wake = 1.5 ms
    delayMicroseconds(2000);
    digitalWrite(ENABLE_PIN, LOW); // disable the driver

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

    // read CTRL3
    uint16_t ctrl3Reg = spi_readRegister(vspi, SPI_CTRL3);
    Serial.print("CTRL3 Register: 0x");
    Serial.println(ctrl3Reg, HEX);

    // enable open load detection
    uint16_t ctrl9 = spi_readRegister(vspi, SPI_CTRL9);
    Serial.print("CTRL9 Register before OLD set: 0x");
    Serial.println(ctrl9, HEX);
    ctrl9 |= OLD_MASK; // set OLD bit
    spi_writeRegister(vspi, SPI_CTRL9, ctrl9);

    // write to CTRL10 to set idle current to 10% (0.1 * 255 = 25.5 ~ 26)
    spi_writeRegister(vspi, SPI_CTRL10, 26); // set idle current to 10%
    // read CTRL10 to make sure idle current setting is correct
    uint16_t ctrl10Reg = spi_readRegister(vspi, SPI_CTRL10);
    Serial.print("CTRL10 Register: 0x");
    Serial.println(ctrl10Reg, HEX);

    // write to CTRL11 to set current to 10% (0.1 * 255 = 25.5 ~ 26)
    spi_writeRegister(vspi, SPI_CTRL11, 26); // set torque to 10%

    // read CTRL11 to make sure torque setting is correct
    uint16_t ctrl11Reg = spi_readRegister(vspi, SPI_CTRL11);
    Serial.print("CTRL11 Register: 0x");
    Serial.println(ctrl11Reg, HEX);

    // if torque setting is incorrect, do not proceed
    if (ctrl11Reg != 26)
    {
        Serial.println("Torque setting failed, halting!");
        while (1)
            ;
    }

    // Use internal Vref
    uint16_t ctrl13 = spi_readRegister(vspi, SPI_CTRL13);
    Serial.print("CTRL13 Register before VREF set: 0x");
    Serial.println(ctrl13, HEX);
    ctrl13 |= VREF_MASK; // set VREF bit
    spi_writeRegister(vspi, SPI_CTRL13, ctrl13);

    delayMicroseconds(2000);        // wait t_en = 500 us
    digitalWrite(ENABLE_PIN, HIGH); // enable the driver
    delay(1000);                    // wait t_en = 500 us

    // enable the driver
    ctrl1Reg = spi_readRegister(vspi, SPI_CTRL1);
    Serial.print("CTRL1 Register before EN_OUT set: 0x");
    Serial.println(ctrl1Reg, HEX);
    ctrl1Reg |= EN_OUT_MASK; // set EN_OUT bit
    spi_writeRegister(vspi, SPI_CTRL1, ctrl1Reg);

    // read the fault register
    faultReg = spi_readRegister(vspi, SPI_FAULT);
    Serial.print("Fault Register: 0x");
    Serial.println(faultReg, HEX);

    ctrl1Reg = spi_readRegister(vspi, SPI_CTRL1);
    Serial.print("CTRL1 Register after EN_OUT set: 0x");
    Serial.println(ctrl1Reg, HEX);

    setup_rmt_stepper();
}

#define LOOP_DELAY_MS 10
#define SINE_FREQ_HZ 0.25  // Frequency of sine wave in Hz
#define SINE_AMPLITUDE 750 // Amplitude of sine wave in steps

void loop()
{
    static unsigned long faultTimer = 0;
    faultTimer += 1;
    if (faultTimer > 1000)
    {
        uint16_t faultReg = spi_readRegister(vspi, SPI_FAULT);
        Serial.print("Fault Register: 0x");
        Serial.println(faultReg, HEX);
        faultTimer = 0;
    }

    static float pos = 0.0;
    static int previous_pos = 0;

    pos = sin(2 * PI * SINE_FREQ_HZ * millis() / 1000.0) * SINE_AMPLITUDE;
    int steps = (int)pos - (int)previous_pos;
    previous_pos = pos;
    int speed_hz = abs(steps) / ((LOOP_DELAY_MS - 1) / 1000.0); // steps per second

    Serial.printf(">Pos:%.2f\n>Steps:%d\n>Speed:%d\n", pos, steps, speed_hz);

    move_steps_rmt(abs(steps), speed_hz);           // Move at 1kHz
    digitalWrite(DIR_PIN, steps >= 0 ? HIGH : LOW); // Set direction

    delay(LOOP_DELAY_MS);

    // check status of rmt
    esp_err_t rmt_status = rmt_wait_tx_done(RMT_CHANNEL, 0); // check if transmission is done (non-blocking)
    if (rmt_status == ESP_ERR_TIMEOUT)
    {
        Serial.println("RMT transmission still in progress...");
    }
    else if (rmt_status == ESP_OK)
    {
        Serial.println("RMT transmission completed.");
    }
    else
    {
        Serial.printf("RMT error: %d\n", rmt_status);
    }
}

