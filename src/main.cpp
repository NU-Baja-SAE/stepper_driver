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
    // stepper.enable();
   
}

#define LOOP_DELAY_MS 10
#define SINE_FREQ_HZ 0.25  // Frequency of sine wave in Hz
#define SINE_AMPLITUDE 750 // Amplitude of sine wave in steps

void loop()
{
    // static unsigned long faultTimer = 0;
    // faultTimer += 1;
    // if (faultTimer > 1000)
    // {
    //     uint16_t faultReg = stepper.readFault();
    //     if (faultReg != 0)
    //     {
    //         Serial.printf("Fault detected! Fault Register: 0x%X\n", faultReg);
    //         stepper.faultDetected();
    //     }
    //     faultTimer = 0;
    // }

    // static float pos = 0.0;
    // static int previous_pos = 0;

    // pos = sin(2 * PI * SINE_FREQ_HZ * millis() / 1000.0) * SINE_AMPLITUDE;
    // int steps = (int)pos - (int)previous_pos;
    // previous_pos = pos;
    // int speed_hz = abs(steps) / ((LOOP_DELAY_MS ) / 1000.0); // steps per second

    // Serial.printf(">Pos:%.2f\n>Steps:%d\n>Speed:%d\n", pos, steps, speed_hz);

    // stepper.moveSteps(abs(steps), speed_hz);
    // digitalWrite(DIR_PIN, steps >= 0 ? HIGH : LOW); // Set direction

    // delay(LOOP_DELAY_MS);

    // // check status of rmt
    // esp_err_t rmt_status = rmt_wait_tx_done(RMT_CHANNEL, 0); // check if transmission is done (non-blocking)
    // if (rmt_status == ESP_ERR_TIMEOUT)
    // {
    //     Serial.println("RMT transmission still in progress...");
    // }
    // else if (rmt_status == ESP_OK)
    // {
    //     Serial.println("RMT transmission completed.");
    // }
    // else
    // {
    //     Serial.printf("RMT error: %d\n", rmt_status);
    // }
}

