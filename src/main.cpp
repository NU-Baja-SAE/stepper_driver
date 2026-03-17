#include <Arduino.h>
#include "DRV8462.h"

#include "DRV8462_REGMAP.h"
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

#if ATQ_USE_LEARNED_PARAMS
    Serial.println("ATQ production mode: using saved learned parameters.");
#endif
}

#define LOOP_DELAY_MS 10
#define ATQ_LEARN_STEP_BATCH 100
#define ATQ_LEARN_SPEED_HZ 1000
#define ATQ_LEARN_MAX_RUN_MS 120000
#define ATQ_LEARN_MIN_POS_STEPS 0
#define ATQ_LEARN_MAX_POS_STEPS 1000

void loop()
{
#if ATQ_USE_LEARNED_PARAMS
    static unsigned long productionFaultPollMs = 0;
    if ((millis() - productionFaultPollMs) > 500)
    {
        productionFaultPollMs = millis();
        uint16_t faultReg = stepper.readFault();
        if (faultReg != 0)
        {
            uint16_t diag2Fault = stepper.readDiag2();
            Serial.printf("Fault detected! fault=0x%X diag2=0x%02X\n", faultReg, diag2Fault);
            stepper.faultDetected();
        }
    }

    delay(50);
    return;
#endif

    static bool learnStarted = false;
    static bool learnFinished = false;
    static unsigned long learnStartMs = 0;
    static unsigned long faultPollMs = 0;
    static unsigned long statusPrintMs = 0;
    static int currentPositionSteps = 0;
    static int direction = 1;

    if (learnFinished)
    {
        delay(1000);
        return;
    }

    if (!learnStarted)
    {
        learnStarted = true;
        learnStartMs = millis();
        Serial.println("ATQ learning motion started");
    }

    esp_err_t txStatus = rmt_wait_tx_done(RMT_CHANNEL, 0);
    bool motorBusy = (txStatus == ESP_ERR_TIMEOUT);
    if (txStatus != ESP_OK && txStatus != ESP_ERR_TIMEOUT)
    {
        Serial.printf("RMT status error: %d\n", txStatus);
    }

    if (!motorBusy)
    {
        int commandedSteps = direction * ATQ_LEARN_STEP_BATCH;
        int candidatePosition = currentPositionSteps + commandedSteps;

        if (candidatePosition > ATQ_LEARN_MAX_POS_STEPS)
        {
            commandedSteps = ATQ_LEARN_MAX_POS_STEPS - currentPositionSteps;
            direction = -1;
        }
        else if (candidatePosition < ATQ_LEARN_MIN_POS_STEPS)
        {
            commandedSteps = ATQ_LEARN_MIN_POS_STEPS - currentPositionSteps;
            direction = 1;
        }

        if (commandedSteps == 0)
        {
            delay(LOOP_DELAY_MS);
            return;
        }

        stepper.moveSteps(commandedSteps, ATQ_LEARN_SPEED_HZ);
        currentPositionSteps += commandedSteps;
    }

    uint16_t diag2 = stepper.readDiag2();
    if ((diag2 & ATQ_LRN_DONE_MASK) != 0)
    {
        stepper.printAtqLearnedParameters();
        stepper.stop();
        stepper.disable();
        learnFinished = true;
        Serial.println("ATQ learning complete. Motor stopped.");
        return;
    }

    if ((millis() - learnStartMs) > ATQ_LEARN_MAX_RUN_MS)
    {
        stepper.stop();
        stepper.disable();
        learnFinished = true;
        Serial.println("ATQ learning did not complete within safety timeout. Motor stopped.");
        return;
    }

    if ((millis() - statusPrintMs) > 1000)
    {
        statusPrintMs = millis();
        Serial.printf("ATQ learning in progress. pos=%d diag2=0x%02X\n", currentPositionSteps, diag2);
    }

    if ((millis() - faultPollMs) > 250)
    {
        faultPollMs = millis();
        uint16_t faultReg = stepper.readFault();
        if (faultReg != 0)
        {
            uint16_t diag2Fault = stepper.readDiag2();

            if ((faultReg & OT_MASK) && (diag2Fault & OTW_MASK) && !(diag2Fault & OTS_MASK))
            {
                Serial.printf("Thermal warning (OTW). fault=0x%X diag2=0x%02X\n", faultReg, diag2Fault);
            }
            else
            {
                Serial.printf("Fault detected! fault=0x%X diag2=0x%02X\n", faultReg, diag2Fault);
                stepper.stop();
                stepper.disable();
                learnFinished = true;
                stepper.faultDetected();
                return;
            }
        }
    }

    delay(LOOP_DELAY_MS);
}

