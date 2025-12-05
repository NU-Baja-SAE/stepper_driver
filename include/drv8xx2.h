/* --COPYRIGHT--,BSD
 * Copyright (c) 2016, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/

/*******************************************************************************
 *
 *  drv8xx2.h - Declaration file for utility functions and global variables
 *  DRV84x2_DRV82x2_DRV89x2EVM_FIRMWARE
 *  02/10/2022
 *  Murugavel Raju
 *
 ******************************************************************************/
#ifndef DRV8xx2_H_
#define DRV8xx2_H_
/******************************************************************************
 * HEADER FILES
 *****************************************************************************/
#include "drv8xx2-RegMap.h"
// #include "msp430.h"

/******************************************************************************
 * MACROS
 *****************************************************************************/
// GPIO Port 1 Definitions
#define MCU_STATUS         BIT0    // P1.0: MCU Status LED output
#define GUI_STATUS         BIT1    // P1.1: GUI Status LED output
#define DRVB_STATUS        BIT2    // P1.2: Spare1 LED output
#define DRVA_STATUS        BIT3    // P1.3: Status LED output
#define nHOME_OCPM         BIT4    // P1.4: nHOME/OCPM input
#define HOME_LED           BIT5    // P1.5: Stepper HOME position LED output
#define STALL_LED          BIT6    // P1.6: Stepper Stall LED output
#define nSLEEP             BIT7    // P1.7: nSLEEP control

#define SINGLE_LED         BIT2    // P1.2: Single H-Bridge mode DRV8252/62 indicator LED
#define DUAL_LED           BIT3    // P1.3: Dual H-Bridge mode DRV8252/62 indicator LED
#define EN1_LED            BIT2    // P1.2: EN1 status LED
#define EN2_LED            BIT3    // P1.3: EN2 status LED
#define EN3_LED            BIT5    // P1.5: EN3 status LED
#define EN4_LED            BIT6    // P1.6: EN4 status LED

// GPIO Port 2 Definitions
#define TP1                BIT0    // P2.0: Test point 1
#define TP2                BIT1    // P2.1: Test point 2
#define TP3                BIT2    // P2.2: Test point 3
#define TP4                BIT3    // P2.3: Test point 4
#define TP5                BIT4    // P2.4: Test point 5
#define TOFF_CTRL          BIT5    // P2.5: TOFF output
#define TOFF_PD            BIT6    // P2.6: TOFF pull-down output
#define nFAULT             BIT7    // P2.7: nFAULT input

#define EN3_PIN            BIT5    // P2.5: EN3 for DRV8952/62

// GPIO Port 3 Definitions
#define TP6                BIT0    // P3.0: Test point 6
#define TA1_0              BIT1    // P3.1: TA1.0 output
#define TP7                BIT2    // P3.2: Test point 7
#define TP8                BIT3    // P3.3: Test point 8
#define TP9                BIT4    // P3.4: Test point 9

// GPIO Port 4 Definitions
#define M1_PD              BIT0    // P4.0: M1 pull-down output
#define ENABLE             BIT1    // P4.1: ENABLE control
#define DIR                BIT2    // P4.2: Direction control
#define STEP               BIT3    // P4.3: STEP output
#define M1                 BIT4    // P4.4: M1 control
#define TP10               BIT5    // P4.5: Test Point TP10
#define TP11               BIT6    // P4.6: Test Point TP11
#define TP12               BIT7    // P4.7: Test Point TP12

#define IN1_PIN            BIT1    // P4.1: IN1 PWM output to driver IN1
#define IN2_PIN            BIT2    // P4.2: IN2 PWM output to driver IN2
#define IN3_PIN            BIT3    // P4.3: IN3 PWM output to driver IN3
#define IN4_PIN            BIT4    // P4.4: IN4 PWM output to driver IN4

// GPIO Port 5 Definitions
#define IDR_3              BIT3    // P5.3: EVM ID3 setting
#define IDR_2              BIT4    // P5.4: EVM ID2 setting
#define IDR_1              BIT5    // P5.5: EVM ID1 setting
#define TP14               BIT6    // P4.6: Test Point TP14
#define TP16               BIT7    // P4.7: Test Point TP16

// GPIO Port 6 Definitions
#define ANALOG1            BIT0    // P6.0: Analog input in BDC variants
#define ANALOG2            BIT1    // P6.1: Analog input in BDC variants
#define ANALOG3            BIT2    // P6.2: Analog input in BDC variants
#define ANALOG4            BIT3    // P6.3: Analog input in BDC variants
#define VSEN_VM            BIT4    // P6.4: A4 - VM V-sense analog input
#define TP13               BIT5    // P6.5: A5 - Test Point TP13
#define M0_nSCS            BIT6    // P6.6: M0 or nSCS control
#define VREF               BIT7    // P6.7: DAC1 - VREF analog output

#define MODE2_CTL          BIT3    // P6.3: MODE2 for DRV8252/62
#define EN4_PIN            BIT6    // P6.6: EN4 for DRV8952/62

// GPIO Port 7 Definitions
#define IDR_4              BIT2    // P7.2: EVM ID4 setting
#define IDR_5              BIT3    // P7.3: EVM ID5 setting
#define VSEN_AOUT1         BIT4    // P7.4: A12 V sense AOUT1 analog input (optional), Test point A1
#define VSEN_AOUT2         BIT5    // P7.5: A13 V sense AOUT1 analog input (optional), Test point A2
#define VSEN_BOUT1         BIT6    // P7.6: A14 V sense AOUT1 analog input (optional), Test point A3
#define VSEN_BOUT2         BIT7    // P7.7: A15 V sense AOUT1 analog input (optional), Test point A4

// GPIO Port 8 Definitions
#define TB0CLK             BIT0    // P8.0: TIMER_B0 clock input
#define UNUSED             BIT1    // P8.1: configure as input
#define UART_TXD           BIT2    // P8.2: UART TXD
#define UART_RXD           BIT3    // P8.3: UART RXD
#define M1_SCLK            BIT4    // P8.4: M1 or UCB1CLK
#define SDI                BIT5    // P8.5: UCB1SIMO
#define DECAY0             BIT5    // P8.5: DECAY0 in Hardware MODE
#define SDO                BIT6    // P8.6: UCB1SOMI
#define DECAY1             BIT6    // P8.6: DECAY1 in Hardware MODE
#define MODE_CTL           BIT7    // P8.7: MODE control output

#define EN1_PIN            BIT5    // P8.5: EN1 for DRV8952/62
#define EN2_PIN            BIT6    // P8.6: EN2 for DRV8952/62
#define MODE1_CTL          BIT7    // P8.7: MODE1 for DRV8252/62

// GPIO Port 9 Definitions
#define TP18               BIT0    // P9.0: Test point 18
#define TA19               BIT1    // P9.1: Test point 19
#define TP20               BIT2    // P9.2: Test point 20
#define TP21               BIT3    // P9.3: Test point 21
#define TP22               BIT4    // P9.4: Test point 22
#define TP23               BIT5    // P9.5: Test point 23
#define TP24               BIT6    // P9.6: Test point 24
#define TP26               BIT7    // P9.7: Test point 26

// MODE GPIO macro
#define set_MODE_hi              P8OUT |= MODE_CTL
#define set_MODE_lo              P8OUT &= ~MODE_CTL

// nSLEEP GPIO macro
#define set_nSLEEP_hi            P1OUT |= nSLEEP
#define set_nSLEEP_lo            P1OUT &= ~nSLEEP

// ENABLE GPIO macro
#define set_ENABLE_hi            P4DIR |= ENABLE; P4OUT |= ENABLE
#define set_ENABLE_lo            P4DIR |= ENABLE; P4OUT &= ~ENABLE
#define set_ENABLE_hiz           P4DIR &= ~ENABLE

// DIR GPIO macro
#define set_DIR_hi               P4OUT |= DIR
#define set_DIR_lo               P4OUT &= ~DIR

// STEP GPIO macro
#define set_STEP_hi               P4OUT |= STEP
#define set_STEP_lo               P4OUT &= ~STEP

#define read_nSLEEP_pin           P1IN & nSLEEP
#define read_ENABLE_pin           P4IN & ENABLE
#define read_nFAULT_pin           P2IN & nFAULT
#define read_MODE_pin             P8IN & MODE_CTL
#define toggle_status_led         P1OUT ^= MCU_STATUS

#define read_ID1_3_pins           P5IN & (IDR_1 | IDR_2 | IDR_3)
#define read_ID4_pin              P7IN & IDR_4
#define read_ID5_pin              P7IN & IDR_5

// Stall detection
#define set_stall_led_hi          P1OUT |= STALL_LED
#define set_stall_led_lo          P1OUT &= ~STALL_LED
#define toggle_stall_led          P1OUT ^= STALL_LED

// Decay mode
#define SLOW_SLOW          0       // Slow Decay
#define SLOW_MIX30         1       // Increasing steps Slow; Decreasing steps Mixed decay30% fast
#define SLOW_MIX60         2       // Increasing steps Slow; Decreasing steps Mixed decay60% fast
#define SLOW_FAST          3       // Increasing steps Slow; Decreasing steps Fast
#define MIX30_MIX30        4       // Mixed Decay 30% fast
#define MIX60_MIX60        5       // Mixed Decay 60% fast
#define STDD_STDD          6       // Smart Tune Dynamic Control
#define STRC_STRC          7       // Smart Tune Ripple Control
#define SILENT_STEP        8       // Silent step decay mode

#define set_slow_slow            P8DIR &= ~DECAY0; P8DIR |= DECAY1; P8OUT |= DECAY1
#define set_slow_mixed30         P8DIR |= DECAY0; P8OUT |= DECAY0; P8DIR = DECAY1; P8OUT |= DECAY1
#define set_slow_mixed60         P8DIR |= DECAY0; P8OUT &= ~DECAY0; P8DIR &= ~DECAY1
#define set_slow_fast            P8DIR |= DECAY0; P8OUT |= DECAY0; P8DIR &= ~DECAY1
#define set_mixed30_mixed30      P8DIR |= DECAY0; P8OUT |= DECAY0; P8DIR = DECAY1; P8OUT &= ~DECAY1
#define set_mixed60_mixed60      P8DIR &= ~DECAY0; P8DIR = DECAY1; P8OUT &= ~DECAY1
#define set_st_dynamic_decay     P8DIR |= DECAY0; P8OUT &= ~DECAY0; P8DIR |= DECAY1; P8OUT &= ~DECAY1
#define set_st_ripple_control    P8OUT &= ~DECAY0; P8OUT |= DECAY1; P8DIR |= DECAY0 | DECAY1
#define set_silent_step_decay    P8DIR &= ~DECAY0; P8DIR &= ~DECAY1

// TOFF
#define TOFF_7u            0       // 7uS TOFF time
#define TOFF_16u           1       // 16uS TOFF time
#define TOFF_24u           2       // 24uS TOFF time
#define TOFF_32u           3       // 32uS TOFF time

// Set TOFF
#define set_TOFF_7us       P2DIR |= TOFF_CTRL; P2DIR &= ~TOFF_PD; P2OUT &= ~TOFF_CTRL
#define set_TOFF_16us      P2DIR |= TOFF_CTRL; P2DIR &= ~TOFF_PD; P2OUT |= TOFF_CTRL
#define set_TOFF_24us      P2DIR &= ~(TOFF_CTRL + TOFF_PD)
#define set_TOFF_32us      P2DIR &= ~TOFF_CTRL; P2DIR |= TOFF_PD; P2OUT &= ~TOFF_PD

// Step Mode
#define FULLSTEP_100       0       // Full step (2-phase excitation) with 100% current
#define FULLSTEP_71        1       // Full step (2-phase excitation) with 71% current
#define NONCIRCLR_uSTEP_2  2       // Non-circular 1/2 step
#define uSTEP_2            3       // 1/2 step
#define uSTEP_4            4       // 1/4 step
#define uSTEP_8            5       // 1/8 step
#define uSTEP_16           6       // 1/16 step
#define uSTEP_32           7       // 1/32 step
#define uSTEP_64           8       // 1/64 step
#define uSTEP_128          9       // 1/128 step
#define uSTEP_256          10      // 1/256 step

// Set Step Mode
#define set_M0_nSCS_hi     P6OUT |= M0_nSCS; P6DIR |= M0_nSCS
#define set_M0_nSCS_lo     P6OUT &= ~M0_nSCS; P6DIR |= M0_nSCS
#define set_M0_nSCS_hiz    P6DIR &= ~M0_nSCS
#define set_M1_hi          P4DIR &= ~M1_PD; P8OUT |= M1_SCLK; P8DIR |= M1_SCLK
#define set_M1_lo          P4DIR &= ~M1_PD; P8OUT &= ~M1_SCLK; P8DIR |= M1_SCLK
#define set_M1_hiz         P8DIR &= ~M1_SCLK; P4DIR &= ~M1_PD
#define set_M1_330k        P8DIR &= ~M1_SCLK; P4OUT &= ~M1_PD; P4DIR |= M1_PD

#define set_fullstep_100          set_M0_nSCS_lo; set_M1_lo
#define set_fullstep_71           set_M0_nSCS_lo; set_M1_330k
#define set_noncircular_halfstep  set_M0_nSCS_hi; set_M1_lo
#define set_halfstep              set_M0_nSCS_hiz; set_M1_lo
#define set_microstep_4           set_M0_nSCS_lo; set_M1_hi
#define set_microstep_8           set_M0_nSCS_hi; set_M1_hi
#define set_microstep_16          set_M0_nSCS_hiz; set_M1_hi
#define set_microstep_32          set_M0_nSCS_lo; set_M1_hiz
#define set_microstep_64          set_M0_nSCS_hiz; set_M1_330k
#define set_microstep_128         set_M0_nSCS_hiz; set_M1_hiz
#define set_microstep_256         set_M0_nSCS_hi; set_M1_hiz

#define read_OCPM_pin             P1IN & nHOME_OCPM
#define set_OCPM_lo               P1OUT &= ~nHOME_OCPM
#define set_OCPM_hi               P1OUT |= nHOME_OCPM
#define read_en1_pin              P8IN & EN1_PIN
#define set_en1_lo                P8OUT &= ~EN1_PIN
#define set_en1_hi                P8OUT |= EN1_PIN
#define read_en2_pin              P8IN & EN2_PIN
#define set_en2_lo                P8OUT &= ~EN2_PIN
#define set_en2_hi                P8OUT |= EN2_PIN
#define read_en3_pin              P2IN & EN3_PIN
#define set_en3_lo                P2OUT &= ~EN3_PIN
#define set_en3_hi                P2OUT |= EN3_PIN
#define read_en4_pin              P6IN & EN4_PIN
#define set_en4_lo                P6OUT &= ~EN4_PIN
#define set_en4_hi                P6OUT |= EN4_PIN
#define read_mode1pin             P8IN & MODE1_CTL
#define set_mode1pin_lo           P8OUT &= ~MODE1_CTL
#define set_mode1pin_hi           P8OUT |= MODE1_CTL
#define read_mode2pin             P6IN & MODE2_CTL
#define set_mode2pin_lo           P6OUT &= ~MODE2_CTL
#define set_mode2pin_hi           P6OUT |= MODE2_CTL

#define _PWM_TIMER_PERIOD 100
#define PWM_TIMER_PERIOD (_PWM_TIMER_PERIOD-1)
#define PWM_SCALE (_PWM_TIMER_PERIOD/100)
#define PWM_FREQ_200KHZ           0
#define PWM_FREQ_100KHZ           ID0
#define PWM_FREQ_50KHZ            ID1
#define PWM_FREQ_25KHZ            ID1 | ID0

#define TA0CCR1_DEFAULT           2
#define NumSamples                48
#define ADC12scale                68

#define REV_PG1_0          0
#define REV_PG2_0          1

/******************************************************************************
* DATA STRUCTURES
 *****************************************************************************/
typedef struct DRV8xx2_Device_Obj_t
{
  float Firmware_Version;          // Firmware version
  // Device Parameters
  volatile uint16_t deviceID;      // Device ID value
  volatile uint8_t revID;          // Silicon revision ID either from Register read or set value
  volatile uint8_t nSleep;         // 0: Device Sleep , 1: Device Awake
  volatile uint8_t gDRVEN;         // 0: Outputs Hi-Z, 1: Outputs active
  volatile uint8_t nFault;         // 0: No fault , 1: Fault
  volatile uint8_t ClrFlt;         // 0: Button not pressed, 1: Button pressed
  volatile uint8_t MODE;           // 0: H/W Mode, 1: SPI Mode
  volatile uint8_t StepMode;       // GUI Step Mode settings in Hardware mode
  volatile uint8_t DecayMode;      // GUI Decay Mode settings in Hardware mode
  volatile uint8_t TOFF;           // GUI TOFF settings in Hardware mode
  volatile uint8_t OCP_latched;    // GUI OCP setting in Hardware mode

  // Register Read Write
  volatile bool       ManWriteCmd;
  volatile bool       ManReadCmd;
  volatile uint16_t   ManWriteAddr;
  volatile uint16_t   ManReadAddr;
  volatile uint16_t   ManWriteData;
  volatile uint16_t   ManReadData;

  // VM supply voltage
  volatile uint16_t   VMvoltage;

  volatile uint8_t    AutoRead;    // 0: Auto Read off, 1: Auto Read on
} DRV8xx2_Device_Obj_t;            // Starting address 0x6200 total 29 bytes includes a filler byte after ManReadCmd for word alignment in MSP430

// Ramp duty controller
typedef struct DRV82x2_DRV89x2_RMPCNTL
{
  volatile uint8_t rampDelayMax;   // Parameter: Maximum delay rate (0-25)
  volatile uint8_t rampDelayCount; // Variable: Incremental delay (0-25)
  volatile uint8_t setpointValue;  // Output: Target output(0-100)
} RMPCNTL_t;

typedef struct DRV82x2_DRV89x2_Device_Obj_t
{
  // Device Parameters for brushed DRV82x2 and DRV89x2
  volatile uint8_t MODE1;          // 0: Dual H-Bridges, 1: Single H-Bridge
  volatile uint8_t MODE2;          // 0: Phase / Enable (PH/EN), 1: Pulse Width Modulation (PWM)
  volatile uint8_t Decay_Mode;     // 0: Slow Decay, 1: Smart Tune Dynamic Decay, and 2: Mixed Decay 30 % Fast
  volatile uint8_t TOFF;           // 0: 7 us,  1: 16 us, 2: 24 us, and 3: 32 us
  volatile uint8_t pwmFreq;        // 0: 25 kHz, 1: 50 kHz, 2: 100 kHz and 3: 200 kHz
  volatile uint8_t OCPM;           // 0: Latch and 1: Auto recover
  volatile uint8_t StartDrive1;
  volatile uint8_t StartDrive2;
  volatile uint8_t StartDrive3;    // for DRV8952/62
  volatile uint8_t StartDrive4;    // for DRV8952/62

  // Ramp controller
  RMPCNTL_t rampInput1;
  RMPCNTL_t rampInput2;
  RMPCNTL_t rampInput3;
  RMPCNTL_t rampInput4;

  // Control Parameters
  volatile uint8_t in1;            // in1 PWM duty cycle
  volatile uint8_t in2;            // in2 PWM duty cycle
  volatile uint8_t in3;            // in3 PWM duty cycle
  volatile uint8_t in4;            // in4 PWM duty cycle
  volatile uint8_t en1;            // enable in1 for DRV8952/62
  volatile uint8_t en2;            // enable in2 for DRV8952/62
  volatile uint8_t en3;            // enable in3 for DRV8952/62
  volatile uint8_t en4;            // enable in4 for DRV8952/62
  volatile uint8_t dir1;           // direction for Bridge 1
  volatile uint8_t pre_dir1;       // previous direction for Bridge 2
  volatile uint8_t dir2;           // direction for Bridge 1
  volatile uint8_t pre_dir2;       // previous direction for Bridge 2
  volatile uint8_t OutputRFtime;   // output rise and fall time setting for DRV8952/62
  volatile uint8_t dummy1;         // for a zero fill byte to align memory location to even address in MSP430

  // 32-bit Floating point variables
  volatile float VREF1;
  volatile float VREF2;
  volatile uint16_t ipropi1Adc;
  volatile uint16_t ipropi2Adc;
  volatile uint16_t ipropi3Adc;
  volatile uint16_t ipropi4Adc;
} DRV82x2_DRV89x2_Device_Obj_t;    // Starting address 0x6380 total 52 bytes

// Utility variables
volatile uint16_t Device_ID;
volatile unsigned char timerUpdate;
volatile unsigned char pwmState1;
volatile unsigned char pwmState2;
volatile unsigned char pwmState3;
volatile unsigned char pwmState4;
volatile float VREF1_previous;
volatile float VREF2_previous;
volatile unsigned int ipropi1;
volatile unsigned int ipropi2;
volatile unsigned int ipropi3;
volatile unsigned int ipropi4;
volatile int ADC12offset;
volatile unsigned char ADC12offsetpositive;
volatile unsigned char ADC12offsetnegative;

/******************************************************************************
 * EXTERN DECLARATIONS
 *****************************************************************************/
extern volatile DRV8xx2_Device_Obj_t gDRV8xx2_Obj;
extern volatile DRV8xx2_REG_t gDeviceSpiReg;
extern volatile DRV82x2_DRV89x2_Device_Obj_t gBrushed_Obj;
extern uint8_t DeviceGroup;

/******************************************************************************
 * API Prototypes
 *****************************************************************************/
void mcu_init(void);
void getEVMID(void);
void drv8xx2_init(void);
void drv84x2_RegistersRead();
void brushed_init(void);
void pwmTimerConfig(void);
void adc12_Init(void);
void adcTimerConfig(void);
void UpdateipropiReading(void);
void drv82x2_GUIcurrentdisplay(void);
void drv89x2_GUIcurrentdisplay(void);
void BrushedGPIOconfiguration(void);
void UpdateGUI_Register(void);
void UpdateDeviceControls_drv89x2_drv82x2(void);
void UpdateGUIControls_drv89x2_drv82x2(void);
void drv82x2_StartStopMotor(void);
void drv89x2_StartStopMotor(void);
void drv82x2_89x2_RampControl(void);
void drv82x2_89x2_TimerUpdate(void);
void drv_PH_EN_RampControl(void);
void drv_PWM_RampControl(void);
void drv_PWM_RampDown(void);
void drv89x2_PWM(void);
void drv82x2_PWM(void);
void drv_RampCTRL(volatile RMPCNTL_t* rmpCntl, uint8_t targetValue);

/*****************************************************************************/
#endif // DRV8xx2_H_
