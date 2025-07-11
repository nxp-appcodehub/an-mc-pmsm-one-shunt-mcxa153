/*
 * Copyright 2016-2025 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <m1_sm_snsless.h>
#include "pin_mux.h"
#include "peripherals.h"
#include "freemaster.h"
#include "freemaster_serial_lpuart.h"
#include "fsl_lpuart.h"
#include "fsl_lpadc.h"
#include "board.h"
#include "mc_periph_init.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* Version info */
#define MCRSP_VER "2.0.0" /* motor control package version */

/* Example's feature set in form of bits inside ui16featureSet.
   This feature set is expected to be growing over time.
   ... | FEATURE_S_RAMP | FEATURE_FIELD_WEAKENING | FEATURE_ENC
*/
#define FEATURE_ENC (1)               /* Encoder feature flag */
#define FEATURE_FIELD_WEAKENING (0)   /* Field weakening feature flag */
#define FEATURE_S_RAMP (0)            /* S-ramp feature flag */

#define FEATURE_SET (FEATURE_ENC << (0) | \
                     FEATURE_FIELD_WEAKENING << (1) | \
                     FEATURE_S_RAMP << (2))

/* Macro for correct Cortex CM0 / CM4 end of interrupt */
#define M1_END_OF_ISR \
    {                 \
        __DSB();      \
        __ISB();      \
    }

/* CPU load measurement SysTick START / STOP macros */
#define SYSTICK_START_COUNT() (SysTick->VAL = SysTick->LOAD)
#define SYSTICK_STOP_COUNT(par1)   \
    uint32_t val  = SysTick->VAL;  \
    uint32_t load = SysTick->LOAD; \
    par1          = load - val

static void BOARD_Init(void);
static void BOARD_InitSysTick(void);
static void BOARD_InitGPIO(void);
static void DemoSpeedStimulator(void);

static void init_freemaster_lpuart(void);

/*******************************************************************************
 * Variables
 ******************************************************************************/

volatile frac16_t f16SampleOne;
volatile frac16_t f16SampleTwo;

/* CPU load measurement using Systick */
uint32_t g_ui32NumberOfCycles    = 0U;
uint32_t g_ui32MaxNumberOfCycles = 0U;

uint32_t g_ui32NumberOfCycles_SLOW    = 0U;
uint32_t g_ui32MaxNumberOfCycles_SLOW = 0U;

GDFLIB_FILTER_MA_T_A32 sIDcOffsetFilter;

/* Demo mode enabled/disabled */
bool_t bDemoMode = FALSE;

uint32_t adc0_isr_cnt = 0;
uint32_t ctimer0_isr_cnt = 0;

/* Used for demo mode */
static uint32_t ui32SpeedStimulatorCnt    = 0U;

/* Counter for button pressing */
static uint32_t ui32ButtonFilter = 0U;

/* Structure used in FM to get required ID's */
app_ver_t g_sAppIdFM = {
    "",                       /* User Path 1- the highest priority */
    "../source",       /* User Path 2 */
    "frdmmcxa153",    /* board id */
    "pmsm_foc_1shunt", /* example id */
    MCRSP_VER,      /* sw version */
    FEATURE_SET,    /* example's feature-set */
};

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * @brief   Application main function processing peripheral function calling and
 *          infinite loop
 *
 * @param   void
 *
 * @return  none
 */
int main(void)
{
    uint32_t ui32PrimaskReg;

    /* Disable all interrupts before peripherals are initialized */
    ui32PrimaskReg = DisableGlobalIRQ();

    /* Disable demo mode after reset */
    bDemoMode              = FALSE;
    ui32SpeedStimulatorCnt = 0U;

    /*Accessing ID structure to prevent optimization*/
    g_sAppIdFM.ui16FeatureSet = FEATURE_SET;

    /* Init board hardware. */
    BOARD_Init();

    sIDcOffsetFilter.u16Sh = 4; // 16 points

    GDFLIB_FilterMAInit_F16(0, &sIDcOffsetFilter);

    /* Init MC peripherals */
    MCDRV_Init_M1();

    /* SysTick initialization for CPU load measurement */
    BOARD_InitSysTick();

    /* Turn off application */
    M1_SetAppSwitch(FALSE);

    /* FreeMASTER communication layer initialization */
    init_freemaster_lpuart();

    /* FreeMASTER driver initialization */
    FMSTR_Init();

    /* Enable interrupts  */
    EnableGlobalIRQ(ui32PrimaskReg);

    FLEXPWM0->SM[0].INTEN |= PWM_INTEN_CMPIE(1 << 1);

    /* Enable PWM0 interrupt. */
    NVIC_SetPriority(FLEXPWM0_SUBMODULE0_IRQn, 1U);//for ADC trigger enable
    NVIC_EnableIRQ(FLEXPWM0_SUBMODULE0_IRQn);

    /* Infinite loop */
    while (1)
    {
        /* FreeMASTER Polling function */
        FMSTR_Poll();
    }
}

//PWM0 compare interrupt for trigger enable
void FLEXPWM0_SUBMODULE0_IRQHandler()
{
    //enable ADC trigger here
    ADC_Trig_Attatch();

    if(FLEXPWM0->SM[0].STS & PWM_STS_CMPF(1<<1))
    {
    	FLEXPWM0->SM[0].STS |= PWM_STS_CMPF(1<<1);
    }
    //disable itself
    NVIC_DisableIRQ(FLEXPWM0_SUBMODULE0_IRQn);
}

/*!
 * @brief   Fast loop ISR
 *
 * @param   void
 *
 * @return  none
 */
FAST_FUNC_LIB
void ADC0_IRQHandler(void)
{
    /* Clear the TCOMP INT flag */
    ADC0->STAT |= (uint32_t)(1U << 9);

    GPIO1->PSOR |= 1U<<10U;

    adc0_isr_cnt++;

    /* Start CPU tick number couting */
    SYSTICK_START_COUNT();

    lpadc_conv_result_t s_ADC_ResultStructure;

    frac16_t f16IABCtemp[3];

    uint32_t tmp32;

    tmp32=ADC0->RESFIFO;

    while( (tmp32 & ADC_RESFIFO_VALID_MASK)>0 )
    {
      s_ADC_ResultStructure.commandIdSource = (tmp32 & ADC_RESFIFO_CMDSRC_MASK) >> ADC_RESFIFO_CMDSRC_SHIFT;
      s_ADC_ResultStructure.convValue       = (uint16_t)(tmp32 & ADC_RESFIFO_D_MASK);

      tmp32 = ADC0->RESFIFO;

      switch( s_ADC_ResultStructure.commandIdSource )
      {
      case 1:
      /* Command 1 */
    	f16SampleOne =(frac16_t)(s_ADC_ResultStructure.convValue);
    	break;

      case 2:
      /* Command 2 */
    	f16SampleTwo =(frac16_t)(s_ADC_ResultStructure.convValue);
    	break;

      case 3:
      /* Command 3 */
    	g_sM1Drive.sFocPMSM.f16IDcOffset =(frac16_t)(s_ADC_ResultStructure.convValue);
    	break;

      case 4:
      /* Command 4 */
    	g_sM1Drive.sFocPMSM.f16UDcBus =(frac16_t)(s_ADC_ResultStructure.convValue);
    	break;

      default:
        break;
      }
    }

    // get the filtered Idc -----------------------------------
    g_sM1Drive.sFocPMSM.f16IDcOffsetFilt = GDFLIB_FilterMA_F16(g_sM1Drive.sFocPMSM.f16IDcOffset, &sIDcOffsetFilter);

    // get the filtered Udc -----------------------------------
    g_sM1Drive.sFocPMSM.f16UDcBusFilt= GDFLIB_FilterIIR1_F16(g_sM1Drive.sFocPMSM.f16UDcBus, &g_sM1Drive.sFocPMSM.sUDcBusFilter);

      //------------------------------ get the reconstructed Iabc -----------------------------
    f16IABCtemp[g_sM1Drive.sFocPMSM.sADCConfig.eSmplOnePh] = MLIB_SubSat_F16(f16SampleOne, g_sM1Drive.sFocPMSM.f16IDcOffsetFilt);
  	f16IABCtemp[g_sM1Drive.sFocPMSM.sADCConfig.eSmplTwoPh] = MLIB_SubSat_F16(g_sM1Drive.sFocPMSM.f16IDcOffsetFilt, f16SampleTwo);
  	f16IABCtemp[g_sM1Drive.sFocPMSM.sADCConfig.eCalcPh] =
  			MLIB_Neg_F16(MLIB_AddSat_F16(f16IABCtemp[g_sM1Drive.sFocPMSM.sADCConfig.eSmplOnePh],f16IABCtemp[g_sM1Drive.sFocPMSM.sADCConfig.eSmplTwoPh]));

  	g_sM1Drive.sFocPMSM.sIABC.f16A = f16IABCtemp[kPhaseA];
  	g_sM1Drive.sFocPMSM.sIABC.f16B = f16IABCtemp[kPhaseB];
  	g_sM1Drive.sFocPMSM.sIABC.f16C = f16IABCtemp[kPhaseC];

    /* State machine */
    SM_StateMachineFast(&g_sM1Ctrl);

    /* stop CPU tick number couting and store actual and maximum ticks */
    SYSTICK_STOP_COUNT(g_ui32NumberOfCycles);
    g_ui32MaxNumberOfCycles =
        g_ui32NumberOfCycles > g_ui32MaxNumberOfCycles ? g_ui32NumberOfCycles : g_ui32MaxNumberOfCycles;

    /* Call FreeMASTER recorder */
    FMSTR_Recorder(0);

    GPIO1->PCOR |= 1U<<10U;

    /* Add empty instructions for correct interrupt flag clearing */
    M1_END_OF_ISR;

}

/*!
 * @brief   Slow loop interrupt handler (1ms period)
 *           - motor M1 slow application machine function
 *
 * @param   void
 *
 * @return  none
 */
FAST_FUNC_LIB
void CTIMER0_IRQHandler(void)
{
    /* Clear the match interrupt flag. */
    CTIMER0->IR |= CTIMER_IR_MR0INT(1U);

    GPIO1->PSOR |= 1U<<12U;

    ctimer0_isr_cnt++;

    if(0 == (ctimer0_isr_cnt & 0xfff))
    {
    	g_ui32MaxNumberOfCycles = 0;
    	g_ui32MaxNumberOfCycles_SLOW = 0;
    }

    static int16_t ui16i = 0;

    /* Start CPU tick number couting */
    SYSTICK_START_COUNT();

    /* M1 Slow StateMachine call */
    SM_StateMachineSlow(&g_sM1Ctrl);

    /* stop CPU tick number couting and store actual and maximum ticks */
    SYSTICK_STOP_COUNT(g_ui32NumberOfCycles_SLOW);
    g_ui32MaxNumberOfCycles_SLOW =
        g_ui32NumberOfCycles_SLOW > g_ui32MaxNumberOfCycles_SLOW ? g_ui32NumberOfCycles_SLOW : g_ui32MaxNumberOfCycles_SLOW;

    /* If in STOP state turn on RED */
    if (M1_GetAppState() == 2U)
    {
        LED_RED_ON();
        LED_GREEN_OFF();
    }

    /* If in FAULT state RED blinking*/
    else if (M1_GetAppState() == 0U)
    {
        if (ui16i-- < 0)
        {
            LED_RED_TOGGLE();
            bDemoMode = FALSE;
            ui16i = 125;
        }
        LED_GREEN_OFF();
    }

    /* If in RUN or INIT state turn on green */
    else
    {
        LED_RED_OFF();
        LED_GREEN_ON();
    }

    /* Demo speed stimulator */
    DemoSpeedStimulator();

    GPIO1->PCOR |= 1U<<12U;

    /* Add empty instructions for correct interrupt flag clearing */
    M1_END_OF_ISR;
}

/*!
 * @brief   DemoSpeedStimulator
 *           - When demo mode is enabled it changes the required speed according
 *             to predefined profile
 *
 * @param   void
 *
 * @return  none
 */
FAST_FUNC_LIB
static void DemoSpeedStimulator(void)
{
    /* Increment push button pressing counter  */
    if (ui32ButtonFilter < 1000)
        ui32ButtonFilter++;

    if (bDemoMode)
    {
        ui32SpeedStimulatorCnt++;
        switch (ui32SpeedStimulatorCnt)
        {
            case 100:
                M1_SetSpeed(FRAC16(1000.0 / M1_N_MAX));
                break;
            case 3000:
                M1_SetSpeed(FRAC16(2000.0 / M1_N_MAX));
                break;
            case 6000:
                M1_SetSpeed(FRAC16(4000.0 / M1_N_MAX));
                break;
            case 9000:
                M1_SetSpeed(FRAC16(900.0 / M1_N_MAX));
                break;
            case 12000:
                M1_SetSpeed(FRAC16(4000.0 / M1_N_MAX));
                break;
            case 15000:
                M1_SetSpeed(FRAC16(2000.0 / M1_N_MAX));
                break;
            case 18000:
                M1_SetSpeed(FRAC16(800.0 / M1_N_MAX));
                ui32SpeedStimulatorCnt = 0;
                break;
            default:
                break;
        }
    }
}


/*!
 * @brief   static void BOARD_Init(void)
 *           - Initialization of clocks, pins and GPIO
 *
 * @param   void
 *
 * @return  none
 */
static void BOARD_Init(void)
{
    /* attach FRO 12M to LPUART0 (debug console) */
    RESET_PeripheralReset(kLPUART0_RST_SHIFT_RSTn);
    RESET_PeripheralReset(BOARD_DEBUG_UART_RST);
    RESET_PeripheralReset(kPORT0_RST_SHIFT_RSTn);
    RESET_PeripheralReset(kPORT2_RST_SHIFT_RSTn);
    RESET_PeripheralReset(kPORT3_RST_SHIFT_RSTn);
    RESET_PeripheralReset(kADC0_RST_SHIFT_RSTn);
    RESET_PeripheralReset(kFLEXPWM0_RST_SHIFT_RSTn);
    RESET_PeripheralReset(kINPUTMUX0_RST_SHIFT_RSTn);
    RESET_PeripheralReset(kCTIMER0_RST_SHIFT_RSTn);

    /* attach FRO 12M to LPUART0 (debug console) */
    CLOCK_SetClockDiv(kCLOCK_DivLPUART0, 1u);
    CLOCK_AttachClk(BOARD_DEBUG_UART_CLK_ATTACH);

    /* write to INPUTMUX0: Peripheral clock is enabled */
    CLOCK_EnableClock(kCLOCK_GateINPUTMUX0);

    CLOCK_EnableClock(kCLOCK_GateCMP0);
    CLOCK_EnableClock(kCLOCK_GateCMP1);

    CLOCK_AttachClk(kFRO_HF_to_CTIMER0);

    /* attach peripheral clock */
    CLOCK_AttachClk(kFRO12M_to_CMP0);
    CLOCK_SetClockDiv(kCLOCK_DivCMP0_FUNC, 1U);

    /* enable CMP0 and CMP0_DAC */
    SPC0->ACTIVE_CFG1 |= ((1U << 16U) | (1U << 20U));

    /* Init pins set in pin_mux file */
    BOARD_InitBootPins();

    /* Initialize clock configuration */
    BOARD_InitBootClocks();

    /* Init peripherals set in peripherals file */
    BOARD_InitBootPeripherals();

    /* Init GPIO pins */
    BOARD_InitGPIO();

}


/*!
 * @brief   Port interrupt handler
 *
 * @param   void
 *
 * @return  none
 */

void BOARD_SW2_IRQ_HANDLER(void) {

    /* Proceed only if pressing longer than timeout */
    if (ui32ButtonFilter > 200)
    {
        ui32ButtonFilter = 0;
        if (bDemoMode)
        {
            M1_SetSpeed(0);
            M1_SetAppSwitch(FALSE);
            bDemoMode = FALSE;
        }
        else
        {
            M1_SetAppSwitch(TRUE);
            bDemoMode = TRUE;
            ui32SpeedStimulatorCnt = 0;
        }
    }

    /* Clear external interrupt flag. */
    GPIO_GpioClearInterruptFlags(BOARD_SW2_GPIO, 1U << BOARD_SW2_GPIO_PIN);

    /* Add empty instructions for correct interrupt flag clearing */
    M1_END_OF_ISR;

}


/*!
 *@brief      Initialization of the GPIO pins
 *
 *@param      none
 *
 *@return     none
 */
static void BOARD_InitGPIO(void)
{
    /* Define the init structure for the input switch pin */
    gpio_pin_config_t sw_config = {
        kGPIO_DigitalInput,
        0,
    };

    /* Init output LED GPIO */
    LED_RED_INIT(LOGIC_LED_OFF);
    LED_GREEN_INIT(LOGIC_LED_OFF);
    LED_BLUE_INIT(LOGIC_LED_OFF);

    /* Init input switch GPIO. */
    GPIO_SetPinInterruptConfig(BOARD_SW2_GPIO, BOARD_SW2_GPIO_PIN, kGPIO_InterruptRisingEdge);
    GPIO_PinInit(BOARD_SW2_GPIO, BOARD_SW2_GPIO_PIN, &sw_config);
    EnableIRQ(BOARD_SW2_IRQ);

}


/*!
 *@brief      SysTick initialization for CPU cycle measurement
 *
 *@param      none
 *
 *@return     none
 */
static void BOARD_InitSysTick(void)
{
    /* Initialize SysTick core timer to run free */
    /* Set period to maximum value 2^24*/
    SysTick->LOAD = 0xFFFFFF;

    /*Clock source - System Clock*/
    SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk;

    /*Start Sys Timer*/
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
}

/*!
 * @brief LPUART Module initialization (LPUART is a the standard block included e.g. in K66F)
 */
static void init_freemaster_lpuart(void)
{
    lpuart_config_t config;

    /*
     * config.baudRate_Bps = 115200U;
     * config.parityMode = kUART_ParityDisabled;
     * config.stopBitCount = kUART_OneStopBit;
     * config.txFifoWatermark = 0;
     * config.rxFifoWatermark = 1;
     * config.enableTx = false;
     * config.enableRx = false;
     */
    LPUART_GetDefaultConfig(&config);
    config.baudRate_Bps = 115200U;
    config.enableTx     = false;
    config.enableRx     = false;

    LPUART_Init((LPUART_Type *)BOARD_DEBUG_UART_BASEADDR, &config, BOARD_DEBUG_UART_CLK_FREQ);

    /* Register communication module used by FreeMASTER driver. */
    FMSTR_SerialSetBaseAddress((LPUART_Type *)BOARD_DEBUG_UART_BASEADDR);

}
