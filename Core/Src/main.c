
/******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************/
#include <stddef.h>
#include "stm32_regs.h"
#include "stm32f1xx_ll_system.h"
#include "stdbool.h"
#include "main.h"
#include "hwInit.h"
#include "appTasks.h"
#include "app_printf.h"
#include "serial_handler.h"
#include "pwm.h"
#include "buttons.h"
#include "t_stat.h"
#include "global.h"

bool spiFlag = false;
int16_t sleepTime = -1;
int16_t tmoTime = 0;

static bool taskFlag;

static void appTask(void);
//static void rtcStart(uint16_t period);
static void toggleLED(void);

static void startThermostat(void);
static void serialCmd(uint16_t msgType);
static void initSysTick(void);

/****************************************************************************
 * main
 *
 * DESCRIPTION:
 *
 */
int main(void) {

    periphEn();
    bdEn(); // enable backup domain
    cfgSysClock();
    cfgLSE();

    //cfgRTC();
    cfgUSART();
    cfgLED();
    cfgFan();
    cfgPWM();
    cfgSPI();

    initAppTask();
    addAppTask(appTask, SLEEP_TIME);
    addAppTask(toggleLED, 500);
    addAppTask(startThermostat, 1000);

    initButtons();
    processCmd = serialCmd;

    NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
    //NVIC_SetPriority(RTC_Alarm_IRQn, 12);
    NVIC_SetPriority(SysTick_IRQn, 11);
    NVIC_SetPriority(DMA1_Channel2_IRQn, 10); // SPI_RX
    NVIC_SetPriority(EXTI15_10_IRQn, 9); // buttons

    LL_GPIO_AF_Remap_SWJ_NOJTAG();

    //SCB->SCR &= ~((uint32_t)1 << 2);
    LL_LPM_EnableSleep();

    __enable_irq();

    //NVIC_EnableIRQ(RTC_Alarm_IRQn);
    NVIC_EnableIRQ(DMA1_Channel2_IRQn); // SPI_RX
    NVIC_EnableIRQ(EXTI15_10_IRQn);     // buttons

    initSysTick();

    taskFlag = true;
    while(1) {
        if(taskFlag == true) {
            sleepTime = appTasksHandler();
            taskFlag = false;
            //rtcStart(sleepTime);
        }
        __WFI();
    }
}

/****************************************************************************
 * RTC_IRQHandler
 *
 * DESCRIPTION:
 *
 *
void RTC_Alarm_IRQHandler(void) {

    uint32_t exti;

    taskFlag = true;

    exti = (uint32_t)1 << 17;
    REG_Write(EXTI__PR, exti);
}
*/
/****************************************************************************
 * rtcStart
 *
 * DESCRIPTION:
 *
 *
static void rtcStart(uint16_t period) {

    rtc_reg_t rtc;

    rtc.cr_l.all = REG_Read(RTC__CRL);
    rtc.cr_l.b.cnf = 1; // 1: Enter configuration mode
    REG_Write(RTC__CRL, rtc.cr_l.all);

    REG_Write(RTC__ALRH, 0);
    REG_Write(RTC__ALRL, (period * 3));

    REG_Write(RTC__CNTH, 0);
    REG_Write(RTC__CNTL, 0);

    rtc.cr_l.b.cnf = 0; // 0: Exit configuration mode (start update of RTC registers).
    REG_Write(RTC__CRL, rtc.cr_l.all);
    do {
        rtc.cr_l.all = REG_Read(RTC__CRL);
    } while(rtc.cr_l.b.rtoff == 0); // 0: Last write operation on RTC registers is still ongoing
}
*/
/****************************************************************************
 * toggleLED
 *
 * DESCRIPTION:
 *
 */
static void toggleLED(void) {

    uint32_t led;
    static bool state = false;

    led = (uint32_t)1 << 13;
    if(state == true){
        REG_Write(GPIOC__BSRR, led);
    }
    else {
        REG_Write(GPIOC__BRR, led);
    }

    state = !state;

    addAppTask(toggleLED, 500);
}

/****************************************************************************
 * appTask
 *
 * DESCRIPTION:
 *
 */
static void appTask(void) {
    uartGetRx();
    addAppTask(appTask, SLEEP_TIME);
}

/****************************************************************************
 * Error_Handler
 *
 * DESCRIPTION:
 *
 */
void Error_Handler(void) {
    __disable_irq();
    while(1) {
    }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line) {
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
}
#endif /* USE_FULL_ASSERT */

/******************************************************************************
 * startThermostat
 *
 * DESCRIPTION: Processes the received message
 *
 * RETURNS:
 *
 */
static void startThermostat(void) {

    tsSet_t *set;

    set = (tsSet_t *)&rxMsg[0];
    set->runFlag = 1;
    set->setPoint = 27*4;
    set->hist = 1*4;
    set->duty = 10;

    setThermostat();
}

/******************************************************************************
 * serialCmd
 *
 * DESCRIPTION: Processes the received message
 *
 */
static void serialCmd(uint16_t msgType) {

    switch(msgType) {
        case SERIAL_TEST_PORT: {
            slWriteMsg(SERIAL_TEST_PORT,
                       sizeof(testPort_t),
                       &rxMsg[0]);
            break;
        }
        case SERIAL_SET_THERMOSTAT: {
            setThermostat();
            break;
        }
        case SERIAL_GET_THERMOSTAT: {
            getThermostat();
            break;
        }
        case SERIAL_CHECK_DEVICE: {
            checkDevice_t *rsp;

            rsp = (checkDevice_t *)&slTxBuf[0];
            rsp->dummy = 0xACDC;

            slWriteMsg(SERIAL_CHECK_DEVICE,
                       sizeof(checkDevice_t),
                       &slTxBuf[0]);
            break;
        }
    }
}

/****************************************************************************
 * initSysTick
 *
 * DESCRIPTION:
 *
 */
static void initSysTick(void) {

    systick_reg_t systick;

    systick.load.all = REG_Read(SYSTICK__LOAD);
    systick.load.b.reload = (9000 - 1);
    REG_Write(SYSTICK__LOAD, systick.load.all);

    REG_Write(SYSTICK__VAL, 0);

    systick.ctrl.all = REG_Read(SYSTICK__CTRL);
    systick.ctrl.b.clk_src = 0;  // 0: AHB/8
    systick.ctrl.b.tick_int = 1; // 1: Counting down to zero to asserts the SysTick exception request.
    systick.ctrl.b.enable = 1;   // 1: Counter enabled
    REG_Write(SYSTICK__CTRL, systick.ctrl.all);
}

/****************************************************************************
 * SysTick_Handler
 *
 * DESCRIPTION:
 *
 */
void SysTick_Handler(void) {

    if(sleepTime > 0){
        sleepTime--;
    }
    else {
        if(sleepTime == 0){
            taskFlag = true;
            sleepTime = -1;
        }
    }
    if(tmoTime > 0){
        tmoTime--;
    }
}

/****************************************************************************/
/****************************************************************************/



