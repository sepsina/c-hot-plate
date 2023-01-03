/*
 * hwInit.c
 *
 *  Created on: Sep 2, 2022
 *      Author: Serfa
 */

#include "hwInit.h"
#include "stm32_regs.h"
#include "stm32f1xx_ll_system.h"
#include "serial_handler.h"

/****************************************************************************
 * periphEn
 *
 * DESCRIPTION:
 *
 */
void bdEn(void) {

    pwr_reg_t pwr;

    pwr.cr.all = REG_Read(PWR__CR);
    pwr.cr.b.dbp = 1; // 1: Access to RTC and Backup registers enabled
    REG_Write(PWR__CR, pwr.cr.all);
}


/****************************************************************************
 * periphEn
 *
 * DESCRIPTION:
 *
 */
void periphEn(void) {

    rcc_reg_t rcc;

    rcc.ahb_en.all = REG_Read(RCC__AHBENR);
    rcc.ahb_en.b.dma1_en = 1; // 1: DMA1 clock enabled
    REG_Write(RCC__AHBENR, rcc.ahb_en.all);

    rcc.apb1_en.all = REG_Read(RCC__APB1ENR);
    rcc.apb1_en.b.pwr_en = 1;    // 1: Power interface clock enable
    rcc.apb1_en.b.bkp_en = 1;    // 1: Backup interface clock enabled
    rcc.apb1_en.b.tim2_en = 1;   // 1: Timer 2 clock enabled
    rcc.apb1_en.b.usart2_en = 1; // 1: USART2 clock enabled
    REG_Write(RCC__APB1ENR, rcc.apb1_en.all);

    rcc.apb2_en.all = REG_Read(RCC__APB2ENR);
    rcc.apb2_en.b.af_io_en = 1;  // 1: Alternate Function I/O clock enabled
    rcc.apb2_en.b.io_pa_en = 1;  // 1: I/O port A clock enabled
    rcc.apb2_en.b.io_pb_en = 1;  // 1: I/O port B clock enabled
    rcc.apb2_en.b.io_pc_en = 1;  // 1: I/O port C clock enabled
    rcc.apb2_en.b.spi1_en = 1;   // 1: SPI 1 clock enabled
    REG_Write(RCC__APB2ENR, rcc.apb2_en.all);

}

/****************************************************************************
 * cfgSysClock
 *
 * DESCRIPTION:
 *
 */
void cfgSysClock(void) {

    rcc_reg_t rcc;
    //pwr_reg_t pwr;

    LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
    while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_2);

    rcc.cr.all = REG_Read(RCC__CR);
    rcc.cr.b.hse_on = 1; // HSE oscillator ON
    REG_Write(RCC__CR, rcc.cr.all);
    do {
        rcc.cr.all = REG_Read(RCC__CR);
    } while(rcc.cr.b.hse_on == 0); // Wait till HSE is ready

    rcc.cfg.all = REG_Read(RCC__CFGR);
    rcc.cfg.b.pll_src = 1;   // Clock from PREDIV1 selected as PLL input clock
    rcc.cfg.b.pll_xtpre = 0; // no divide by 2
    rcc.cfg.b.pll_mul = 7;   // 0111: PLL input clock x 9
    REG_Write(RCC__CFGR, rcc.cfg.all);

    rcc.cr.all = REG_Read(RCC__CR);
    rcc.cr.b.pll_on = 1; // PLL ON
    REG_Write(RCC__CR, rcc.cr.all);
    do {
        rcc.cr.all = REG_Read(RCC__CR);
    } while(rcc.cr.b.pll_on == 0); // Wait till PLL is ready

    rcc.cfg.all = REG_Read(RCC__CFGR);
    rcc.cfg.b.ahb_pre = 0;  // 0xxx: SYSCLK not divided
    rcc.cfg.b.apb1_pre = 4; // 100: HCLK divided by 2
    rcc.cfg.b.apb2_pre = 0; // 0xx: HCLK not divided
    rcc.cfg.b.clk_sw = 2;   // 10: PLL selected as system clock
    REG_Write(RCC__CFGR, rcc.cfg.all);
    do {
        rcc.cfg.all = REG_Read(RCC__CFGR);
    } while(rcc.cfg.b.clk_sw_sts != 2); // 10: PLL selected as system clock

}

/****************************************************************************
 * cfgLSE
 *
 * DESCRIPTION:
 *
 */
void cfgLSE(void) {

    rcc_reg_t rcc;

    rcc.bd_cr.all = REG_Read(RCC__BDCR);
    rcc.bd_cr.b.lse_on = 1; // 1: External 32 kHz oscillator ON
    REG_Write(RCC__BDCR, rcc.bd_cr.all);
    do {
        rcc.bd_cr.all = REG_Read(RCC__BDCR);
    } while(rcc.bd_cr.b.lse_rdy == 0); // 0: External 32 kHz oscillator not ready

}

/****************************************************************************
 * cfgLED
 *
 * DESCRIPTION:
 *
 */
void cfgLED(void) {

    gpio_reg_t gpio;
    uint32_t led;

    // PC13   ------> ON BOARD LED
    gpio.crh.all = REG_Read(GPIOC__CRH);
    gpio.crh.b.cnf_13   = 1; // 01: General purpose output Open-drain
    gpio.crh.b.mode_13  = 2; // 10: Output mode, max speed 2 MHz
    REG_Write(GPIOC__CRH, gpio.crh.all);

    led = (uint32_t)1 << 13;
    REG_Write(GPIOC__BSRR, led);

}

/****************************************************************************
 * cfgFan
 *
 * DESCRIPTION:
 *
 */
void cfgFan(void) {

    gpio_reg_t gpio;
    uint32_t fan;

    // PB10 --> Fan

    gpio.crh.all = REG_Read(GPIOB__CRH);
    gpio.crh.b.cnf_10   = 0; // 00: General purpose output push-pull
    gpio.crh.b.mode_10  = 2; // 10: Output mode, max speed 2 MHz
    REG_Write(GPIOB__CRH, gpio.crh.all);

    fan = (uint32_t)1 << 10;
    REG_Write(GPIOB__BSRR, fan);

}

/****************************************************************************
 * cfgRTC
 *
 * DESCRIPTION:
 *
 */
void cfgRTC(void) {

    rcc_reg_t rcc;
    //rtc_reg_t rtc;
    rtc_crl_t crl;
    uint32_t exti;

    rcc.bd_cr.all = REG_Read(RCC__BDCR);
    rcc.bd_cr.b.rtc_sel = 1; // 01: LSE oscillator clock used as RTC clock (32768 Hz)
    rcc.bd_cr.b.rtc_en = 1;  // 0: RTC clock disabled
    REG_Write(RCC__BDCR, rcc.bd_cr.all);

    crl.all = REG_Read(RTC__CRL);
    crl.b.rsf = 0;
    REG_Write(RTC__CRL, crl.all);
    do {
        crl.all = REG_Read(RTC__CRL);
    } while(crl.b.rsf == 0); // 0: Registers not yet synchronized

    exti = REG_Read(EXTI__IMR);
    exti |= (uint32_t)1 << 17;
    REG_Write(EXTI__IMR, exti);
    exti = REG_Read(EXTI__RTSR);
    exti |= (uint32_t)1 << 17;
    REG_Write(EXTI__RTSR, exti);

    /*
    rtc.cr_h.all = REG_Read(RTC__CRH);
    rtc.cr_h.b.alr_ie = 1;
    REG_Write(RTC__CRH, rtc.cr_h.all);
    */
    crl.b.cnf = 1; // 1: Enter configuration mode
    REG_Write(RTC__CRL, crl.all);

    REG_Write(RTC__PRLH, 0);
    REG_Write(RTC__PRLL, 10); // RTC pre-scaler reload value low -> fTR_CLK = fRTCCLK/(PRL[19:0]+1) = 32768/11

    REG_Write(RTC__CNTH, 0);
    REG_Write(RTC__CNTL, 0);

    REG_Write(RTC__ALRH, 0);
    REG_Write(RTC__ALRL, 65535);

    crl.b.cnf = 1; // 0: Exit configuration mode (start update of RTC registers)
    REG_Write(RTC__CRL, crl.all);
    do {
        crl.all = REG_Read(RTC__CRL);
    } while(crl.b.rtoff == 0); // 0: Last write operation on RTC registers is still ongoing
}

/****************************************************************************
 * cfgUSART
 *
 * DESCRIPTION:
 *
 */
void cfgUSART(void) {

    gpio_reg_t gpio;
    dma_reg_t dma;
    usart_reg_t usart;

    // USART2 GPIO Configuration
    // PA2 ------> USART2_TX
    // PA3 ------> USART2_RX
    gpio.crl.all = REG_Read(GPIOA__CRL);
    // TxD
    gpio.crl.b.cnf_2  = 2; // 10: Alternate function output Push-pull
    gpio.crl.b.mode_2 = 3; // 11: Output mode, max speed 50 MHz
    // RxD
    gpio.crl.b.cnf_3  = 1; // 01: Floating input (reset state)
    gpio.crl.b.mode_3 = 0; // 00: Input mode (reset state)
    REG_Write(GPIOA__CRL, gpio.crl.all);

    // USART RX
    dma.mem_addr.b.ma = (uint32_t)&rxBuf.data[0];
    REG_Write(DMA__CMAR6, dma.mem_addr.all);

    dma.per_addr.b.pa = USART2__DR;
    REG_Write(DMA__CPAR6, dma.per_addr.all);

    dma.data_len.b.ndt = RX_BUF_SIZE;
    REG_Write(DMA__CNDTR6, dma.data_len.all);

    dma.ch_cfg.all = REG_Read(DMA__CCR6);
    dma.ch_cfg.b.dir = 0;    // 0: Read from peripheral
    dma.ch_cfg.b.pl = 2;     // 10: high priority
    dma.ch_cfg.b.circ = 1;   // 1: Circular mode enabled
    dma.ch_cfg.b.pinc = 0;   // 0: Peripheral increment mode disabled
    dma.ch_cfg.b.minc = 1;   // 1: Memory increment mode enabled
    dma.ch_cfg.b.psize = 0;  // 00: 8-bits
    dma.ch_cfg.b.msize = 0;  // 00: 8-bits
    dma.ch_cfg.b.en = 1;     // 1: Channel enabled
    REG_Write(DMA__CCR6, dma.ch_cfg.all);

    // USART TX
    dma.ch_cfg.all = REG_Read(DMA__CCR7);
    dma.ch_cfg.b.dir = 1;    // 1: Read from memory
    dma.ch_cfg.b.pl = 0;     // 00: low priority
    dma.ch_cfg.b.circ = 0;   // 0: Circular mode disabled
    dma.ch_cfg.b.pinc = 0;   // 0: Peripheral increment mode disabled
    dma.ch_cfg.b.minc = 1;   // 1: Memory increment mode enabled
    dma.ch_cfg.b.psize = 0;  // 00: 8-bits
    dma.ch_cfg.b.msize = 0;  // 00: 8-bits
    REG_Write(DMA__CCR7, dma.ch_cfg.all);

    //baud rate-> 115200
    usart.brr.all = REG_Read(USART2__BRR);
    usart.brr.b.mantissa = 19;
    usart.brr.b.fraction = 8;
    REG_Write(USART2__BRR, usart.brr.all);

    usart.cr2.all = REG_Read(USART2__CR2);
    usart.cr2.b.stop = 0; // 00: 1 Stop bit
    REG_Write(USART2__CR2, usart.cr2.all);

    usart.cr3.all = REG_Read(USART2__CR3);
    usart.cr3.b.dmar = 1; // 1: DMA mode is enabled for reception
    usart.cr3.b.dmat = 1; // 1: DMA mode is enabled for transmission
    REG_Write(USART2__CR3, usart.cr3.all);

    usart.cr1.all = REG_Read(USART2__CR1);
    usart.cr1.b.m = 0;     // 0: 1 Start bit, 8 Data bits, n Stop bit
    usart.cr1.b.pce = 0;   // 0: Parity control disabled
    usart.cr1.b.rx_en = 1; // 1: Receiver is enabled and begins searching for a start bit
    usart.cr1.b.tx_en = 1; // 1: Transmitter is enabled
    usart.cr1.b.en = 1;    // 1: USART enabled
    REG_Write(USART2__CR1, usart.cr1.all);

}

/****************************************************************************
 * cfgPWM
 *
 * DESCRIPTION:
 *
 */
void cfgPWM(void) {

    gpio_reg_t gpio;
    uint32_t b_set;
    tim_reg_t tim;

    // PWM GPIO Configuration
    // PA0  ------> PWM
    gpio.crl.all = REG_Read(GPIOA__CRL);
    gpio.crl.b.cnf_0  = 2; // 10: Alternate function output Push-pull
    gpio.crl.b.mode_0 = 3; // 11: Output mode, max speed 50 MHz
    REG_Write(GPIOA__CRL, gpio.crl.all);

    b_set = (uint32_t)1 << 0;
    REG_Write(GPIOA__BRR, b_set); // pull down

    tim.cr1.all = REG_Read(TIM2__CR1);
    tim.cr1.b.cdk = 0;  // 00: tDTS = tCK_INT
    tim.cr1.b.arpe = 1; // 1: TIMx_ARR register is buffered
    tim.cr1.b.cms = 0;  // 00: Edge-aligned mode
    tim.cr1.b.dir = 0;  // 0: Counter used as upcounter
    REG_Write(TIM2__CR1, tim.cr1.all);

    tim.ccmr1.all = REG_Read(TIM2__CCMR1);
    tim.ccmr1.b.cc1s = 0;  // 00: CC1 channel is configured as output.
    tim.ccmr1.b.oc1m = 6;  // 110: PWM mode 1 - In upcounting, active when TIMx_CNT<TIMx_CCR1
    tim.ccmr1.b.oc1pe = 1; // 1: Preload register on TIMx_CCR1 enabled
    REG_Write(TIM2__CCMR1, tim.ccmr1.all);

    tim.psc.all = REG_Read(TIM2__PSC);
    tim.psc.b.psc = (7200 - 1); // 72MHz/7200 -> 10kHz counter clock
    REG_Write(TIM2__PSC, tim.psc.all);

    tim.arr.all = REG_Read(TIM2__ARR);
    tim.arr.b.arr = (10000 - 1); // 10kHz/10000 -> 1Hz PWM frequency
    REG_Write(TIM2__ARR, tim.arr.all);

    tim.ccr.all = REG_Read(TIM2__CCR1);
    tim.ccr.b.ccr = (0 * 100); // compare value -> 0% duty cycle
    REG_Write(TIM2__CCR1, tim.ccr.all);

    tim.ccer.all = REG_Read(TIM2__CCER);
    tim.ccer.b.cc1p = 0;  // 0: OC1 active high.
    tim.ccer. b.cc1e = 1; // 1: On - OC1 signal is output on the corresponding output pin.
    REG_Write(TIM2__CCER, tim.ccer.all);

    tim.cr1.all = REG_Read(TIM2__CR1);
    tim.cr1.b.cen = 1; // 1: Counter enabled
    REG_Write(TIM2__CR1, tim.cr1.all);

    tim.egr.all = REG_Read(TIM2__EGR);
    tim.egr.b.ug = 1; // 1: Re-initialize the counter and generates an update of the registers
    REG_Write(TIM2__CR1, tim.egr.all);

}

/****************************************************************************
 * cfgSPI
 *
 * DESCRIPTION:
 *
 */
void cfgSPI(void) {

    gpio_reg_t gpio;
    uint32_t b_set;
    spi_reg_t spi;
    dma_reg_t dma;

    // PB5 -> CSS
    gpio.crl.all = REG_Read(GPIOB__CRL);
    gpio.crl.b.cnf_5  = 0; // 00: General purpose output push-pull
    gpio.crl.b.mode_5 = 2; // 10: Output mode, max speed 2 MHz
    REG_Write(GPIOB__CRL, gpio.crl.all);

    REG_Write(GPIOB__BSRR, (uint32_t)1 << 5); // PB5 -> 1

    // SPI1 GPIO Configuration
    // NSS  ---> PA4
    // SCK  ---> PA5
    // MISO ---> PA6
    // MOSI ---> PA7
    gpio.crl.all = REG_Read(GPIOA__CRL);
    // SS
    gpio.crl.b.cnf_4  = 2; // 10: Input with pull-up / pull-down
    gpio.crl.b.mode_4 = 0; // 00: Input mode
    // SCK
    gpio.crl.b.cnf_5  = 2; // 10: Alternate function output Push-pull
    gpio.crl.b.mode_5 = 3; // 11: Output mode, max speed 50 MHz.
    // MISO
    gpio.crl.b.cnf_6  = 2; // 10: Input with pull-up / pull-down
    gpio.crl.b.mode_6 = 0; // 00: Input mode
    // MOSI
    gpio.crl.b.cnf_7  = 2; // 10: Alternate function output Push-pull
    gpio.crl.b.mode_7 = 2; // 10: Output mode, max speed 2 MHz.
    REG_Write(GPIOA__CRL, gpio.crl.all);

    b_set  = (uint32_t)1 << 6;
    b_set |= (uint32_t)1 << 4;
    REG_Write(GPIOA__BSRR, b_set); // pull up

    spi.cr1.all = REG_Read(SPI1__CR1);
    spi.cr1.b.dff = 0;       // 0: 8-bit data frame format is selected for transmission/reception
    spi.cr1.b.rx_only = 0;   // 0: Full duplex (Transmit and receive)
    spi.cr1.b.ssm = 0;       // 0: Software slave management disabled
    spi.cr1.b.lsb_first = 0; // 0: MSB transmitted first
    spi.cr1.b.br = 5;        // 101: fPCLK/64
    spi.cr1.b.mstr = 1;      // 1: Master configuration
    spi.cr1.b.clk_pol = 0;   // 0: CK to 0 when idle
	spi.cr1.b.clk_pha = 0;   // 0: The first clock transition is the first data capture edge
    //spi.cr1.b.clk_pol = 1;   // 1: CK to 1 when idle
    //spi.cr1.b.clk_pha = 1;   // 1: The second clock transition is the first data capture edge
    REG_Write(SPI1__CR1, spi.cr1.all);

    spi.cr2.all = REG_Read(SPI1__CR2);
    spi.cr2.b.ss_oe = 0;       // 0: SS output is disabled in master mode and the cell can work in multimaster configuration
    spi.cr2.b.rx_dma_en = 1;   // 1: Rx buffer DMA enabled
    spi.cr2.b.tx_dma_en = 1;   // 1: Tx buffer DMA enabled
    REG_Write(SPI1__CR2, spi.cr2.all);

    // DMA_CH2 -> SPI_RX
    dma.per_addr.b.pa = SPI1__DR;
    REG_Write(DMA__CPAR2, dma.per_addr.all);

    dma.ch_cfg.all = REG_Read(DMA__CCR2);
    dma.ch_cfg.b.dir = 0;    // 0: Read from peripheral
    dma.ch_cfg.b.pl = 2;     // 10: high priority
    dma.ch_cfg.b.circ = 0;   // 0: Circular mode disabled
    dma.ch_cfg.b.pinc = 0;   // 0: Peripheral increment mode disabled
    dma.ch_cfg.b.minc = 1;   // 1: Memory increment mode enabled
    dma.ch_cfg.b.psize = 0;  // 00: 8-bits
    dma.ch_cfg.b.msize = 0;  // 00: 8-bits
    dma.ch_cfg.b.tcie = 1;   // 1: TC interrupt enabled
    REG_Write(DMA__CCR2, dma.ch_cfg.all);

    // DMA_CH_3 -> SPI_TX
    dma.per_addr.b.pa = SPI1__DR;
    REG_Write(DMA__CPAR3, dma.per_addr.all);

    dma.ch_cfg.all = REG_Read(DMA__CCR3);
    dma.ch_cfg.b.dir = 1;    // 1: Read from memory
    dma.ch_cfg.b.pl = 0;     // 10: high priority
    dma.ch_cfg.b.circ = 0;   // 0: Circular mode disabled
    dma.ch_cfg.b.pinc = 0;   // 0: Peripheral increment mode disabled
    dma.ch_cfg.b.minc = 1;   // 1: Memory increment mode enabled
    dma.ch_cfg.b.psize = 0;  // 00: 8-bits
    dma.ch_cfg.b.msize = 0;  // 00: 8-bits
    REG_Write(DMA__CCR3, dma.ch_cfg.all);

}


