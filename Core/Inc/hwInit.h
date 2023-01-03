/*
 * hwInit.h
 *
 *  Created on: Sep 2, 2022
 *      Author: Serfa
 */

#ifndef HW_INIT_H_
#define HW_INIT_H_


extern void cfgSysClock(void);
extern void cfgLSE(void);
extern void bdEn(void);
extern void periphEn(void);
extern void cfgLED(void);
extern void cfgFan(void);
extern void cfgRTC(void);
extern void cfgUSART(void);
extern void cfgI2C(void);
extern void cfgPWM(void);
extern void cfgSPI(void);
extern void cfgADC(void);

#endif /* HW_INIT_H_ */
