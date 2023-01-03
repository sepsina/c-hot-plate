/*
 * global.h
 *
 *  Created on: 15. lis 2022.
 *      Author: Sep
 */

#ifndef SRC_GLOBAL_H_
#define SRC_GLOBAL_H_

#include "stdbool.h"

#define CMD_QUEUE_SIZE   8
#define CMD_PARAMS_LEN   64

#define SERIAL_TEST_PORT        0x0001
#define SERIAL_SET_THERMOSTAT   0x0002
#define SERIAL_GET_THERMOSTAT   0x0003
#define SERIAL_CHECK_DEVICE     0x0004

typedef enum {
	CMD_WAIT,
	CMD_RUN,
	CMD_DONE
} cmdState_t;

typedef union {
	struct {
		uint32_t i2c_1: 1;
		uint32_t r__1: 30;
	} b;
	uint32_t all;
} usedFlags_t;

#pragma pack(push, 1)

typedef struct {
    uint8_t used;
    uint8_t ttl;
    cmdState_t state;
    uint32_t ip;
    uint16_t port;
    uint16_t pktFunc;
	uint16_t cmdID;
	uint8_t cmd[CMD_PARAMS_LEN];
} cmdQueue_t;

typedef struct {
    uint32_t id;
} testPort_t;

typedef struct {
    uint16_t dummy;
} checkDevice_t;


#pragma pack(pop)

extern cmdQueue_t cmdQueue[];
extern usedFlags_t usedFlags;
extern bool spiFlag;
extern int16_t sleepTime;
extern int16_t tmoTime;

#endif /* SRC_GLOBAL_H_ */
