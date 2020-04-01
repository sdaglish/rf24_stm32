#ifndef RF24_H
#define RF24_H

#include "nRF24L01.h"
#include "stm32f0xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/**
 * Power Amplifier level.
 *
 * For use with setPALevel()
 */
typedef enum {
  RF24_PA_MIN = 0,
  RF24_PA_LOW,
  RF24_PA_HIGH,
  RF24_PA_MAX,
  RF24_PA_ERROR
} rf24_pa_dbm_e;

/**
 * Data rate.  How fast data moves through the air.
 *
 * For use with setDataRate()
 */
typedef enum { RF24_1MBPS = 0, RF24_2MBPS, RF24_250KBPS } rf24_datarate_e;

bool rf_begin(void);
uint8_t write_register(uint8_t reg, uint8_t value);
void rf_setPALevel(uint8_t level);
void rf_openWritingPipe(const uint8_t *address);
void rf_openReadingPipe(uint8_t child, const uint8_t *address);
void rf_startListening(void);
bool rf_available(uint8_t *pipe_num);
uint8_t rf_read_payload(void *buf, uint8_t data_len);
void rf_read(void *buf, uint8_t len);
bool rf_write(uint8_t* buf, uint8_t len);
void rf_stopListening(void);
void rf_enableAckPayload(void);
void rf_enableDynamicPayloads(void);
void rf_writeAckPayload(uint8_t pipe, const uint8_t* buf, uint8_t len);

#endif
