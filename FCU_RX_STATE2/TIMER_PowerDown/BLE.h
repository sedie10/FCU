#ifndef __BLE_H__
#define __BLE_H__

#include <stdint.h>

void sendDataToBle ( uint8_t *data, uint8_t size);
void uart2Bufferinit( void );
void BLEenable(void);
void BLEdisable(void);

#endif