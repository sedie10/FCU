#ifndef _BLE_H
#define _BLE_H

#include "Targetdef.h"
#include "NuMicro.h"


#if defined( BLE_HANDSHAKING )

extern void sendResponseToBle ( uint8_t type );
extern void sendDataToBle ( uint8_t *data, uint8_t size);
extern void sendNotavailableToBle( void );
extern void sendIdentificationToBle( void );
extern void sendConfigurationToBle( void );
extern uint8_t getString_UART0( void );
extern uint8_t getStatus_UART0(void);
extern void UART0_Proc( uint8_t data );
extern void sendMacAddToBle( void );

#endif

#endif
