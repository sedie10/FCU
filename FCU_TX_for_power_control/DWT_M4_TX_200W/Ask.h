#ifndef _ASK_H
#define _ASK_H

#include "Targetdef.h"

#define _16MHZ_PCLK				0
#define _40MHZ_PCLK				0
#define _36MHZ_PCLK				0
#define _96MHZ_PCLK				1

#if _16MHZ_PCLK // 16MHz timer clock
#define TOLERANED_MARGEN                 30
//8000*0.06255us=500us
// 10400 => 28A0  , 5600 => 15E0

#define FULL_500U_PERIOD_VALUE           8000   
#define FULL_500U_PERIOD_VALUE_UP        10400
#define FULL_500U_PERIOD_VALUE_LOW       5800

//4000*0.06255us=250us
#define HALF_250U_PERIOD_VALUE           4000   

#define HALF_250U_PERIOD_VALUE_UP        5600
#define HALF_250U_PERIOD_VALUE_LOW       2300

#elif (_40MHZ_PCLK)	// 40MHz timer clock
#define TOLERANED_MARGEN                 30
//20000*0.025us=500us
//??? 10400 => 28A0  , 5600 => 15E0

#define FULL_500U_PERIOD_VALUE           20000   
#define FULL_500U_PERIOD_VALUE_UP        26000
#define FULL_500U_PERIOD_VALUE_LOW       14000

//10000*0.025us=250us
#define HALF_250U_PERIOD_VALUE           10000   

#define HALF_250U_PERIOD_VALUE_UP        13000
#define HALF_250U_PERIOD_VALUE_LOW        7000

#elif (_36MHZ_PCLK)
#define TOLERANED_MARGEN                 30
//8000*0.027778us=500us
// 10400 => 28A0  , 5600 => 15E0

#define FULL_500U_PERIOD_VALUE           	18000  /* 500us  */ 
//#define FULL_500U_PERIOD_VALUE_UP        23400
#define FULL_500U_PERIOD_VALUE_UP_33P			23940  /* 665us 33%*/
#define FULL_500U_PERIOD_VALUE_UP_30P			23400  /* 650us 30%*/
#define FULL_500U_PERIOD_VALUE_UP_25P			22500  /* 625us 25%*/
#define FULL_500U_PERIOD_VALUE_LOW_33P   	12060  /* 375us 33%*/
#define FULL_500U_PERIOD_VALUE_LOW_30P   	12600  /* 350us 30%*/
#define FULL_500U_PERIOD_VALUE_LOW_25P   	13500  /* 375us 25%*/
#define FULL_500U_PERIOD_VALUE_LOW_20P   	14400  /* 400us 20%*/
#define FULL_500U_PERIOD_VALUE_LOW_15P   	15300  /* 425us 15%*/
#define FULL_500U_PERIOD_VALUE_UP        	FULL_500U_PERIOD_VALUE_UP_33P
#define FULL_500U_PERIOD_VALUE_LOW       	FULL_500U_PERIOD_VALUE_LOW_33P

//4000*0.06255us=250us
#define HALF_250U_PERIOD_VALUE           9000   

#define HALF_250U_PERIOD_VALUE_UP_33P 		11970 /* 333us 33%*/
#define HALF_250U_PERIOD_VALUE_UP_30P 		11700 /* 325us 30%*/
#define HALF_250U_PERIOD_VALUE_UP_25P 		11250 /* 313us 25%*/
#define HALF_250U_PERIOD_VALUE_LOW_33P		6030  /* 167us  33%*/
#define HALF_250U_PERIOD_VALUE_LOW_30P		6300  /* 175us  30%*/
#define HALF_250U_PERIOD_VALUE_LOW_25P		6750  /* 188us 25%*/
#define HALF_250U_PERIOD_VALUE_LOW_20P		7200  /* 200us 20% */
#define HALF_250U_PERIOD_VALUE_LOW_15P		7650  /* 213us 15% */
#define GLITCH_150U_PERIOD_VALUE          5400  /* 150us */
#define GLITCH_100U_PERIOD_VALUE          3600  /* 100us */
#define GLITCH_50U_PERIOD_VALUE          	1800  /* 50us */
#define HALF_250U_PERIOD_VALUE_UP        	HALF_250U_PERIOD_VALUE_UP_33P /* 333us 33%*/
#define HALF_250U_PERIOD_VALUE_LOW				HALF_250U_PERIOD_VALUE_LOW_33P  /* 164us  33%*/

#define ONBITPERIOD_MARGEN                1500

#elif (_96MHZ_PCLK)

#define TOLERANED_MARGEN                 30
//8000*0.027778us=500us
// 10400 => 28A0  , 5600 => 15E0

#define FULL_500U_PERIOD_VALUE           	24000  /* 500us  */ 
//#define FULL_500U_PERIOD_VALUE_UP        23400
#define FULL_500U_PERIOD_VALUE_UP_33P			31920  /* 665us 33%*/
#define FULL_500U_PERIOD_VALUE_UP_30P			31200  /* 650us 30%*/
#define FULL_500U_PERIOD_VALUE_UP_25P			30000  /* 625us 25%*/
#define FULL_500U_PERIOD_VALUE_LOW_33P   	16080  /* 375us 33%*/
#define FULL_500U_PERIOD_VALUE_LOW_30P   	16800  /* 350us 30%*/
#define FULL_500U_PERIOD_VALUE_LOW_25P   	18000  /* 375us 25%*/
#define FULL_500U_PERIOD_VALUE_LOW_20P   	19200  /* 400us 20%*/
#define FULL_500U_PERIOD_VALUE_LOW_15P   	20400  /* 425us 15%*/
#define FULL_500U_PERIOD_VALUE_UP        	FULL_500U_PERIOD_VALUE_UP_33P
#define FULL_500U_PERIOD_VALUE_LOW       	FULL_500U_PERIOD_VALUE_LOW_33P

//4000*0.06255us=250us
#define HALF_250U_PERIOD_VALUE           	12000   

#define HALF_250U_PERIOD_VALUE_UP_33P 		15960 /* 333us 33%*/
#define HALF_250U_PERIOD_VALUE_UP_30P 		15600 /* 325us 30%*/
#define HALF_250U_PERIOD_VALUE_UP_25P 		15000 /* 313us 25%*/
#define HALF_250U_PERIOD_VALUE_LOW_33P		8040  /* 167us  33%*/
#define HALF_250U_PERIOD_VALUE_LOW_30P		8400  /* 175us  30%*/
#define HALF_250U_PERIOD_VALUE_LOW_25P		9000  /* 188us 25%*/
#define HALF_250U_PERIOD_VALUE_LOW_20P		9600  /* 200us 20% */
#define HALF_250U_PERIOD_VALUE_LOW_15P		10200  /* 213us 15% */
#define GLITCH_150U_PERIOD_VALUE          7200  /* 150us */
#define GLITCH_100U_PERIOD_VALUE          4800  /* 100us */
#define GLITCH_50U_PERIOD_VALUE          	2400  /* 50us */
#define HALF_250U_PERIOD_VALUE_UP        	HALF_250U_PERIOD_VALUE_UP_33P /* 333us 33%*/
#define HALF_250U_PERIOD_VALUE_LOW				HALF_250U_PERIOD_VALUE_LOW_33P  /* 164us  33%*/

#define ONBITPERIOD_MARGEN                1500


#endif

typedef enum
{
  ASKDECODE_PREAMBLE_HUNT_EDGE_S = 0, //preamble_hunt_edge = 0,                                                     /*!< Hunting for edge during pre-amble */
  ASKDECODE_DATA_HUNT_EDGE_S,         //hunt_edge,                                                              /*!< Hunting for edge */
    
} WPCQiTX_AskDecodeState_t; 
//qi_Comm_Bit_State;


typedef enum
{
  
  // Byte states
  
  ASKBITDECODE_START_S, //byte_hunting_start,
  ASKBITDECODE_DATA_S,    //byte_receiving_data,
  ASKBITDECODE_PARITY_S,  //byte_hunting_parity,
  ASKBITDECODE_STOP_S,    //byte_hunting_stop,
} WPCQiTX_AskBitDecodeState_t; 
  // Message states
typedef enum
{
  
  ASKBYTEDECODE_START_S, //message_hunting_header,
  ASKBYTEDECODE_DATA_S,   //message_receiving_data,
  ASKBYTEDECODE_CHECKSUM_S, //message_hunting_checksum,

} WPCQiTX_AskByteDecodeState_t;


extern WPCQiTX_AskBitDecodeState_t askBitDecodeState;                     /*!< Current state of qi comm state machine */
extern WPCQiTX_AskByteDecodeState_t askByteDecodeState;                  /*!< Current state of the message state machine */

#endif

