/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Demonstrate the usage of Cortex-M4 MPU.
 *
 *
 * @copyright (C) 2020 FCUnwired Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "WPCQiTx.h"
#include "Hardware.h"
#include "Ble.h"


int main()
{
    /* Init System, IP clock and multi-function I/O */
	SYS_vInit();


	while(1)
	{

#if defined(BLE_HANDSHAKING)
	   if( getStatus_UART0())
	   {
	      UART0_Proc(getString_UART0());
	   }
#endif	   

	};
}

/*** (C) COPYRIGHT 2020 FCUnwired Corp. ***/
