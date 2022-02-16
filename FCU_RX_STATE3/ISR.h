#ifndef __ISR_H__
#define __ISR_H__


#define HALFBIT     256
#define FULLBIT     512

void TMR0_IRQHandler(void);
void GPAB_IRQHandler(void);
void UART0_IRQHandler(void);
void UART2_IRQHandler(void);


#endif