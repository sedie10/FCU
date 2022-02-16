#ifndef __ASK_H__
#define __ASK_H__

#include <stdint.h>

#define PREAMCNT            25
#define ASK_PREAM_STATE     0
#define ASK_HEADER_STATE    1
#define ASK_DATA_STATE      2
#define ASK_CHECKSUM_STATE  3


void DataPacketStart(uint8_t header); //buffer <- data update & ASK start ( 1packet transfer )
void PB14Toggle();
void AskOut();
void AskByteOut(uint8_t Data);
void MyDelay(int count);

#endif