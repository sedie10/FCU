# FCU

WPC Qi Standard Wireless Power Transfer TX and RX code

Download firmware to board
TX : FCU_TX_for_power_control/DWT_M4_TX_200W/Keil/obj/DWT_M4_TX.bin
RX : FCU_RX_STATE3/KEIL/obj/TIMER_PowerDown.bin
(or FCU_RX_STATE2) 

TX
- nego state time delay 300~500ms -> removed
- In PowerTransfer State, 24bit received power : 110KHz <-> 145KHz power control

RX
- FCU_RX_STATE2 -> just 1PATH EPP (no power control)
- FCU_RX_STATE3 -> power control example with 24bit received power message (EPP)
- NO FSK ver.
