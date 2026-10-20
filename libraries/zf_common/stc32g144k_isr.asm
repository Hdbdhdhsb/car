;-------------------------------------------------------------------------------------------------------------------
; File    : stc32g144k_isr.asm
; Purpose : High interrupt-vector trampoline for STC32G144K246.
;           Keil C251 "interrupt N" only supports 0..31, but this chip uses
;           several hardware vectors >31. This table LJMPs each high vector to
;           the remapped low-numbered C ISR defined in isr.c.
; Note    : vector address = 3 + 8 * vector_number
;-------------------------------------------------------------------------------------------------------------------

	CSEG AT 019BH		; DMA_UR1R_VECTOR   (51)  UART1 DMA RX
	LJMP 002BH			;   -> low vector 5
	CSEG AT 01ABH		; DMA_UR2R_VECTOR   (53)  UART2 DMA RX
	LJMP 0033H			;   -> low vector 6
	CSEG AT 01BBH		; DMA_UR3R_VECTOR   (55)  UART3 DMA RX
	LJMP 003BH			;   -> low vector 7
	CSEG AT 01CBH		; DMA_UR4R_VECTOR   (57)  UART4 DMA RX
	LJMP 004BH			;   -> low vector 9
	CSEG AT 029BH		; DMA_UR5R_VECTOR   (83)  UART5 DMA RX
	LJMP 006BH			;   -> low vector 13
	CSEG AT 02ABH		; DMA_UR6R_VECTOR   (85)  UART6 DMA RX
	LJMP 00ABH			;   -> low vector 21
	CSEG AT 02BBH		; DMA_UR7R_VECTOR   (87)  UART7 DMA RX
	LJMP 00B3H			;   -> low vector 22
	CSEG AT 02CBH		; DMA_UR8R_VECTOR   (89)  UART8 DMA RX
	LJMP 00BBH			;   -> low vector 23
	CSEG AT 0303H		; TMR5_TMR6_VECTOR  (96)  Timer5/6
	LJMP 00C3H			;   -> low vector 24
	CSEG AT 030BH		; TMR7_TMR8_VECTOR  (97)  Timer7/8
	LJMP 00F3H			;   -> low vector 30
	CSEG AT 0313H		; TMR9_TMR10_VECTOR (98)  Timer9/10
	LJMP 00D3H			;   -> low vector 26
	CSEG AT 021BH		; TMR11_VECTOR      (67)  Timer11
	LJMP 00DBH			;   -> low vector 27
	CSEG AT 031BH		; TMR17_TMR18_VECTOR(99)  Timer17/18
	LJMP 00E3H			;   -> low vector 28
	CSEG AT 01D3H		; DMA_LCM_VECTOR    (58)  MT9V03X camera DMA done
	LJMP 00EBH			;   -> low vector 29

	END
