
#include "main.h"

#ifndef chip
  #define chip 2541 // ->ioCC254x_bitdef.h
#endif

#define DEBOUNCE 5000

uint16_t DebounceCnt = DEBOUNCE;

#pragma vector = P0INT_VECTOR 
__interrupt void P0_ISR(void){ // Port 0 Inputs handler

  if(READ_BIT(P0IFG, BIT0)){ // event: P0_0 (SB1- button) released
    R_W0_REG(P0IFG, BIT0); // Clear status flag for P0_0 with R/W0 method, see datasheet.
    if(!DebounceCnt){
      TOGGLE_BIT(P1, BIT0); // Toggle P1_0 (Green led).
      DebounceCnt = DEBOUNCE;
    }
  }
  CLEAR_BIT(IRCON, BIT5); // Clear CPU interrupt status flag for P0.
}


void main(void){
  
  sys_setup();
  
  while(1){
    
    for(;DebounceCnt > 0; DebounceCnt--);
    
  }
}

void sys_setup(void){
  
  /*Clock configuration, optional*/
  
  SET_BIT(SLEEPCMD, BIT7); // Disable the 32 kHz RCOSC calibration

  CLEAR_BIT(CLKCONCMD, BIT6); // Change the system clock source to HS XOSC
  MODIFY_REG(CLKCONCMD, CLKCON_CLKSPD, CLKCON_CLKSPD_32M); // Set the clock speed to 32 MHz
  // Wait until system clock source has changed to HS XOSC (CLKCONSTA.OSC = 0):
  while(READ_BIT(CLKCONSTA, BIT6));
  
  /* If calibration is not disabled, the 32 kHz RCOSC starts to calibrate ~2 ms.*/
  /* By reset timer ticks output set to 16MHz*/
  
   /*Setup I/O*/
  
  CLEAR_BIT(P0SEL, BIT0); // P0_0 as GPIO
  CLEAR_BIT(P0DIR, BIT0); // P0_0 as input
  CLEAR_BIT(P0INP, BIT0); // P0_0 pull up/pull down
  CLEAR_BIT(P2INP, BIT5); // P0 pullup

  CLEAR_BIT(P1SEL, BIT0); // P1_0, P1_1 as GPIO
  SET_BIT(P1DIR, BIT0); // P1_0, P1_1 as output
  CLEAR_BIT(P1, BIT0); // P1_0, P1_1 LOW

  /*Setup interrupts*/
  
  R_W0_REG(P0IFG, BIT0); // Clear status flag for P0_0
  CLEAR_BIT(IRCON, BIT5); // Clear CPU interrupt status flag for P0
  SET_BIT(P0IEN, BIT0); // Enable interrupt from P0_0
  CLEAR_BIT(PICTL, BIT0); // Rising edge P0
  SET_BIT(IEN1, BIT5); // Enable P0 interrupts
  SET_BIT(IEN0, BIT7); // Enable global interrupt

}
