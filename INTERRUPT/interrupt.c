void PORTA_IRQHandler(void) {
  NVIC_ClearPendingIRQ(PORTA_IRQn); 
  /* NVIC = Nested Vectored Interrupt Controller
  If an interrupt is marked "pending" in NVIC, this clears that pending flag
  It's defensive: makes sure there's no leftover pending state for PORTA
  */
  
  GPIOD_PTOR |= 0xFF; 
  /* GPIOD_PTOR = Port D Toggle Output Register
  0xFF = bits 0-7 -> all LEDs 
  Writing 1s here flips their current state
  This is where the visiable effect of the interrupt happens: 
  LEDs toggle once per interrupt 
  */

  PORTA_ISFR = (1 << 1);
  /* PORTA_ISFR = Interrupt Status Flag Register for PORTA
  Bit 1 corresponds to PTA1
  Writing a 1 to the bit cleras the interrupt flag 
  This is crucial: If you don't clear ISFR, the interrupt line stays active
  and you usually won't get subsequent edges
  After this line, the hardware is ready to detect the next falling edge
  */
}

void main(void) {
  SIM_SCGC5 |= (SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTD_MASK);
  /* Turn on the peripheral clocks for PORTA and PORTD
  No clock -> port doesn't work -> always do this first 
  */

  PORTA_PCR1 = 0xA0100; 
  /* Select GPIO function for PTA1 (MUX=001)
  Enable interrupt on falling edge (IRQC bits = "interrupt on falling edge"
  So PTA1 is a GPIO pin, wired to generate an interrupt when it goes from 1 -> 0
  */

  PORTD_GPCLR = 0x00FF0100; 
  /* Uses Global Pin Control register
  Upper bits select PD0-PD7
  Lower bits set their PCRs (again, MUX=GPIO)
  Result: PTD0-PTD7 are GPIO pins for LEDs
  */

  GPIOA_PDDR |= (0 << 1);
  /* 0 = input
  This line is a bit goofy since (0 << 1) is 0. more correct would be clearing the bit
  conceptually: PTA1 is input 
  */

  GPIOD_PDDR |= 0x000000FF; 
  /* PORTD[7:0] output mode 
  Sets bit 0-7 to 1 -> PTD0-PTD7 as outputs -> can drive LEDs
  */

  GPIOD_PDOR |= 0x00; 
  /* Leaves LEDs all off 
  In practice you might explicitlu clear: GPIOD_PDOR &= ~0xFF;
  */

  PORTA_ISFR = (1 << 1); 
  /* In case there's a stale flag from before enabling
  write 1 to bit 1 -> clera interrupt flag for PTA1 
  start with a clean state, so the first real edge will trigger the ISR
  */

  NVIC_EnableIRQ(PORTA_IRQn);
  /* Tells the NVIC: "accept interrupts from the PORTA line" 
  we already tought PORTA when to generate interrupts (PCR1) 
  Now we globally allow that interrupt into the CPU
  */

  while(1); 
}
