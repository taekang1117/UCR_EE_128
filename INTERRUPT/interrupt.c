// Digital waveform: Port A, Bit 1
// LED: PORTD, Bits 0-7

void PORTA_IRQHandler(void) {
  NVIC_ClearPendingIRQ(PORTA_IRQn); // clear pending interrupt
  GPIOD_PTOR |= 0xFF; // toggle
  PORTA_ISFR = (1 << 1); // clear ISFR for PORTA, pin 1
}

void main(void) {
  SIM_SCGC5 |= (SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTD_MASK);
  PORTA_PCR1 = 0xA0100; 
  // TODO
}
