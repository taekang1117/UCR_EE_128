unsigned short ADC_read16b(void){
  ADC0_SC1A = 0x1A; // Write to SC1A to start conversion from ADC_0
  while(ADC0_SC2 & ADC_SC2_ADACT_MASK);
  while(!(ADC0_SC1A & ADC_SC1_COCO_MASK)); 
  return ADC0_RA;
}

// A to D conversion can start by writing 0x1A (b11010) to ADC0_SC1A 
// The example code uses Temperature Sensor (that's why it is 11010) 
// Your lab will use 00000 (channel 0) for single-ended DADP0 input 
// (= ADC0_DP0 pin)
