// ADC reading function 
unsigned short ADC_read16b(void){
  // Writing to ADC0_SC1A starts a conversion 
  ADC0_SC1A = 0x1A;  // 0x1A = 0001 1010 
  // ADCH bits select the input channel 
  // COCO and interrupt bits are cleared when you write
  // For lab3, use channel 0, (00000) 
  // Single ended DADP0 input (= ADC0_DP0 pin)
  
  while(ADC0_SC2 & ADC_SC2_ADACT_MASK);
  // ADCx_SC2 to check conversion status 
  // This loop waits until conversion is done (ADACT cleared) 
  
  // ADCx_SC2 , Status Control Register 2
  // Used to configure trigger status and reference voltage
  // Bit 6: ADC conversion can be triggered by either HW and SW
  // Bit 5-2: Not necessary for this class and default is 0000
  // Bit 1-0: Reference voltage. "00" will use the defauly 3.3V and 0V voltage refrence
  
  
  while(!(ADC0_SC1A & ADC_SC1_COCO_MASK)); 
  // ADCx_SC1A to check if data is ready. 
  // 1 -> conversion is completed
  // 0 -> conversion is not completed
  
  return ADC0_RA;
}

// A to D conversion can start by writing 0x1A (b11010) to ADC0_SC1A 
// The example code uses Temperature Sensor (that's why it is 11010) 
// Your lab will use 00000 (channel 0) for single-ended DADP0 input 
// (= ADC0_DP0 pin)
