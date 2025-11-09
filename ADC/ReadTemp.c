#include "fsl_device_registers.h"
unsigned short ADC_read16b(void);

// ADC Initialization in the main function 
int main(void) {
  // i -> used for delay loop
  // data -> stores ADC conversion result 
  unsigned int i, data; 

  // The System Integration Module (SIM) controls clocks to peripherals
  // This line turns ON the ADC0 module clock by setting the ADC0 bit in the SIM_SCGC6 register
  // Without enabling the clock, ADC0 registers won't function
  SIM_SCGC6 |= SIM_SCGC6_ADC0_MASK; // 0x8000000u; Enable ADC0 clock


  // CFG1 -> Sets conversion resolution, input clock, and clock divider
  ADC0_CFG1 = 0x0C; // 16 bits ADC; 0x0C = 0000 1100 , Sets 16 bit resolution using the BUS clock
  // Bits [7:6] is ADICLK -> Select Input source (BUS clock here)
  // Bits [5:4] is MODE -> Conversion mode: 16-bit (binary 11)
  // Bits [3:2] is ADLSMP -> Sample time (short, long)
  // Bits [1:0] is ADIV -> Clock divide ratio

  // SC1A -> Used to select the input channel, start a conversion, 
  // and configure interrupt and differential mode
  ADC0_SC1A = 0x1F; // Disable the module during init, ADCH - 11111 
  // ADCH = 11111 (0x1F) means "moudle disabled", no channel selected
  // This is a safe default initialization
  
  // Main loop
  while(1) {
      data = ADC_read16b(); // data increases as temperature decreases
      // After returning, the value is stored into data in the main loop
    
      for (i=0; i < 300000; i++); // Software delay
  }
  // Infinite loop repeatedly reads ADC values
  // ADC_read16b() starts a conversion and returns the result 
  // Delay loop ...for(i=0)... gives time between reads 0 purely software delay 
  // No timer used
}

// ADC reading function
unsigned short ADC_read16b(void) {
  // SC1A -> Used to select the input channel, start a conversion 
  // Write to SC1A to start conversion from ADC_0
  ADC0_SC1A = 0x1A; // Enable it back 
  // 0x1A = 0001 1010 , ADCH bits select the input channel 
  // COCO and and interrupt bits are cleared when you write

  // ADC0_SC2 -> Sets the trigger source, voltage reference, and compare mode
  // ADC0_SC1A -> Start a conversion and configure interrupt
  while(ADC0_SC2 & ADC_SC2_ADACT_MASK); // Conversion in progress
  // Refers to ADCx_SC2 (status/control register 2) 
  // The bit ADACT = 1 means "conversion active" 
  // This loop waits until conversion is done (ADACT cleared) 
  
  while(!(ADC0_SC1A & ADC_SC1_COCO_MASK)); // Until conversion completes 
  // SC1A has a bit COCO (Conversion Complete flag) 
  // waits until COCO=1, which signals conversion result ready 
  
  return ADC0_RA;
  // ADC0_RA now holds the 16-bit digital result of the analog input
}
