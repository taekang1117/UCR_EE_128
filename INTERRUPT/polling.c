void main (void) {
    SIM_SCGC5 |= (SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTD_MASK);
    /*  SIM_SCGC5 = System Integration Module System Clock Gating Control Register 5
    Each PORT's digital logic needs its ***clock enabled*** before use
    SIM_SCGC5_PORTA_MASK and SIM_SCGC5_PORTD_MASK are bitmasks
    |= turns on both PORTA and PORTD clocks
    Without this line: all later PORT configs silently do nothing
    */ 

    PORTA_PCR1 = 0x0100;      
    /* PORTA_PCR1 = Pin Control Register for PTA1 
    K64F pins are multipurpose; PCR choose the function
    0x100 sets MUX = 001 -> Select GPIO function (and default settings)
    So PTA1 is now a GPIO pin (not UART, I2C, etc)
    */
  
    PORTD_GPCLR = 0x00FF0100; 
    /* PORTD_GPCLR is a clever "global PCR clear" helper 
    Upper 16 bits (0x00FF0000) select which pins 
    Lower 16 bits (0x0100) is the value written into their PCRs
    (here, MUX = 001 again) 

    0x00FF0000 -> select pins D0-D7
    0x0100 -> set them all to GPIO
    Net effect: PTD0-7 become GPIO pins 
    */
  
    GPIOA_PDDR |= (0 << 1);   
    /* GPIOA_PDDR = Port A Pin Data Direction Register 
    0 = input, 1 = output 
    (0 << 1) is literally 0, so |= with 0 does nothing 
    But logically: "ensure PTA1 is input" 
    In practice, you'd often write GPIODA_PDDR &= ~(1<<1); // clear bit 1-> input
    */
  
    GPIOD_PDDR |= 0x000000FF; 
    // GPIOD_PDDR bits 0-7 set to 1 -> PTD0-7 are outputs (to drive LEDs)
  
    GPIOD_PDOR |= 0x00;
    /* GPIOD_PDOR = Port D Data Output Register
    |= 0x00 does nothing; effectively "start with all LEDs off"
    More explicit would be GPIOD_PDOR &= ~0xFF; 
    */

    // Main loop
    while (1){

        // wait for falling edge
        while( GPIOA_PDIR & 0x2 );    
        /*  GPIOA_PDIR = Port A Data Input Register (read current pin levels)
        0x2 = 1 << 1 -> mask for PTA1
        condition: "while PTA1 is 1, keep looping" 
        so we spin until the signal goes from 1 -> 0
        That moment (when loop exits) is the falling edge detection
        */ 
      
        GPIOD_PTOR |= 0xFF;           
        /* GPIOD_PTOR = Port Toggle Output Register
        Writing 1s flips corresponding bits in PDOR
        0xFF toggles all 8 LEDs at once
        So every falling edge inverts the LED pattern 
        */
      
        while(!(GPIOA_PDIR & 0x2));   
        /* Now we wait until PTA1 becomes 1 again
        This ensures: 
        We don't keep toggling while input stays low
        Next toggle only happens on the next fall
        Together, the two while loops implement:
        "Wait for high -> low (fall), toggle once, wait for low -> high (rise), repeat
        */
    }
}
