SIM_SCGC3 |= SIM_SCGC3_FTM3_MASK;   // Enable FTM3 clock
PORTC_PCR10 = PORT_PCR_MUX(3);      // ALT3 = FTM3_CH6
FTM3_SC = 0;                        // Disable timer during setup
FTM3_MOD = 7499;                    // Set MOD for 1 kHz (with prescaler 8)
FTM3_C6SC = FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK; // Edge-aligned PWM, high-true
FTM3_C6V = 2250;                    // 30% duty cycle
FTM3_SC = FTM_SC_CLKS(1) | FTM_SC_PS(3); // Use system clock, prescaler = 8
