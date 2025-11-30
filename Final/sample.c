/*
 * K64F - Duty Cycle Meter with Dual 7-Segment Display
 * Measures pulse width using FTM3 Input Capture
 * Displays duty cycle percentage on two 7-segment displays
 *
 * Connections:
 * - PWM Input: PTC10 (FTM3_CH6)
 * - 7-Segment Display 1 (ones): Port D (PD0-PD7)
 * - 7-Segment Display 2 (tens): Port C (PC0-PC5, PC7-PC8)
 */
#include "fsl_device_registers.h"

// 7-segment display digit patterns (common cathode)
int nums[10] = {
    0b1111110, // 0
    0b0110000, // 1
    0b1101101, // 2
    0b1111001, // 3
    0b0110011, // 4
    0b1011011, // 5
    0b1011111, // 6
    0b1110000, // 7
    0b1111111, // 8
    0b1111011  // 9
};

enum states{start, Locked, Unlocked} state;

/*void Tick_Fct()
{
	switch(state){
	case start:
		state = Locked;
		break;
	case Locked:
		if()
			state = Locked;
		else if()
	}
}
*/
volatile uint8_t TickFlag = 0;
volatile uint32_t get_msTicks = 0;

void init_gpio(void);
void display_7_Segment(uint8_t duty);
void delay_ms(uint_32 ms)
{
	uint32_t init = get_msTicks;
	while((get_msTicks - init) < ms)
	{
		//break moment
	}
}


int main(void) {
    init_gpio();
    init_ftm3_input_capture();

    while (1) {
        // Display duty cycle on 7-segment displays
        /*if(TickFlag)
        {
        	Tickflag = 0;
        	//Tick Fct here
        }*/

        GPIOC_PTOR = (1 << 8);  // or write/set/clear as in your lab
        delay_ms(500);                // wait 500 ms
    }
}

void init_gpio(void) {
    // Enable clock for Port C and D
    SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;
    SIM_SCGC5 |= SIM_SCGC5_PORTD_MASK;

    // Configure PTC10 as FTM3_CH6 (ALT3) for input capture
    PORTC_PCR10 = 0x300;  // ALT3

    // Initialize PORT D as GPIO (ones place display)
    PORTD_PCR0 = 0x100;
    PORTD_PCR1 = 0x100;
    PORTD_PCR2 = 0x100;
    PORTD_PCR3 = 0x100;
    PORTD_PCR4 = 0x100;
    PORTD_PCR5 = 0x100;
    PORTD_PCR6 = 0x100;
    PORTD_PCR7 = 0x100;

    // Initialize PORT C as GPIO (tens place display)
    PORTC_PCR0 = 0x100;
    PORTC_PCR1 = 0x100;
    PORTC_PCR2 = 0x100;
    PORTC_PCR3 = 0x100;
    PORTC_PCR4 = 0x100;
    PORTC_PCR5 = 0x100;
    PORTC_PCR7 = 0x100;
    PORTC_PCR8 = 0x100;

    // Set PORT D as output (PD0-PD7)
    GPIOD_PDDR |= 0xFF;

    // Set PORT C as output (PC0-PC5, PC7-PC8)
    GPIOC_PDDR |= (1<<0) | (1<<1) | (1<<2) | (1<<3) | (1<<4) | (1<<5) | (1<<7) | (1<<8);

    // Initialize displays to show 00
    //GPIOD_PDOR = nums[0] & 0xFF;
    //GPIOC_PDOR = (nums[0] & 0x3F) | ((nums[0] & 0xC0) << 1);
}

void init_ftm3_input_capture(void) {
    // Enable FTM3 clock
    SIM_SCGC3 |= SIM_SCGC3_FTM3_MASK;

    // Disable write protection
    FTM3_MODE = 0x05;  // Enable FTM3

    // Set modulo to maximum
    FTM3_MOD = 0xA40;

    // Set prescaler to 8 (System clock / 8)
    FTM3_SC = 0x4B;  // Clock source: system clock, prescaler = 8

    // Enable FTM3 Channel 6 interrupt in NVIC
    NVIC_EnableIRQ(FTM3_IRQn);
}

// FTM3 ISR - Measures pulse width and period
void FTM3_IRQHandler(void) {
    if (FTM3_SC & FTM_SC_TOF_MASK) {  // Check TOIE flag

    	FTM3_SC &= ~FTM_SC_TOF_MASK;
    	get_msTicks++;
        }

        // Clear TOIE flag

    }
}

// Display duty cycle percentage on 7-segment displays
void display_duty_cycle(uint8_t duty) {
    int tensPlace = duty / 10;
    int onesPlace = duty % 10;

    // Update ones place (Port D)
    GPIOD_PDOR = (GPIOD_PDOR & ~0xFF) | (nums[onesPlace] & 0xFF);

    // Update tens place (Port C) - matches your reference code format
    GPIOC_PDOR = (nums[tensPlace] & 0x3F) | ((nums[tensPlace] & 0xC0) << 1);
}
