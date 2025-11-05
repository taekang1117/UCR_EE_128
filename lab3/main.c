#include "fsl_device_registers.h"


long map(unsigned int x, long min1, long max1, long min2, long max2)
{
    return (x - min1) * (max2 - min2) / (max1 - min1) + min2;
}


int nums[10] = {0b1111110, // 0
0b0110000, // 1
0b1101101, // 2
0b1111001, // 3
0b0110011, // 4
0b1011011, // 5
0b1011111, // 6
0b1110000, // 7
0b1111111, // 8
0b1111011, // 9
};


unsigned short ADC_read16b(void)
{
ADC0_SC1A = 0x00; //Write to SC1A to start conversion from ADC_0
 while(ADC0_SC2 & 0x80); // Conversion in progress
 while(!(ADC0_SC1A & 0x80)); // Until conversion complete
 return ADC0_RA;
}


int cnt = 0;
int sigFig = 0;


int data;
int adcValue;
int tensPlace;
int onesPlace;


void PORTA_IRQHandler(void)
{
	GPIOC_PTOR = (1 << 9);  //GPIOC_PDOR = (~GPIOC_PDOR & 0x0200); // Swap decimal point.
    NVIC_ClearPendingIRQ(PORTA_IRQn);
            if ((GPIOB_PDIR & 0x04) == (1 << 2)) // Check if DIP switch 1 is ON
            {

                sigFig = cnt / 10; // Calculate the value of the tens position.
                onesPlace = cnt % 10;
                GPIOD_PDOR = (GPIOD_PDOR & ~0xFF) | (nums[onesPlace] & 0xFF); // Update displays with value.
                GPIOC_PDOR = (nums[sigFig] & 0x3F) | ((nums[sigFig] & 0xC0) << 1);
                if ((GPIOB_PDIR & 0x08) == (1 << 3)) // Check if DIP switch 1 is on.
                {


                    if (cnt == 99) // If i is not max, then increment.
                    {
                        cnt = 0;
                    }
                    else
                    {
                    cnt++;
                    }
                }
                else
                {
                    if (cnt == 0) // If i is not 0, then decrement.
                    {
                        cnt = 99;
                    }
                    else
                    {
                    cnt--;
                    }
                }




            }
            else
            {
                data = ADC_read16b(); // Read value of potentiometer.
                adcValue = map(data, 0, 65536, 0, 99); // Map value to a range of 0 - 99.
                tensPlace = adcValue / 10; // Calculate the ones and tens places, and update the displays accordingly.
                onesPlace = (adcValue - tensPlace * 10);
                GPIOD_PDOR = (GPIOD_PDOR & ~0xFF) | (nums[onesPlace] & 0xFF);
                GPIOC_PDOR = (nums[tensPlace] & 0x3F) | ((nums[tensPlace] & 0xC0) << 1);
            }
        PORTA_ISFR = (1 << 1);
}


int main(void)
{
    SIM_SCGC5 |= SIM_SCGC5_PORTD_MASK; /*Enable Port D Clock Gate Control*/
    SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK; /*Enable Port C Clock Gate Control*/
    SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;
    SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK; /*Enable Port A Clock Gate Control*/
    SIM_SCGC6 |= SIM_SCGC6_ADC0_MASK; // 0x8000000u; Enable ADC0 Clock


    PORTA_PCR1 = 0xA0100; /* Configure PORTA[1] for GPIO & Interrupt on Falling-Edge */
    PORTA_ISFR = (1 << 1); /* Clear ISFR for PORTA, Pin 1*/



    // Initialize PORT D as GPIO
    PORTD_PCR0 = 0x100;
    PORTD_PCR1 = 0x100;
    PORTD_PCR2 = 0x100;
    PORTD_PCR3 = 0x100;
    PORTD_PCR4 = 0x100;
    PORTD_PCR5 = 0x100;
    PORTD_PCR6 = 0x100;
    PORTD_PCR7 = 0x100;


    // Initialize PORT C as GPIO
    PORTC_PCR0 = 0x100;
    PORTC_PCR1 = 0x100;
    PORTC_PCR2 = 0x100;
    PORTC_PCR3 = 0x100;
    PORTC_PCR4 = 0x100;
    PORTC_PCR5 = 0x100;
    PORTC_PCR7 = 0x100;
    PORTC_PCR8 = 0x100;
    PORTC_PCR9 = 0x100;


    // Initialize PORT A as GPIO
    PORTB_PCR2 = 0x100;
    PORTB_PCR3 = 0x100;


    // Initialize PORT B GPIO as Input
    GPIOB_PDDR &= ~(1 << 2);
    GPIOB_PDDR &= ~(1 << 3);


    // Initialize PORT D GPIO as Output
    GPIOD_PDDR |= (1 << 7); /* SETTING 0-7 port D output */
	GPIOD_PDDR |= (1 << 6);
	GPIOD_PDDR |= (1 << 5);
	GPIOD_PDDR |= (1 << 4);
	GPIOD_PDDR |= (1 << 3);
	GPIOD_PDDR |= (1 << 2);
	GPIOD_PDDR |= (1 << 1);
	GPIOD_PDDR |= (1 << 0);

	GPIOC_PDDR |= (1<<0); /*setting port C 0-5 and 7-8 for output */
	GPIOC_PDDR |= (1<<1);
	GPIOC_PDDR |= (1<<2);
	GPIOC_PDDR |= (1<<3);
	GPIOC_PDDR |= (1<<4);
	GPIOC_PDDR |= (1<<5);
	GPIOC_PDDR |= (1<<7);
	GPIOC_PDDR |= (1<<8);
	GPIOC_PDDR |= (1<<9);



    ADC0_CFG1 = 0x0C; // 16bits ADC; Bus Clock
    ADC0_SC1A = 0x1F; // Disable the module during init, ADCH = 11111
    NVIC_EnableIRQ(PORTA_IRQn); /* Enable interrupts from PORTA */

    while(1);
    }
