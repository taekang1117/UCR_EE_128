#include "fsl_device_registers.h"

const uint8_t stepPattern[4][4] = {
    {0, 1, 1, 0},  
    {1, 0, 1, 0},  
    {1, 0, 0, 1},  
    {0, 1, 0, 1}   
};

volatile int currentStep = 0;
volatile uint8_t direction = 0;  // 0 = CW, 1 = CCW
volatile uint8_t speed = 0;      // 0 = slow, 1 = fast

volatile int pit_isr_count = 0;  // Helps with breakpoints

void init_gpio(void);
void init_pit(void);
void read_switches(void);
void output_step(int step);

int main(void) 
{
    init_gpio();

    // Read DIP switches before PIT starts
    read_switches();

    init_pit();

    __enable_irq();   // IMPORTANT

    while(1)
    {
        read_switches();   // breakpoint here to inspect switch values
    }
}

void init_gpio(void)
{
    SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
    SIM_SCGC5 |= SIM_SCGC5_PORTD_MASK;

    PORTD_PCR0 = 0x100;
    PORTD_PCR1 = 0x100;
    PORTD_PCR2 = 0x100;
    PORTD_PCR3 = 0x100;
    PORTD_PCR4 = 0x100;
    PORTD_PCR5 = 0x100;

    PORTB_PCR2 = 0x103;  // pull-up
    PORTB_PCR3 = 0x103;

    GPIOD_PDDR |= 0x3F;   // PD0–PD5 outputs
    GPIOB_PDDR &= ~((1<<2)|(1<<3)); 

    GPIOD_PDOR &= ~0x3F;            // all low
    GPIOD_PDOR |= (1<<4)|(1<<5);    // ENA, ENB HIGH
}

void init_pit(void)
{
    SIM_SCGC6 |= SIM_SCGC6_PIT_MASK;
    PIT_MCR = 0x00;

    PIT_LDVAL0 = 15000000 - 1;  // slow default

    PIT_TCTRL0 = 0x02; 
    PIT_TFLG0 = 0x01;

    NVIC_ClearPendingIRQ(PIT0_IRQn);
    NVIC_EnableIRQ(PIT0_IRQn);

    PIT_TCTRL0 |= 0x01;  // TEN = 1
}

void read_switches(void)
{
    int new_dir = (GPIOB_PDIR & (1<<2)) ? 0 : 1;
    int new_spd = (GPIOB_PDIR & (1<<3)) ? 0 : 1;

    direction = new_dir;

    if (speed != new_spd)
    {
        speed = new_spd;

        PIT_TCTRL0 &= ~0x01;

        if (speed == 0)
            PIT_LDVAL0 = 15000000 - 1;
        else
            PIT_LDVAL0 = 1875000 - 1;

        PIT_TCTRL0 |= 0x01;
    }
}

void output_step(int step)
{
    uint32_t out = 0;

    if (stepPattern[step][0]) out |= (1<<0);
    if (stepPattern[step][1]) out |= (1<<1);
    if (stepPattern[step][2]) out |= (1<<2);
    if (stepPattern[step][3]) out |= (1<<3);

    GPIOD_PDOR = (GPIOD_PDOR & ~0x0F) | (out & 0x0F);   
}

void PIT0_IRQHandler(void)
{
    pit_isr_count++;  // breakpoint here to confirm ISR firing

    output_step(currentStep);

    if (direction == 0)
    {
        currentStep++;
        if (currentStep > 3) currentStep = 0;
    }
    else
    {
        currentStep--;
        if (currentStep < 0) currentStep = 3;
    }

    PIT_TFLG0 = 0x01;
}
