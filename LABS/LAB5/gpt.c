#include "fsl_device_registers.h"
#include <stdio.h>

const uint8_t stepPattern[4][4] = {
    {0, 1, 1, 0},  
    {1, 0, 1, 0},  
    {1, 0, 0, 1},  
    {0, 1, 0, 1}   
};

volatile int currentStep = 0;
volatile uint8_t direction = 0;
volatile uint8_t speed = 0;

volatile int pit_isr_count = 0;

void init_gpio(void);
void init_pit(void);
void read_switches(void);
void output_step(int step);

int main(void) 
{
    init_gpio();

    read_switches();  

    init_pit();

    __enable_irq();

    printf("Debug start. Direction=%d Speed=%d\n", direction, speed);

    int last_isr = 0;

    while(1)
    {
        read_switches();

        if (pit_isr_count != last_isr)
        {
            last_isr = pit_isr_count;
            printf("ISR=%d  Step=%d  Dir=%d  Spd=%d  PDOR=0x%x\n",
                   pit_isr_count,
                   currentStep,
                   direction,
                   speed,
                   GPIOD_PDOR);
        }
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

    PORTB_PCR2 = 0x103;
    PORTB_PCR3 = 0x103;

    GPIOD_PDDR |= 0x3F;
    GPIOB_PDDR &= ~((1<<2)|(1<<3));

    GPIOD_PDOR &= ~0x3F;
    GPIOD_PDOR |= (1<<4)|(1<<5);

    printf("init_gpio done. PDOR=0x%x PDDR=0x%x\n", GPIOD_PDOR, GPIOD_PDDR);
}

void init_pit(void)
{
    SIM_SCGC6 |= SIM_SCGC6_PIT_MASK;
    PIT_MCR = 0x00;

    PIT_LDVAL0 = 15000000 - 1;

    PIT_TCTRL0 = 0x02;
    PIT_TFLG0 = 0x01;

    NVIC_ClearPendingIRQ(PIT0_IRQn);
    NVIC_EnableIRQ(PIT0_IRQn);

    PIT_TCTRL0 |= 0x01;

    printf("init_pit done. LDVAL=%d TCTRL=0x%x\n", PIT_LDVAL0, PIT_TCTRL0);
}

void read_switches(void)
{
    static int last_dir = -1;
    static int last_spd = -1;

    int new_dir = (GPIOB_PDIR & (1<<2)) ? 0 : 1;
    int new_spd = (GPIOB_PDIR & (1<<3)) ? 0 : 1;

    if (new_dir != last_dir || new_spd != last_spd)
    {
        last_dir = new_dir;
        last_spd = new_spd;
        printf("Switches: DIR=%d SPD=%d PDIR=0x%x\n", new_dir, new_spd, GPIOB_PDIR);
    }

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

        printf("Speed changed. New LDVAL=%d\n", PIT_LDVAL0);
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
    pit_isr_count++;

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
