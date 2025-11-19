#include "fsl_device_registers.h"

#define DELAY_FAST  3000u
#define DELAY_SLOW  6000u

volatile unsigned long i;   // used by Delay()

void init(void);
void Delay(unsigned long Speed);

int main(void)
{
    unsigned char FR1;   // current direction bit
    unsigned char FR2 = 0;   // previous direction bit
    unsigned long Speed;

    init();

    while (1)
    {
        // Read direction switch on PB2
        // With pull-up: open = 1, closed = 0
        FR1 = (GPIOB_PDIR & (1u << 2)) ? 1u : 0u;

        // Read speed switch on PB3
        // If bit is 1 → fast, else → slow (same as reference: (PORTB & 0x02)?delay_fast:delay_slow)
        Speed = (GPIOB_PDIR & (1u << 3)) ? DELAY_FAST : DELAY_SLOW;

        // If direction changed since last loop, force a specific pattern (0x35)
        // to avoid sudden jump in sequence (same as "if (FR2 != FR1) PORTA = 0x35;")
        if (FR2 != FR1)
        {
            GPIOD_PDOR = 0x35u;  // 0011 0101 → ENA&ENB=1, coils pattern for a stable step
        }

        if (FR1)  // One direction (say, CW)
        {
            GPIOD_PDOR = 0x36u;  // step 0: A2/B1 high
            Delay(Speed);

            GPIOD_PDOR = 0x35u;  // step 1
            Delay(Speed);

            GPIOD_PDOR = 0x39u;  // step 2
            Delay(Speed);

            GPIOD_PDOR = 0x3Au;  // step 3
            Delay(Speed);

            FR2 = FR1;           // remember direction
        }
        else      // Opposite direction (CCW) – reverse sequence
        {
            GPIOD_PDOR = 0x36u;  // step 0
            Delay(Speed);

            GPIOD_PDOR = 0x3Au;  // step 3
            Delay(Speed);

            GPIOD_PDOR = 0x39u;  // step 2
            Delay(Speed);

            GPIOD_PDOR = 0x35u;  // step 1
            Delay(Speed);

            FR2 = FR1;
        }
    }
}

void init(void)
{
    // Enable clocks for Port A/B/C/D as needed; here we only need B and D
    SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
    SIM_SCGC5 |= SIM_SCGC5_PORTD_MASK;

    // Configure Port D pins 0–5 as GPIO (MUX = 001)
    PORTD_PCR0 = 0x100u;  // PD0: IN1/A1
    PORTD_PCR1 = 0x100u;  // PD1: IN2/A2
    PORTD_PCR2 = 0x100u;  // PD2: IN3/B1
    PORTD_PCR3 = 0x100u;  // PD3: IN4/B2
    PORTD_PCR4 = 0x100u;  // PD4: ENA
    PORTD_PCR5 = 0x100u;  // PD5: ENB

    // Configure Port B pins 2–3 as GPIO with pull-up (active-low switches)
    // 0x103 = MUX(001) + PE(1) + PS(1)
    PORTB_PCR2 = 0x103u;  // direction switch
    PORTB_PCR3 = 0x103u;  // speed switch

    // DDRA = 0xFF in original → all A pins output
    // Here: GPIOD_PDDR bits 0–5 output
    GPIOD_PDDR |= 0x3Fu;

    // DDRB = 0x00 in original → all B pins input
    // Here: PB2, PB3 input
    GPIOB_PDDR &= ~((1u << 2) | (1u << 3));

    // PORTA = 0x30 in original:
    // 0x30 = 0011 0000 → ENA & ENB high, coils off initially.
    GPIOD_PDOR = 0x30u;
}

void Delay(unsigned long Speed)
{
    // Simple busy-wait loop, like the reference code
    for (i = 0; i < Speed; i++)
    {
        __NOP();    // optional: prevents aggressive optimization
    }
}
