#include "fsl_device_registers.h"
#include <stdint.h>

#define SYSTEM_CLOCK (120000000u)   // try 120 MHz first; if nothing works we'll try 48 MHz

// 7-seg patterns (common cathode) on PD0–PD6
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

#define RED_LED_MASK   (1u << 7)   // PTD7
#define GREEN_LED_MASK (1u << 8)   // PTC8

static inline void red_on(void)    { GPIOD_PSOR = RED_LED_MASK; }
static inline void red_off(void)   { GPIOD_PCOR = RED_LED_MASK; }
static inline void green_on(void)  { GPIOC_PSOR = GREEN_LED_MASK; }
static inline void green_off(void) { GPIOC_PCOR = GREEN_LED_MASK; }

void display_digit(uint8_t digit)
{
    if (digit > 9) digit = 9;
    uint32_t pattern = (uint32_t)(nums[digit] & 0x7F);

    uint32_t current = GPIOD_PDOR & ~0x7F;  // clear PD0–PD6, keep PD7
    GPIOD_PDOR = current | pattern;
}

void init_gpio(void)
{
    SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK | SIM_SCGC5_PORTD_MASK;

    // PD0–PD7: 7-seg + red LED
    PORTD_PCR0 = PORT_PCR_MUX(1);
    PORTD_PCR1 = PORT_PCR_MUX(1);
    PORTD_PCR2 = PORT_PCR_MUX(1);
    PORTD_PCR3 = PORT_PCR_MUX(1);
    PORTD_PCR4 = PORT_PCR_MUX(1);
    PORTD_PCR5 = PORT_PCR_MUX(1);
    PORTD_PCR6 = PORT_PCR_MUX(1);
    PORTD_PCR7 = PORT_PCR_MUX(1);
    GPIOD_PDDR |= 0xFF;
    GPIOD_PDOR &= ~0xFF;

    // Green LED on PTC8
    PORTC_PCR8 = PORT_PCR_MUX(1);
    GPIOC_PDDR |= GREEN_LED_MASK;
    GPIOC_PDOR &= ~GREEN_LED_MASK;
}

void init_uart1(uint32_t baudrate)
{
    SIM_SCGC4 |= SIM_SCGC4_UART1_MASK;
    SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;

    // PTC3 = UART1_RX, PTC4 = UART1_TX (ALT3)
    PORTC_PCR3 = PORT_PCR_MUX(3);
    PORTC_PCR4 = PORT_PCR_MUX(3);

    UART1_C2 &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK); // disable TX/RX

    uint16_t sbr = (uint16_t)(SYSTEM_CLOCK / (16u * baudrate));
    UART1_BDH = (UART1_BDH & ~UART_BDH_SBR_MASK) | ((sbr >> 8) & UART_BDH_SBR_MASK);
    UART1_BDL = (uint8_t)(sbr & 0xFF);

    UART1_C1 = 0x00;  // 8N1
    UART1_C2 = UART_C2_TE_MASK | UART_C2_RE_MASK;  // enable TX/RX
}

int UART1_Available(void)
{
    return (UART1_S1 & UART_S1_RDRF_MASK) != 0;
}

char UART1_GetChar(void)
{
    while (!(UART1_S1 & UART_S1_RDRF_MASK)) { }
    return UART1_D;
}

int main(void)
{
    init_gpio();
    init_uart1(9600);

    red_on();
    green_off();
    display_digit(9);

    while (1) {
        if (UART1_Available()) {
            char c = UART1_GetChar();

            // Toggle green LED on every received byte
            GPIOC_PTOR = GREEN_LED_MASK;

            if (c == '1') {
                display_digit(0);
            } else {
                display_digit(8);
            }
        }
    }
}
