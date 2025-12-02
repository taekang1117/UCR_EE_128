#include "fsl_device_registers.h"
#include <stdint.h>
#include <stdbool.h>

#define SYSTEM_CLOCK (48000000u)  

// 7-segment display digit patterns 
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

typedef enum {
    STATE_LOCKED = 0,
    STATE_UNLOCKED
} AlarmState;

// --- Globals ---
volatile uint32_t msTicks      = 0;   // 1 ms ticks from SysTick
volatile uint8_t  displayFlag  = 0;   // set when we want to refresh 7-seg

AlarmState currentState = STATE_LOCKED;

// --- GPIO masks ---
#define RED_LED_MASK    (1u << 7)   // PTD7
#define GREEN_LED_MASK  (1u << 8)   // PTC8

// Servo on PORTA pin 1
#define SERVO_MASK      (1u << 1)   // PTA1

// Servo pulse width in microseconds (1 ms = lock, 1.5 ms = unlock, tune as needed)
volatile uint16_t servo_pulse_us = 1000;  // start locked

// LED helpers
static inline void red_on(void)    { GPIOD_PSOR = RED_LED_MASK; }
static inline void red_off(void)   { GPIOD_PCOR = RED_LED_MASK; }
static inline void green_on(void)  { GPIOC_PSOR = GREEN_LED_MASK; }
static inline void green_off(void) { GPIOC_PCOR = GREEN_LED_MASK; }

// 7-seg display: PD0–PD6 segments, PD7 is LED
void display_digit(uint8_t digit)
{
    if (digit > 9) digit = 9;
    uint32_t pattern = (uint32_t)(nums[digit] & 0x7F); // bits 0–6

    uint32_t current = GPIOD_PDOR & ~0x7F;  // clear PD0–PD6
    GPIOD_PDOR = current | pattern;
}

// ---------------- Small delay in microseconds ----------------
static inline void delay_us(uint16_t us)
{
    // Approximate busy-wait using core clock.
    // 1 us -> 48 cycles. We divide by ~4 to account for loop overhead.
    uint32_t cycles = (SYSTEM_CLOCK / 1000000u) * us / 4u;
    while (cycles--) {
        __NOP();
    }
}

// ---------------- Servo helpers (software PWM on PTA1) ----------------
void init_servo_pin(void)
{
    // Enable PORTA clock
    SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;

    // PTA1 as GPIO
    PORTA_PCR1 = PORT_PCR_MUX(1);
    GPIOA_PDDR |= SERVO_MASK;   // output
    GPIOA_PDOR &= ~SERVO_MASK;  // low
}

// One 20 ms period pulse: high for servo_pulse_us, then low
void servo_pulse_once(void)
{
    GPIOA_PSOR = SERVO_MASK;              // high
    delay_us(servo_pulse_us);             // keep high for pulse width
    GPIOA_PCOR = SERVO_MASK;              // low
}

// Position functions - adjust pulse widths if needed
void servo_lock_position(void)
{
    servo_pulse_us = 1000;    // ~1 ms pulse (tune for your servo "locked" angle)
}

void servo_unlock_position(void)
{
    servo_pulse_us = 1500;    // ~1.5 ms pulse (tune for "unlocked" angle)
}

// ---------------- SysTick (1 ms) ----------------
void SysTick_Handler(void)
{
    msTicks++;
}

// ---------------- UART1 (PTC3 RX, PTC4 TX) ----------------
void init_uart1(uint32_t baudrate)
{
    SIM_SCGC4 |= SIM_SCGC4_UART1_MASK;
    SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;

    // PTC3 = UART1_RX, PTC4 = UART1_TX (ALT3)
    PORTC_PCR3 = PORT_PCR_MUX(3);
    PORTC_PCR4 = PORT_PCR_MUX(3);

    UART1_C2 &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK); // disable

    uint16_t sbr = (uint16_t)(SYSTEM_CLOCK / (16u * baudrate));
    UART1_BDH = (UART1_BDH & ~UART_BDH_SBR_MASK) | ((sbr >> 8) & UART_BDH_SBR_MASK);
    UART1_BDL = (uint8_t)(sbr & 0xFF);

    UART1_C1 = 0x00;                       // 8N1
    UART1_C2 = UART_C2_RE_MASK;           // only RX enabled now (one-way UART)
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

// ---------------- GPIO init (LEDs + 7-seg + servo) ----------------
void init_gpio(void)
{
    SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK
              |  SIM_SCGC5_PORTB_MASK
              |  SIM_SCGC5_PORTC_MASK
              |  SIM_SCGC5_PORTD_MASK;

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
    GPIOD_PDOR &= ~0xFF;   // all low

    // Green LED on PTC8
    PORTC_PCR8 = PORT_PCR_MUX(1);
    GPIOC_PDDR |= GREEN_LED_MASK;
    GPIOC_PDOR &= ~GREEN_LED_MASK;

    // Servo pin PTA1
    init_servo_pin();
}

// ---------------- SysTick init ----------------
void init_systick(void)
{
    uint32_t reload = (SYSTEM_CLOCK / 1000u) - 1u; // 1 ms
    SysTick->LOAD = reload;
    SysTick->VAL  = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk
                  | SysTick_CTRL_TICKINT_Msk
                  | SysTick_CTRL_ENABLE_Msk;
}

// ---------------- main ----------------
int main(void)
{
    init_gpio();
    init_uart1(9600);
    init_systick();

    // Start LOCKED
    currentState = STATE_LOCKED;
    servo_lock_position();
    red_on();
    green_off();
    display_digit(9);     // show 9 when locked

    uint32_t lastServoPulseMs = 0;

    while (1) {

        // --- 1) Handle UART: toggle on '1' ---
        if (UART1_Available()) {
            char c = UART1_GetChar();

            if (c == '1') {
                if (currentState == STATE_LOCKED) {
                    // Go to UNLOCKED
                    currentState = STATE_UNLOCKED;
                    servo_unlock_position();
                    red_off();
                    green_on();
                    display_digit(0);     // show 0 when unlocked (your choice)
                } else {
                    // UNLOCKED -> LOCKED
                    currentState = STATE_LOCKED;
                    servo_lock_position();
                    red_on();
                    green_off();
                    display_digit(9);     // show 9 when locked
                }
            }
        }

        // --- 2) Servo refresh: send one pulse every ~20 ms ---
        if ((msTicks - lastServoPulseMs) >= 20u) {
            lastServoPulseMs = msTicks;
            servo_pulse_once();
        }
    }
}
