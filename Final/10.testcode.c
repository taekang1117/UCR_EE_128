#include "fsl_device_registers.h"
#include <stdint.h>
#include <stdbool.h>

#define SYSTEM_CLOCK (48000000u)

// ----- Servo on PTA1 using FTM0_CH6 -----
#define SERVO_MIN_PULSE_TICKS  (375u)   // ~1.0 ms at 48 MHz / 128
#define SERVO_90_PULSE_TICKS   (563u)   // ~1.5 ms at 48 MHz / 128
#define SERVO_FTM_MOD          (7499u)  // 20 ms period (50 Hz) at 48 MHz / 128

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
    STATE_COUNTDOWN,
    STATE_UNLOCKED
} AlarmState;

// --- Globals ---
volatile uint32_t msTicks       = 0;
volatile uint32_t countdown_ms  = 0;
volatile uint8_t  seconds_left  = 9;
volatile uint8_t  displayFlag   = 0;
volatile uint8_t  buttonFlag    = 0;
volatile uint8_t  timeoutFlag   = 0;
volatile uint8_t  countdown_active = 0;

AlarmState currentState = STATE_LOCKED;

// --- GPIO masks ---
#define RED_LED_MASK    (1u << 7)   // PTD7
#define GREEN_LED_MASK  (1u << 8)   // PTC8
#define BUTTON_MASK     (1u << 2)   // PTB2

// --- New UART + button IRQ globals ---
volatile char     uart_rx_char       = 0;
volatile uint8_t  uart_rx_flag       = 0;   // set when a new byte arrives

volatile uint32_t last_button_ms     = 0;   // for debounce in IRQ
const uint32_t    BUTTON_DEBOUNCE_MS = 50;  // 50 ms debounce

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

// ---------------- SysTick (1 ms) ----------------
void SysTick_Handler(void)
{
    msTicks++;

    // Countdown runs in both COUNTDOWN and UNLOCKED states
    if (countdown_active &&
        (currentState == STATE_COUNTDOWN || currentState == STATE_UNLOCKED)) {

        if (countdown_ms > 0) {
            countdown_ms--;

            if ((countdown_ms % 1000u) == 0u && seconds_left > 0) {
                seconds_left--;
                displayFlag = 1;

                if (seconds_left == 0 && countdown_ms == 0) {
                    timeoutFlag = 1;
                    countdown_active = 0;
                }
            }
        }
    }
}

// ---------------- Button interrupt (PTB2, active low, FALLING EDGE) ----------------
void PORTB_IRQHandler(void)
{
    if (PORTB_ISFR & BUTTON_MASK) {
        // Clear interrupt status flag by writing 1
        PORTB_ISFR = BUTTON_MASK;

        // Debounce in ISR using msTicks
        uint32_t now = msTicks;
        if ((now - last_button_ms) >= BUTTON_DEBOUNCE_MS) {
            last_button_ms = now;
            buttonFlag = 1;  // only set if "real" press
        }
    }
}

// ---------------- UART1 (PTC3 RX, PTC4 TX) ----------------
void init_uart1(uint32_t baudrate)
{
    SIM_SCGC4 |= SIM_SCGC4_UART1_MASK;
    SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;

    // PTC3 = UART1_RX, PTC4 = UART1_TX (ALT3)
    PORTC_PCR3 = PORT_PCR_MUX(3);
    PORTC_PCR4 = PORT_PCR_MUX(3);

    // Disable TX/RX before configura
