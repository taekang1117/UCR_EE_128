#include "fsl_device_registers.h"
#include <stdint.h>
#include <stdbool.h>

#define SYSTEM_CLOCK (48000000u)  // 48 MHz core clock (FRDM-K64F default)

// 7-segment display digit patterns (common cathode, segments a-g on PD0-PD6)
// Bit 0 -> segment a, Bit 1 -> b, ... Bit 6 -> g
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
    STATE_COUNTDOWN
} AlarmState;

// --- Global volatile flags/counters ---
volatile uint32_t msTicks = 0;          // global ms tick from SysTick
volatile uint32_t countdown_ms = 0;     // remaining countdown in ms
volatile uint8_t  seconds_left = 15;    // remaining seconds (for display)
volatile uint8_t  displayUpdateFlag = 0;
volatile uint8_t  buttonPressedFlag = 0;
volatile uint8_t  timeExpiredFlag = 0;
volatile uint8_t  countdown_active = 0;

AlarmState currentState = STATE_LOCKED;

// --- GPIO masks ---
#define RED_LED_MASK    (1u << 7)   // PTD7
#define GREEN_LED_MASK  (1u << 8)   // PTC8
#define BUTTON_MASK     (1u << 2)   // PTB2 (active low)

// -----------------------------------------------------------------------------
// Function prototypes
// -----------------------------------------------------------------------------
void init_gpio(void);
void init_systick(void);
void init_uart1(uint32_t baudrate);
int  UART1_Available(void);
char UART1_GetChar(void);
void UART1_PutChar(char c);
void display_digit(uint8_t digit);
void delay_ms(uint32_t ms);

// -----------------------------------------------------------------------------
// Simple helper functions for LEDs
// -----------------------------------------------------------------------------
static inline void red_on(void)   { GPIOD_PSOR = RED_LED_MASK;  }  // set PD7
static inline void red_off(void)  { GPIOD_PCOR = RED_LED_MASK;  }
static inline void green_on(void) { GPIOC_PSOR = GREEN_LED_MASK; }
static inline void green_off(void){ GPIOC_PCOR = GREEN_LED_MASK; }

// -----------------------------------------------------------------------------
// SysTick Handler: called every 1 ms
// -----------------------------------------------------------------------------
void SysTick_Handler(void)
{
    msTicks++;

    if (countdown_active && currentState == STATE_COUNTDOWN) {
        if (countdown_ms > 0) {
            countdown_ms--;

            // Every 1000 ms, decrement seconds_left and update display
            if ((countdown_ms % 1000u) == 0u && seconds_left > 0) {
                seconds_left--;
                displayUpdateFlag = 1;

                // When seconds reach 0 and time elapsed
                if (seconds_left == 0 && countdown_ms == 0) {
                    timeExpiredFlag = 1;
                    countdown_active = 0;
                }
            }
        }
    }
}

// -----------------------------------------------------------------------------
// PORTB IRQ Handler: button on PTB2 (active low)
// -----------------------------------------------------------------------------
void PORTB_IRQHandler(void)
{
    // Check if PTB2 caused the interrupt
    if (PORTB_ISFR & BUTTON_MASK) {
        PORTB_ISFR = BUTTON_MASK;     // clear interrupt flag by writing 1
        buttonPressedFlag = 1;
    }
}

// -----------------------------------------------------------------------------
// Initialize GPIO: LEDs, button, 7-seg
// -----------------------------------------------------------------------------
void init_gpio(void)
{
    // Enable clocks for PORTB, PORTC, PORTD
    SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK |
                 SIM_SCGC5_PORTC_MASK |
                 SIM_SCGC5_PORTD_MASK;

    // --- Configure PTD0-6 as GPIO (7-seg segments), PTD7 as RED LED ---
    PORTD_PCR0 = 0x100; // MUX=1 (GPIO)
    PORTD_PCR1 = 0x100;
    PORTD_PCR2 = 0x100;
    PORTD_PCR3 = 0x100;
    PORTD_PCR4 = 0x100;
    PORTD_PCR5 = 0x100;
    PORTD_PCR6 = 0x100;
    PORTD_PCR7 = 0x100;

    // Set PTD0-7 as outputs
    GPIOD_PDDR |= 0xFF;

    // Clear all bits initially
    GPIOD_PDOR &= ~0xFF;

    // --- Configure PTC8 as GPIO output for GREEN LED ---
    PORTC_PCR8 = 0x100;     // MUX=1
    GPIOC_PDDR |= GREEN_LED_MASK;
    GPIOC_PDOR &= ~GREEN_LED_MASK; // off

    // --- Configure PTB2 as GPIO input (button, active low), with pull-up & IRQ ---
    // MUX=1 (GPIO), PE=1 (enable pull), PS=1 (pull-up), IRQC=0xA (falling edge)
    PORTB_PCR2 = 0x0A0103;
    GPIOB_PDDR &= ~BUTTON_MASK;  // input

    // Clear any pending interrupt flags
    PORTB_ISFR = BUTTON_MASK;

    // Enable PORTB interrupt in NVIC
    NVIC_EnableIRQ(PORTB_IRQn);
}

// -----------------------------------------------------------------------------
// Initialize SysTick for 1 ms interrupts
// -----------------------------------------------------------------------------
void init_systick(void)
{
    uint32_t reload = (SYSTEM_CLOCK / 1000u) - 1u; // 1 ms
    SysTick->LOAD = reload;
    SysTick->VAL  = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |
                    SysTick_CTRL_TICKINT_Msk   |
                    SysTick_CTRL_ENABLE_Msk;
}

// -----------------------------------------------------------------------------
// Initialize UART1 on PTC3 (RX) and PTC4 (TX) at given baudrate
// -----------------------------------------------------------------------------
void init_uart1(uint32_t baudrate)
{
    // Enable clocks for UART1 and PORTC
    SIM_SCGC4 |= SIM_SCGC4_UART1_MASK;
    SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;

    // Set PTC3 as UART1_RX (ALT3), PTC4 as UART1_TX (ALT3)
    PORTC_PCR3 = 0x300;  // MUX=3
    PORTC_PCR4 = 0x300;  // MUX=3

    // Disable UART1 before config
    UART1_C2 &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK);

    // Baud rate calculation: SBR = system_clock / (16 * baudrate)
    uint16_t sbr = (uint16_t)(SYSTEM_CLOCK / (16u * baudrate));

    UART1_BDH = (UART1_BDH & ~UART_BDH_SBR_MASK) | ((sbr >> 8) & UART_BDH_SBR_MASK);
    UART1_BDL = (uint8_t)(sbr & 0xFF);

    // 8N1, default settings are OK (no parity, 1 stop bit)
    UART1_C1 = 0x00;

    // Enable transmitter and receiver
    UART1_C2 = UART_C2_TE_MASK | UART_C2_RE_MASK;
}

// Non-blocking: returns 1 if a char is ready, 0 otherwise
int UART1_Available(void)
{
    return (UART1_S1 & UART_S1_RDRF_MASK) != 0;
}

// Blocking receive
char UART1_GetChar(void)
{
    while (!(UART1_S1 & UART_S1_RDRF_MASK)) {
        // wait for data
    }
    return UART1_D;
}

// Blocking send
void UART1_PutChar(char c)
{
    while (!(UART1_S1 & UART_S1_TDRE_MASK)) {
        // wait for transmit buffer empty
    }
    UART1_D = c;
}

// -----------------------------------------------------------------------------
// Display a single digit (0–9) on PD0–PD6. PD7 (red LED) is untouched.
// -----------------------------------------------------------------------------
void display_digit(uint8_t digit)
{
    if (digit > 9) digit = 9;

    uint32_t pattern = (uint32_t)(nums[digit] & 0x7F); // bits 0–6 only

    // Clear PD0–PD6, keep PD7 (red LED) as is
    uint32_t current = GPIOD_PDOR & ~0x7F;
    GPIOD_PDOR = current | pattern;
}

// Basic delay using msTicks (not used in core logic, but handy for testing)
void delay_ms(uint32_t ms)
{
    uint32_t start = msTicks;
    while ((msTicks - start) < ms) {
        // busy-wait
    }
}

// -----------------------------------------------------------------------------
// main()
// -----------------------------------------------------------------------------
int main(void)
{
    // Init hardware
    init_gpio();
    init_uart1(9600);
    init_systick();

    // Initial state: LOCKED
    currentState      = STATE_LOCKED;
    seconds_left      = 15;
    countdown_ms      = 0;
    countdown_active  = 0;
    displayUpdateFlag = 1;  // draw initial digit

    // LEDs: RED ON, GREEN OFF in LOCKED
    red_on();
    green_off();

    // Main loop
    while (1) {

        // 1) Handle UART receive from Arduino (unlock signal)
        if (UART1_Available()) {
            char c = UART1_GetChar();

            if (c == '1' && currentState == STATE_LOCKED) {
                // Start 15-second countdown
                currentState      = STATE_COUNTDOWN;
                countdown_ms      = 15000;  // 15 sec
                seconds_left      = 15;
                countdown_active  = 1;
                displayUpdateFlag = 1;

                // LEDs: GREEN ON, RED OFF in COUNTDOWN
                red_off();
                green_on();
            }
        }

        // 2) Handle button press (active low on PTB2)
        if (buttonPressedFlag) {
            buttonPressedFlag = 0;

            // Only relevant in COUNTDOWN and while time remains
            if (currentState == STATE_COUNTDOWN && seconds_left > 0) {
                // Success: user pressed button before timeout.
                // Tell Arduino to LOCK (servo back to 0 deg).
                UART1_PutChar('L');

                // Go back to LOCKED state
                currentState      = STATE_LOCKED;
                countdown_active  = 0;
                countdown_ms      = 0;
                seconds_left      = 15;
                displayUpdateFlag = 1;

                red_on();
                green_off();
            }
        }

        // 3) Handle timeout (15s expired)
        if (timeExpiredFlag) {
            timeExpiredFlag = 0;

            // Time ran out: also tell Arduino to LOCK
            UART1_PutChar('L');

            currentState      = STATE_LOCKED;
            countdown_active  = 0;
            countdown_ms      = 0;
            seconds_left      = 15;
            displayUpdateFlag = 1;

            red_on();
            green_off();
        }

        // 4) Update 7-seg display when needed
        if (displayUpdateFlag) {
            displayUpdateFlag = 0;
            display_digit(seconds_left % 10);  // ones digit only, since we have 1 digit
        }

        // (No blocking delays in main; everything is event/timer driven)
    }
}
