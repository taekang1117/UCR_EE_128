#include "fsl_device_registers.h"
#include <stdint.h>
#include <stdbool.h>

#define SYSTEM_CLOCK (48000000u)

typedef enum {
    STATE_LOCKED = 0,
    STATE_UNLOCKED
} LockState;

// --- Globals ---
// FIX: Set initial state to UNLOCKED as requested ("default as unlock")
volatile LockState currentState = STATE_UNLOCKED;

// --- GPIO masks ---
#define LOCK_BUTTON_MASK    (1u << 2)   // PTB2 - LOCK button (Active Low)
#define UNLOCK_BUTTON_MASK  (1u << 3)   // PTB3 - UNLOCK button (Active Low)
#define GREEN_LED_MASK      (1u << 10)  // PTB10 (UNLOCKED indicator, assume Active High)
#define RED_LED_MASK        (1u << 11)  // PTB11 (LOCKED indicator, assume Active High)

// LED control functions (Assumed Active-High: Set bit = ON, Clear bit = OFF)
static inline void red_on(void)    { GPIOB_PSOR = RED_LED_MASK; }
static inline void red_off(void)   { GPIOB_PCOR = RED_LED_MASK; }
static inline void green_on(void)  { GPIOB_PSOR = GREEN_LED_MASK; }
static inline void green_off(void) { GPIOB_PCOR = GREEN_LED_MASK; }

// Set locked state: Red ON, Green OFF
void set_locked(void)
{
    if (currentState != STATE_LOCKED) {
        currentState = STATE_LOCKED;
        red_on();
        green_off();
    }
}

// Set unlocked state: Green ON, Red OFF
void set_unlocked(void)
{
    if (currentState != STATE_UNLOCKED) {
        currentState = STATE_UNLOCKED;
        green_on();
        red_off();
    }
}

// ---------------- Button interrupt handler ----------------
void PORTB_IRQHandler(void)
{
    uint32_t isfr = PORTB_ISFR;
    bool handled = false;

    // --- Critical Section: Debouncing and State Change ---

    // Lock button (PTB2)
    if (isfr & LOCK_BUTTON_MASK) {
        // Clear interrupt flag
        PORTB_ISFR = LOCK_BUTTON_MASK;

        // Check if button is actually pressed (low)
        if (!(GPIOB_PDIR & LOCK_BUTTON_MASK)) {
            // FIX: Add small, blocking delay (inefficient, but necessary for simple debouncing)
            for(volatile uint32_t i = 0; i < 10000; i++) { __NOP(); }

            // Check again after delay
            if (!(GPIOB_PDIR & LOCK_BUTTON_MASK)) {
                set_locked();
                handled = true;
            }
        }
    }

    // Unlock button (PTB3) - Only check if LOCK wasn't just handled
    if (!handled && (isfr & UNLOCK_BUTTON_MASK)) {
        // Clear interrupt flag
        PORTB_ISFR = UNLOCK_BUTTON_MASK;

        // Check if button is actually pressed (low)
        if (!(GPIOB_PDIR & UNLOCK_BUTTON_MASK)) {
            // FIX: Add small, blocking delay for debouncing
            for(volatile uint32_t i = 0; i < 10000; i++) { __NOP(); }

            // Check again after delay
            if (!(GPIOB_PDIR & UNLOCK_BUTTON_MASK)) {
                set_unlocked();
                handled = true;
            }
        }
    }
}

// ---------------- GPIO init ----------------
void init_gpio(void)
{
    // Enable PORTB clock
    SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;

    // Configure LEDs (PTB10 and PTB11) as GPIO outputs
    PORTB_PCR10 = PORT_PCR_MUX(1);  // Green LED
    PORTB_PCR11 = PORT_PCR_MUX(1);  // Red LED
    GPIOB_PDDR |= (GREEN_LED_MASK | RED_LED_MASK);  // Set as outputs
    GPIOB_PDOR &= ~(GREEN_LED_MASK | RED_LED_MASK); // Initially off

    // Configure Lock Button (PTB2) - Active Low, Falling-edge interrupt
    PORTB_PCR2 = PORT_PCR_MUX(1)
               | PORT_PCR_PE_MASK         // Pull Enable
               | PORT_PCR_PS_MASK         // Pull-Up Select
               | PORT_PCR_IRQC(0xA);      // Interrupt on falling edge
    GPIOB_PDDR &= ~LOCK_BUTTON_MASK;      // Set as input

    // Configure Unlock Button (PTB3) - Active Low, Falling-edge interrupt
    PORTB_PCR3 = PORT_PCR_MUX(1)
               | PORT_PCR_PE_MASK         // Pull Enable
               | PORT_PCR_PS_MASK         // Pull-Up Select
               | PORT_PCR_IRQC(0xA);      // Interrupt on falling edge
    GPIOB_PDDR &= ~UNLOCK_BUTTON_MASK;    // Set as input

    // Clear any pending interrupts before enabling NVIC
    PORTB_ISFR = (LOCK_BUTTON_MASK | UNLOCK_BUTTON_MASK);

    // Enable PORTB interrupt in NVIC
    NVIC_ClearPendingIRQ(PORTB_IRQn);
    NVIC_SetPriority(PORTB_IRQn, 2); // Set priority (0 = highest)
    NVIC_EnableIRQ(PORTB_IRQn);
}

// ---------------- main ----------------
int main(void)
{
    // Disable interrupts during initialization
    __disable_irq();

    init_gpio();

    // Set initial state to UNLOCKED (Green LED on)
    set_unlocked();

    // Enable interrupts
    __enable_irq();

    // Main loop does nothing - everything handled by interrupts
    while (1) {
        // Enter low-power mode (wait for interrupt)
        __WFI();
    }
}
