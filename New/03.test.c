#include "fsl_device_registers.h"
#include <stdint.h>
#include <stdbool.h>

#define SYSTEM_CLOCK (48000000u)

#define LEFT_BTN_MASK   (1u << 2)   // PTB2  - left button (lock)
#define RIGHT_BTN_MASK  (1u << 3)   // PTB3  - right button (unlock)
#define GREEN_LED_MASK  (1u << 10)  // PTB10 - green LED
#define RED_LED_MASK    (1u << 11)  // PTB11 - red LED

typedef enum {
    STATE_LOCKED = 0,
    STATE_UNLOCKED
} LockState;

volatile LockState g_lockState = STATE_UNLOCKED;

static inline void green_on(void)   { GPIOB_PCOR = GREEN_LED_MASK; }  // drive low
static inline void green_off(void)  { GPIOB_PSOR = GREEN_LED_MASK; }  // drive high
static inline void red_on(void)     { GPIOB_PCOR = RED_LED_MASK; }
static inline void red_off(void)    { GPIOB_PSOR = RED_LED_MASK; }
static inline void update_leds_from_state(void)
{
    if (g_lockState == STATE_LOCKED) {
        // locked: red ON, green OFF
        red_on();
        green_off();
    } else {
        // unlocked: green ON, red OFF
        green_on();
        red_off();
    }
}

// ---------------- GPIO init ----------------
void init_gpio(void)
{
    SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;

    PORTB_PCR10 = PORT_PCR_MUX(1);  // GPIO
    PORTB_PCR11 = PORT_PCR_MUX(1);  // GPIO
    GPIOB_PDDR |= (GREEN_LED_MASK | RED_LED_MASK);   // outputs

    GPIOB_PSOR = (GREEN_LED_MASK | RED_LED_MASK);
    GPIOB_PCOR = 0; // doesn't hurt

    PORTB_PCR2 = PORT_PCR_MUX(1)     // GPIO
               | PORT_PCR_PE_MASK    // enable pull resistor
               | PORT_PCR_PS_MASK    // pull-up
               | PORT_PCR_IRQC(0xA); // interrupt on falling edge

    PORTB_PCR3 = PORT_PCR_MUX(1)
               | PORT_PCR_PE_MASK
               | PORT_PCR_PS_MASK
               | PORT_PCR_IRQC(0xA); // falling edge

    GPIOB_PDDR &= ~(LEFT_BTN_MASK | RIGHT_BTN_MASK); // inputs

    // Clear any pending interrupts on PORTB before enabling
    PORTB_ISFR = 0xFFFFFFFFu;

    // Enable PORTB interrupt in NVIC
    NVIC_EnableIRQ(PORTB_IRQn);
}

// ---------------- PORTB interrupt handler ----------------
void PORTB_IRQHandler(void)
{
    uint32_t flags = PORTB_ISFR;  // latch which pins triggered
    PORTB_ISFR = flags;           // clear them by writing back

    uint32_t inputs = GPIOB_PDIR; // read current pin levels

    // LEFT button (lock) pressed? (active-low: pin reads 0)
    if (flags & LEFT_BTN_MASK) {
        if ((inputs & LEFT_BTN_MASK) == 0) {
            g_lockState = STATE_LOCKED;
            update_leds_from_state();
        }
    }

    // RIGHT button (unlock) pressed?
    if (flags & RIGHT_BTN_MASK) {
        if ((inputs & RIGHT_BTN_MASK) == 0) {
            g_lockState = STATE_UNLOCKED;
            update_leds_from_state();
        }
    }
}

// ---------------- main ----------------
int main(void)
{
    init_gpio();

    // Default state: UNLOCKED (you wanted green ON by default)
    g_lockState = STATE_UNLOCKED;
    update_leds_from_state();

    // Enable global interrupts
    __enable_irq();

    while (1) {
        // No button polling needed. Everything happens in the ISR.
        __WFI();   // Wait For Interrupt (low-power sleep)
    }
}
