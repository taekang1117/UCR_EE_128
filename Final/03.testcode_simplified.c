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
    STATE_COUNTDOWN
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

    if (countdown_active && currentState == STATE_COUNTDOWN) {
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

// ---------------- Button interrupt (PTB2, active low) ----------------
void PORTB_IRQHandler(void)
{
    if (PORTB_ISFR & BUTTON_MASK) {
        PORTB_ISFR = BUTTON_MASK;
        buttonFlag = 1;
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

    UART1_C2 &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK); // disable

    uint16_t sbr = (uint16_t)(SYSTEM_CLOCK / (16u * baudrate));
    UART1_BDH = (UART1_BDH & ~UART_BDH_SBR_MASK) | ((sbr >> 8) & UART_BDH_SBR_MASK);
    UART1_BDL = (uint8_t)(sbr & 0xFF);

    UART1_C1 = 0x00; // 8N1
    UART1_C2 = UART_C2_TE_MASK | UART_C2_RE_MASK; // enable
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

void UART1_PutChar(char c)
{
    while (!(UART1_S1 & UART_S1_TDRE_MASK)) { }
    UART1_D = c;
}

// ---------------- GPIO init ----------------
void init_gpio(void)
{
    SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK
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

    // Button PTB2, GPIO input, pull-up, falling-edge interrupt
    PORTB_PCR2 = PORT_PCR_MUX(1)
               | PORT_PCR_PE_MASK
               | PORT_PCR_PS_MASK
               | PORT_PCR_IRQC(0xA); // falling edge
    GPIOB_PDDR &= ~BUTTON_MASK;
    PORTB_ISFR = BUTTON_MASK;
    NVIC_EnableIRQ(PORTB_IRQn);
}

// ---------------- SysTick init ----------------
void init_systick(void)
{
    uint32_t reload = (SYSTEM_CLOCK / 1000u) - 1u;
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

    // Start LOCKED: show 9, red ON, green OFF
    currentState      = STATE_LOCKED;
    seconds_left      = 9;
    countdown_ms      = 0;
    countdown_active  = 0;
    displayFlag       = 1;
    timeoutFlag       = 0;
    buttonFlag        = 0;

    red_on();
    green_off();

    while (1) {

        // --- 1) UART: any byte while LOCKED starts countdown ---
        if (currentState == STATE_LOCKED && UART1_Available()) {
            (void)UART1_GetChar();  // discard value, just acknowledge
            currentState      = STATE_COUNTDOWN;
            seconds_left      = 9;
            countdown_ms      = 9000;      // 9 seconds
            countdown_active  = 1;
            displayFlag       = 1;
            timeoutFlag       = 0;

            red_off();
            green_on();
        }

        // --- 2) Button pressed during COUNTDOWN ---
        if (buttonFlag) {
            buttonFlag = 0;

            if (currentState == STATE_COUNTDOWN && seconds_left > 0) {
                // Success: button pressed before timeout
                UART1_PutChar('L');  // tell UNO to lock servo

                currentState      = STATE_LOCKED;
                countdown_active  = 0;
                countdown_ms      = 0;
                seconds_left      = 9;
                displayFlag       = 1;

                red_on();
                green_off();
            }
        }

        // --- 3) Timer expired ---
        if (timeoutFlag) {
            timeoutFlag = 0;

            UART1_PutChar('L');  // time out: lock

            currentState      = STATE_LOCKED;
            countdown_active  = 0;
            countdown_ms      = 0;
            seconds_left      = 9;
            displayFlag       = 1;

            red_on();
            green_off();
        }

        // --- 4) Update display when needed ---
        if (displayFlag) {
            displayFlag = 0;
            display_digit(seconds_left % 10);
        }
    }
}
