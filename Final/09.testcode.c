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

    // Disable TX/RX before configuration
    UART1_C2 &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK);

    uint16_t sbr = (uint16_t)(SYSTEM_CLOCK / (16u * baudrate));
    UART1_BDH = (UART1_BDH & ~UART_BDH_SBR_MASK) | ((sbr >> 8) & UART_BDH_SBR_MASK);
    UART1_BDL = (uint8_t)(sbr & 0xFF);

    UART1_C1 = 0x00; // 8N1, no parity

    // Clear any status flags by dummy read
    (void)UART1_S1;
    (void)UART1_D;

    // Enable TX, RX, and RX interrupt
    UART1_C2 = UART_C2_TE_MASK | UART_C2_RE_MASK | UART_C2_RIE_MASK;

    // Enable UART1 interrupt in NVIC
    NVIC_ClearPendingIRQ(UART1_RX_TX_IRQn);
    NVIC_EnableIRQ(UART1_RX_TX_IRQn);
}

// Optional: still here if you want them, but not used in main logic
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

// UART1 ISR: RX interrupt-driven
void UART1_RX_TX_IRQHandler(void)
{
    uint8_t status = UART1_S1;

    if (status & UART_S1_RDRF_MASK) {
        char c = UART1_D;       // reading D clears RDRF
        uart_rx_char = c;
        uart_rx_flag = 1;       // signal main loop
    }

    // (Optional) handle errors here if needed (OR, NF, FE, PF)
}

// ---------------- Servo (PTA1, FTM0_CH6) ----------------
void init_servo(void)
{
    // Enable clock for PORTA and FTM0
    SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;
    SIM_SCGC6 |= SIM_SCGC6_FTM0_MASK;

    // Disable write protection for FTM0
    FTM0_MODE |= FTM_MODE_WPDIS_MASK;

    // PTA1 as FTM0_CH6 (ALT3)
    PORTA_PCR1 = PORT_PCR_MUX(3);

    // Disable FTM0 before configuration
    FTM0_SC = 0;
    FTM0_CNTIN = 0;
    FTM0_CNT = 0;

    // Set PWM period: 20 ms (50 Hz)
    FTM0_MOD = SERVO_FTM_MOD;

    // Edge-aligned PWM, high-true pulses on CH6:
    // MSB:MSA = 10 (edge-aligned PWM)
    // ELSB:ELSA = 10 (high-true pulses)
    FTM0_C6SC = 0;
    FTM0_C6SC = FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;

    // Start at 0° (locked)
    FTM0_C6V = SERVO_MIN_PULSE_TICKS;

    // Select system clock, prescaler = 128
    FTM0_SC = FTM_SC_CLKS(1) | FTM_SC_PS(7);
}

static inline void servo_set_locked(void)
{
    // 0 degrees -> ~1.0 ms pulse
    FTM0_C6V = SERVO_MIN_PULSE_TICKS;
}

static inline void servo_set_unlocked(void)
{
    // 90 degrees -> ~1.5 ms pulse
    FTM0_C6V = SERVO_90_PULSE_TICKS;
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

    // Button PTB2, GPIO input, pull-up, FALLING-EDGE interrupt
    PORTB_PCR2 = PORT_PCR_MUX(1)
               | PORT_PCR_PE_MASK     // enable pull
               | PORT_PCR_PS_MASK     // pull-up (so pin idles high)
               | PORT_PCR_IRQC(0xA);  // 0xA = interrupt on falling edge

    GPIOB_PDDR &= ~BUTTON_MASK;       // input
    PORTB_ISFR = BUTTON_MASK;         // clear any pending flag
    NVIC_ClearPendingIRQ(PORTB_IRQn);
    NVIC_EnableIRQ(PORTB_IRQn);       // enable PORTB IRQ in NVIC
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
    init_servo();

    // Start LOCKED: show 9, red ON, green OFF
    currentState      = STATE_LOCKED;
    seconds_left      = 9;
    countdown_ms      = 0;
    countdown_active  = 0;
    displayFlag       = 1;
    timeoutFlag       = 0;
    buttonFlag        = 0;
    uart_rx_flag      = 0;

    red_on();
    green_off();
    servo_set_locked();
    display_digit(9);

    while (1) {

        // --- 1) UART: any byte '1' while LOCKED starts countdown (IRQ-driven) ---
        if (currentState == STATE_LOCKED && uart_rx_flag) {
            uart_rx_flag = 0;          // consume char

            if (uart_rx_char == '1') { // only react to '1'
                currentState      = STATE_COUNTDOWN;
                seconds_left      = 9;
                countdown_ms      = 9000;      // 9 seconds
                countdown_active  = 1;
                displayFlag       = 1;
                timeoutFlag       = 0;

                red_off();
                green_on();
                servo_set_unlocked();
            }
        }

        // --- 2) Button pressed during COUNTDOWN (falling edge) ---
        if (buttonFlag) {
            buttonFlag = 0;

            if (currentState == STATE_COUNTDOWN && seconds_left > 0) {
                // Success: button pressed before timeout
                UART1_PutChar('L');  // tell UNO: locked
                // (Optional) flush RX here if needed

                currentState      = STATE_LOCKED;
                countdown_active  = 0;
                countdown_ms      = 0;
                seconds_left      = 9;
                displayFlag       = 1;

                red_on();
                green_off();
                servo_set_locked();
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
            servo_set_locked();
        }

        // --- 4) Update display when needed ---
        if (displayFlag) {
            displayFlag = 0;
            display_digit(seconds_left % 10);
        }
    }
}
