#include "fsl_device_registers.h"

#define BUTTON_MASK (1u << 2)  // PTB2

volatile uint8_t buttonFlag = 0;
volatile uint32_t last_button_ms = 0;
const uint32_t BUTTON_DEBOUNCE_MS = 50;

#define F_BUS                   48000000u
#define FTM0_OVERFLOW_FREQUENCY 50u
#define FTM0_CLK_PRESCALE       7       // ÷128

uint32_t High_Count, Low_Count, Total_Count;

void init_button_irq(void)
{
    SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;

    PORTB_PCR2 = PORT_PCR_MUX(1)
               | PORT_PCR_PE_MASK     // pull enable
               | PORT_PCR_PS_MASK     // pull-up
               | PORT_PCR_IRQC(0xA);  // falling edge

    GPIOB_PDDR &= ~BUTTON_MASK;       // input

    PORTB_ISFR = BUTTON_MASK;         // clear flag
    NVIC_EnableIRQ(PORTB_IRQn);
}

void PORTB_IRQHandler(void)
{
    if (PORTB_ISFR & BUTTON_MASK) {
        PORTB_ISFR = BUTTON_MASK;  // clear IRQ flag

        uint32_t now = msTicks;    // from SysTick 1ms
        if ((now - last_button_ms) >= BUTTON_DEBOUNCE_MS) {
            last_button_ms = now;
            buttonFlag = 1;        // signal main code
        }
    }
}



void Init_PWM_Servo(void)
{
    SIM_SCGC6 |= SIM_SCGC6_FTM0_MASK;
    SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;

    // Example: FTM0_CH3 on PTC4 (check reference manual / your board)
    PORTC_PCR4 = PORT_PCR_MUX(4) | PORT_PCR_DSE_MASK;

    FTM0_MODE |= FTM_MODE_WPDIS_MASK;
    FTM0_MODE &= ~1;          // clear FTMEN if needed

    FTM0_CNT   = 0;
    FTM0_CNTIN = 0;

    uint32_t prescale_div = 1u << FTM0_CLK_PRESCALE;
    FTM0_MOD = (F_BUS / prescale_div) / FTM0_OVERFLOW_FREQUENCY;

    float period_sec = 1.0f / (float)FTM0_OVERFLOW_FREQUENCY; // 0.02 s
    float duty_low   = 0.001f / period_sec;  // 1 ms
    float duty_high  = 0.002f / period_sec;  // 2 ms

    Low_Count   = (uint32_t)(FTM0_MOD * duty_low);
    High_Count  = (uint32_t)(FTM0_MOD * duty_high);
    Total_Count = High_Count - Low_Count;

    // Edge-aligned PWM, high-true on CH3
    FTM0_C3SC = FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;

    // Start at 0°
    FTM0_C3V = Low_Count;

    // System clock, /128, no interrupts required
    FTM0_SC = FTM_SC_CLKS(1) | FTM_SC_PS(FTM0_CLK_PRESCALE);
}

void PWM_Servo_Angle(float angle_deg)
{
    if (angle_deg < 0.0f)   angle_deg = 0.0f;
    if (angle_deg > 90.0f)  angle_deg = 90.0f; // or 180 if you want

    // Map 0..90 → High_Count..Low_Count (or vice versa)
    float scale = High_Count - Total_Count * (angle_deg / 90.0f);
    FTM0_C3V = (uint32_t)scale;
}

int main(void)
{
    init_systick();      // for msTicks
    init_button_irq();   // falling-edge IRQ on button
    Init_PWM_Servo();    // FTM0 PWM for servo

    float current_angle = 0.0f;
    PWM_Servo_Angle(current_angle);  // start at 0°

    while (1) {
        if (buttonFlag) {
            buttonFlag = 0;

            // Example: toggle between 0° and 90°
            if (current_angle < 45.0f) {
                current_angle = 90.0f;
            } else {
                current_angle = 0.0f;
            }
            PWM_Servo_Angle(current_angle);
        }

        // other state machine stuff / UART here...
    }
}
