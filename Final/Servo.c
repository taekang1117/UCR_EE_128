#include "fsl_device_registers.h"
#include <stdint.h>

#define F_BUS                   48000000u
#define FTM0_OVERFLOW_FREQUENCY 50u        // 50 Hz (20 ms)
#define FTM0_CLK_PRESCALE       7          // prescaler = 7 => divide by 128

uint32_t High_Count, Low_Count, Total_Count;

// crude blocking delay (1 ms * ms at 48 MHz)
// not precise, but fine for a simple servo test
void delay_ms(uint32_t ms)
{
    for (uint32_t i = 0; i < ms; i++) {
        for (volatile uint32_t j = 0; j < 8000; j++) {
            __NOP();                      // small busy-wait
        }
    }
}

void Init_PWM_Servo(void)
{
    // Enable clocks for FTM0 and PORTC (for PTC4)
    SIM_SCGC6 |= SIM_SCGC6_FTM0_MASK;
    //SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;
    SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;   // if using PTA1
    // PTA1 as FTM0_CH6
       PORTA_PCR1 = PORT_PCR_MUX(3) | PORT_PCR_DSE_MASK;

    // Disable write protection
    FTM0_MODE |= FTM_MODE_WPDIS_MASK;
    FTM0_MODE &= ~1;          // clear FTMEN if needed

    // Reset counter
    FTM0_SC    = 0;
    FTM0_CNT   = 0;
    FTM0_CNTIN = 0;

    // Set MOD for 50 Hz PWM
    uint32_t prescale_div = 1u << FTM0_CLK_PRESCALE;
    FTM0_MOD = ((F_BUS / prescale_div) / FTM0_OVERFLOW_FREQUENCY) - 1;

    // Compute counts for 1 ms and 2 ms pulses
    float period_sec = 1.0f / (float)FTM0_OVERFLOW_FREQUENCY; // 0.02 s
    float duty_low   = 0.001f / period_sec;   // 1 ms  -> 5 duty
    float duty_high  = 0.002f / period_sec;   // 2 ms  -> 10 duty

    Low_Count   = (uint32_t)(FTM0_MOD * duty_low);   // 375
    High_Count  = (uint32_t)(FTM0_MOD * duty_high);  // 750
    Total_Count = High_Count - Low_Count;            // 375 span

    // Edge-aligned, high-true PWM on CH3
    FTM0_C3SC = FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;

    // Start at 0 (1 ms pulse)
    FTM0_C3V = Low_Count;

    // System clock, prescaler /128, no interrupts
    FTM0_SC = FTM_SC_CLKS(1) | FTM_SC_PS(FTM0_CLK_PRESCALE);
}

void PWM_Servo_Angle(float angle_deg)
{
    if (angle_deg < 0.0f)   angle_deg = 0.0f;
    if (angle_deg > 90.0f)  angle_deg = 90.0f;   // change to 180.0f if you want 0–180

    // Map 0..90  Low_Count..High_Count (or reverse if direction is flipped)
    float scale = Low_Count + Total_Count * (angle_deg / 90.0f);
    FTM0_C3V = (uint32_t)scale;
}

int main(void)
{
    Init_PWM_Servo();

    while (1) {
        // 0 degrees
        PWM_Servo_Angle(0.0f);
        delay_ms(1000);          // 1 second

        // 90 degrees
        PWM_Servo_Angle(90.0f);
        delay_ms(1000);          // 1 second
    }
}
