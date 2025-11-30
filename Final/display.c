#include "fsl_device_registers.h"
#include <stdint.h>

// For common-anode:
// 0 = ON, 1 = OFF on each segment (PD0–PD6)

int main(void)
{
    // Enable Port D clock
    SIM_SCGC5 |= SIM_SCGC5_PORTD_MASK;

    // Set PD0–PD7 to GPIO
    PORTD_PCR0 = PORT_PCR_MUX(1);
    PORTD_PCR1 = PORT_PCR_MUX(1);
    PORTD_PCR2 = PORT_PCR_MUX(1);
    PORTD_PCR3 = PORT_PCR_MUX(1);
    PORTD_PCR4 = PORT_PCR_MUX(1);
    PORTD_PCR5 = PORT_PCR_MUX(1);
    PORTD_PCR6 = PORT_PCR_MUX(1);
    PORTD_PCR7 = PORT_PCR_MUX(1);

    // PD0–PD7 as outputs
    GPIOD_PDDR |= 0xFF;

    // Clear everything first
    GPIOD_PDOR &= ~0xFF;

    // For common-anode 7-seg:
    // Show "8" = all segments ON (a–g), so PD0–PD6 must be LOW.
    // PD7 is the red LED: set HIGH to turn it ON.

    // Segments: PD0..PD6 = 0 (already done by clear), PD7 = 1
    GPIOD_PSOR = (1u << 7); // turn red LED ON

    while (1) {
        // do nothing, just hold outputs steady
    }
}
