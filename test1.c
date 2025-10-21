#include "fsl_device_registers.h"
#include <stdint.h>

void software_delay(unsigned long delay) {
    while (delay > 0) delay--;
}

void main(void) {
    SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK; /* Enable Port B Clock Gate Control*/
    SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK; /* Enable Port C Clock Gate Control*/
    SIM_SCGC5 |= SIM_SCGC5_PORTD_MASK; /* Enable Port D Clock Gate Control*/

    PORTB_GPCLR = 0x000C0100; /* Configures Pins 2 and 3 on Port B to be GPIO */
    PORTC_GPCLR = 0x01BF0100; /* Configures Pins 0-5, 7-8 on Port C to be GPIO */
    PORTD_GPCLR = 0x00FF0100; /* Configures Pins 0-7 on Port D to be GPIO */

    GPIOB_PDDR = 0x00000000; /* Configures Pins 2 and 3 of port B as Input */
    GPIOC_PDDR = 0x000001BF; /* Configures Pins 0-5, 7-8 on Port C as Output */
    GPIOD_PDDR = 0x000000FF; /* Configures Pins 0-7 on Port D as Output */


    GPIOB_PDOR = 0x0000000C; /* Set Pins 2 and 3 on Port B to be high voltage */
    GPIOC_PDOR = 0x00000000; /* Set Pins 0-5, 7-8 on Port C to be low voltage */
    GPIOD_PDOR = 0x000000FF; /* Set Pins 0-7 on Port D to be low voltage */

    uint32_t inval = 0;
    const uint32_t CNT_DIR = 0x4;
    const uint32_t ROT_DIR = 0x8;
    unsigned long Delay = 0x100000;
    unsigned long i = 0;
    unsigned long j = 0;
    while (1) {
        if (GPIOB_PDIR & CNT_DIR) {          // if counter (PTB2) is on
            if (GPIOC_PDOR <= 255) { // if counter (PTC0) is less than 255
                GPIOC_PDOR++;
            }
            else {
                GPIOC_PDOR = 0; // reset counter (PTC0) to 0
            }
        }
        else if (!(GPIOB_PDIR & CNT_DIR)) {  // if counter (PTB2) is off
            if (GPIOC_PDOR >= 0) { // if counter (PTC0) is greater than 0
                GPIOC_PDOR--;
            }
            else {
                GPIOC_PDOR = 255; // reset counter (PTC0) to 255
            }
        }
        i = GPIOC_PDOR;
        j = i & 0x1F; // mask to get the last 5 bits of the counter (PTC0)
        i = i << 1;
        i = i & 0xC0; // mask to get the last 2 bits of the counter (PTC0)
        i = i | j; // combine the last 5 bits and the last 2 bits of the counter (PTC0)
        // The bit 5 (PTC6) will be 0.

        if (GPIOB_PDIR & ROT_DIR) {          // if rotator (PTB3) is on
            if (GPIOD_PDOR <= 0x80) { // if counter is not at bit 7 (PTD7)
                GPIOD_PDOR = GPIOD_PDOR << 1; // shift left
            }
            else {
                GPIOD_PDOR = 0x01; // reset counter (PTD0) to 1
            }
        }
        else if (!(GPIOB_PDIR & ROT_DIR)) {  // if rotator (PTB3) is off
            if (GPIOD_PDOR >= 0x01) { // if counter is not at bit 0 (PTD0)
                GPIOD_PDOR = GPIOD_PDOR >> 1; // shift left
            }
            else {
                GPIOD_PDOR = 0x80; // reset counter (PTD7) to 128
            }
        }

        software_delay(Delay);
    }
}
