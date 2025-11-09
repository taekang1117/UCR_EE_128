# Outline
### Interrupts
- Interrupt handling sequence
### Interrupt vector
- How does the system know where to go when an interrupt occurs?
### Coretex-M Nested Vector Interrupt Controller (NVIC)
- How to handle multiple concurrent interrupts?
### Polling vs. Interrupt
- Code examples

## Interrupts
* MCU can "sleep" or run other tasks 
* An interrupt is triggered when drop below a threshold
* MCU jumps into an interrupt handler, reads ADC once, checks the value, and reacts

## Polling 
**Polling**: Periodically checking to see if something is ready 
- e.g., waiting for switch input 

```
while(1) {
  outval = 0;
  inval = GPIOB_PDIR & 0x0C;
  if (inval & 0x8) outval = 0x0F;
  if (inval & 0x4) outval |= 0xF0; 
  GPIOD_PDOR = outval; 
  }
}
```

### Drawbacks
- Need to continually check (difficult to weave checks into complex code
- If the code becomes complex, might poll too slowly
- Can waste a lot of CPU time checking for very infrequent events

