# Enable Timer Counter
1. Configure clock gating for the appropriate FTM
2. Enable the FTM
3. Set the timer counter limit
4. Choose clock source and prescaler 
* Clock gating configuration
* To enable the FTM, we need to configure FTMx_MODE[2:0]


## Key Registers 
SC: Set clock source, prescaler, overflow interrupt

CNT: Current counter value

MOD: Maximum count (defines PWM period)

CnSc: Per-channel control (input capture / output compare / PWM mode)

CnV: Channel compare value (defines duty cycle or trigger point)

STATUS: Flags for events (e.g., overflow, channel match)


# FTMx_CNT [15:0] (Timer Counter)
16 bit non stop hardware counter. Up to 0xFFFF (Must be enabled first)
```
unsigned int cnt;
cnt = FTM0_CNT; // read counter
FTM0_CNT = 0; // reset counter
```

## FTM Clock Gating
* FTM0, FTM1, FTM2 -> SIM_SCGC6
```
// Example
SIM_SCGC6 |= SIM_SCGC6_FTM0_MASK; // Enable FTM0
```

* FTM2, FTM3 -> SIM_SCGC3
```
// Example
SIM_SCGC3 |= SIM_SCGC3_FTM3_MASK;
// Enable FTM3
```

# FTMx_MODE (Enable FTM)
To enable FTM, FTMx_MODE bit 0 needs to be set. <br>
* FTMx_MODE has a write protection flag in Bit 2  (default is 1: write protection disabled) 

```
FTM3_MODE = 0x5; // Enable FTM3 & Disable WR Protection
FTM3_MODE |= 0x1; // Enable FTM3
```

# FTMx_MOD (Set Counter Limit)
* FTMx_MOD: Modulo register -> Maximum counter value
We will use 0xFFFF for simplicity -> `FTM3_MOD = 0xFFFF;`

# FTMx_SC [4:3] (Timer Clock Source)
* The FlexTimer moudle can have various clock sources, but we will be using the system clock (default 20MHz)
* To select this we can configure `FTMx_SC[4:3]`

00 No clock selected. This in effect disables the FTM counter
01 System clock
10 Fixed frequency clock
11 External clock

## Prescaler
* FRDM K64F system clock speed: 21MHz by default
* 1 clock cycle = 1 second / 21 MHz = 47.6 nsec
* How often does the timer counter (FTMx_CNT) overflow? <br>  47.6 nsec * 65536 = 3.12 msec

This may be too fast for our purpose. So we scale down the timer clock speed by dividing the clock source
with the **prescale factor**

# FTMx_SC [2:0] (Prescale factor)
The **prescale factor** for the main timer is selected by `FTMx_SC[2:0]`


000 Divide by 1 <br>
001 Divide by 2 <br>
010 Divide by 4 <br>
011 Divide by 8 <br>
100 Divide by 16 <br>
101 Divide by 32 <br>
110 Divide by 64 <br>
111 Divide by 128

# Ex1. Reading timer counter
```
unsigned int timer_val;

SIM_SCGC3 |= SIM_SCGC3_FTM3_MASK; // FTM3 clock gating on
FTM3_MODE = 0x5; // Enable FTM3
FTM3_MOD = 0xFFFF; // Max counter value
// Set prescale factor and clock source (FTMx_SC[4:0])
FTM3_SC = 0x0F; // 0x0F => b01111 => system clock / 128 

timer_val = FTM3_CNT; // Get current counter value 
```

# Overflow
Sets overflow flag when count reaches the maximum value <br>
* can generate interrupt on counter overflow `FTMx_CNT`

### Discussion Question 1
What happens when a 16-bit hardware timer counter overflows? <br>
A. It clears all other registers <br>
B. It stops counting  <br>
C. It sends a reset signal to MCU <br>
D. It wraps back to 0 and set an overflow flag <br>

The ANS is D. <br>
A 16-bit hardware timer can count values from `0` to `65535 (0xFFFF)` <br>
When it reaches its maximum value and increments one more time: 
The counter overflows. 
* It resets back to 0 automatically (wrap-around behavior).
* The overflow flag (TOF or similar) in the timer status register is set to 1.
* This flag can trigger an interrupt if the overflow interrupt is enabled.
* This mechanism lets the MCU know that the timer has completed a full cycle, which is often used for timekeeping, PWM generation, or event measurement.
