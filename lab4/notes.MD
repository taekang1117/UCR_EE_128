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
* How often does the timer counter (FTMx_CNT) overflow?
- 47.6 nsec * 65536 = 3.12 msec

This may be too fast for our purpose. So we scale down the timer clock speed by dividing the clock source
with the *prescale factor*




