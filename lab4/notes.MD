# Key Registers 
SC: Set clock source, prescaler, overflow interrupt

CNT: Current counter value

MOD: Maximum count (defines PWM period)

CnSc: Per-channel control (input capture / output compare / PWM mode)

CnV: Channel compare value (defines duty cycle or trigger point)

STATUS: Flags for events (e.g., overflow, channel match)


# Timer Counter (FTMx_CNT)
16 bit non stop hardware counter. Up to 0xFFFF (Must be enabled first)
```
unsigned int cnt;
cnt = FTM0_CNT; // read counter
FTM0_CNT = 0; // reset counter
```

# Enable Timer Counter
Steps
1. Configure clock gating for the appropriate FTM
2. Enable the FTM
3. Set the timer counter limit
4. Choose clock source and prescaler 

