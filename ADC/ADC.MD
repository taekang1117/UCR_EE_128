# Analog to Digital Conversion
## Transducer 
Converts a non-electric quantity to an electronic quantity (voltage)
* Temperature sensor: temperature -> voltage
* microphone: sound pressure -> voltage

## A/D Converter
Only deals with electric voltage 
* Converts analog quantity (voltage) to digital format

## N-bit A/D Converter
- Takes snapshots of the voltage at regular time intervals (**sampling**) 
- Only $2^n$ distinct levels (e.g., 8-bit -> 256 levels) 
- More bits = finer steps = more precise

## ADC Precision
- The number of **distinguishable** ADC inputs
- E.g., 12-bit A/D converter -> 4096 ($2^{12}$) distinguishable inputs 

## ADC Resolution
- The smallest distinguishable change in input
- E.g., 12-bit ADC with a range of 0 to 5V -> $$\frac{5V}{4096-1} = 1.2mV$$

## Average ADC Conversion Error 
1/2 of resolution: 
$E = \frac{\dfrac{V_{RH} - V_{RL}}{2^{n} - 1}}{2}$

# Registers 
Each A/D module (ADC0 and ADC1) has the following registers: 

## ADC Status and control registers 
### ADCx_SC1A
Used to select the input channel, start a conversion, and configure interrupt and differential mode. 

### ADCx_SC1B
Writing to SC1A starts a conversion. SC1B is used for simultaneous or sequential dual-channel conversion

### ADCx_SC2
Sets the trigger source (software or hardware), voltage reference, and compare mode

### ADC_SC3
Enables hardware averaging, continuous conversion, and calibration mode 

## Configuration registers
### ADCx_CFG1
Sets conversion resolution (8/10/12/16-bit), input clock, and clock divider. 

### ADCx_CFG2
Controls sampling time, high-speed mode, and channel multiplexer settings. 

## ADC Data Result Registers 
### ADCx_RA
Holds result from SC1A

### ADCx_RB
Holds result from SC1B
