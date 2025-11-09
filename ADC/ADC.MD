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
