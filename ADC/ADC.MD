# Analog to Digital Conversion
### Transducer 
Converts a non-electric quantity to an electronic quantity (voltage)
* Temperature sensor: temperature -> voltage
* microphone: sound pressure -> voltage

### A/D Converter
Only deals with electric voltage 
* Converts analog quantity (voltage) to digital format

## N-bit A/D Converter
* Takes snapshots of the voltage at regular time intervals (**sampling**) 
* Only $2^n$ distinct levels (e.g., 8-bit -> 256 levels) 
* More bits = finer steps = more precise

## ADC Precision
- The number of **distinguishable** ADC inputs
- E.g., 12-bit A/D converter -> 4096 ($2^{12}$ distinguishable inputs 
