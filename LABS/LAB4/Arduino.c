void setup() {
  // Configure Pin 9 (OC1A) as output
  pinMode(9, OUTPUT);
  
  // Configure Timer1 for Fast PWM, ICR1 as TOP
  // Fast PWM Mode 14: WGM13=1, WGM12=1, WGM11=1, WGM10=0
  TCCR1A = (1 << COM1A1) | (1 << WGM11);  // Clear OC1A on compare match, Fast PWM mode
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11); // Fast PWM, prescaler = 8
  
  // Calculate TOP value for 245Hz
  // f_PWM = f_CPU / (Prescaler * (1 + TOP))
  // 245 = 16MHz / (8 * (1 + TOP))
  // TOP = (16MHz / (8 * 245)) - 1 = 8162.5 ≈ 8163
  ICR1 = 8163;  // Sets frequency to ~245Hz
  
  // Initialize duty cycle to 50%
  OCR1A = ICR1 / 2;
}

void loop() {
  // ===== OPTION 1: Potentiometer Control =====
  // Uncomment this section to use a potentiometer
  /*
  int potValue = analogRead(A0);
  OCR1A = map(potValue, 0, 1023, 0, ICR1);
  delay(10);
  */
  
  // ===== OPTION 2: Sweep Pattern (for testing) =====
  // Uncomment this section to cycle through duty cycles
  
  // Sweep from 0% to 100%
  for (int duty = 0; duty <= 100; duty += 5) {
    OCR1A = map(duty, 0, 100, 0, ICR1);
    delay(500);  // Hold each duty cycle for 500ms
  }
  
  // Sweep from 100% back to 0%
  for (int duty = 100; duty >= 0; duty -= 5) {
    OCR1A = map(duty, 0, 100, 0, ICR1);
    delay(500);
  }
  
}
