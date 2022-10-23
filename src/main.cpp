
/**
 * @brief Use the Atmel 328 to drive a high voltage power supply
*/
#include <Arduino.h>

void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);
  Serial.println("boot");

  // Set all ports to be outputs
  // Initialize all I/O pins to output, then one for the analog input
  DDRD = B11111111;
  DDRC = B00111111;
  DDRB = B00111111;

  pinMode(A0, INPUT);
  pinMode(A1, INPUT);

  // Use Timer1 for the HV PS control signal
  // Set the timer to Fast PWM. COM1A1:0 --> 1, 0
  // Set the timer for 10-bit resolution. WGM13:0 --> 0, 1, 1, 1
  TCCR1A = _BV(COM1A1) | _BV(WGM11) | _BV(WGM10);
  // Set the pre-scaler at 1 (62.5 kHz) and the two high-order bits of WGM
  TCCR1B = _BV(WGM12) | _BV(CS10);

  OCR1A = 0x10; // 10-bit resolution --> 0x0000 - 0x03FF
}

void loop() {
  // put your main code here, to run repeatedly:

  Serial.println(OCR1A);

  uint32_t analog_read_start = micros();
  int v = analogRead(A0); // 0 - 1023 -> 0 - 5v
  int ref = analogRead(A1);

  Serial.print("diff: ");
  Serial.print(v - ref);
  Serial.print(", time: ");
  Serial.println(micros() - analog_read_start);

  // TODO Look up: ANALOG_COMP_vect ANALOG_COMP_vect_num
  // Cap the OCR0A value between 0x10 and 0xF0 to avoid extremes
  if (v > ref)
  {
    if (OCR1A > 0x0010)
      OCR1A -= 1;
  }
  else
  {
    if (OCR1A < 0x03F0)
      OCR1A += 1;
  }

  uint32_t t1 = micros();
  t1 += 100000;  // 10ms wait
  while (t1 > micros())
  {
    // Serial.println(t1 - t2);
  }
}