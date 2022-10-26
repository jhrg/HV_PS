
/**
 * @brief Use the Atmel 328 to drive a high voltage power supply
*/
#include <Arduino.h>

#define LOGGING 1

int32_t last_time = 0;
// uint32_t last_voltage = 0;
int32_t last_error = 0;
int32_t cum_error = 0;

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

  // Use Timer1 for the HV PS control signal
  // Set the timer to Fast PWM. COM1A1:0 --> 1, 0
  // Set the timer for 10-bit resolution. WGM13:0 --> 0, 1, 1, 1
  // Set the timer for 9-bit resolution. WGM13:0 --> 0, 1, 1, 0
  TCCR1A = _BV(COM1A1) | _BV(WGM11);
  // Set the pre-scaler at 1 (62.5 kHz) and the two high-order bits of WGM
  TCCR1B = _BV(WGM12) | _BV(CS10);

  OCR1A = 0x10; // 10-bit resolution --> 0x0000 - 0x03FF

#if LOGGING
  Serial.println(" voltage, error, delta_t, cum_error, rate_error, correction, OCR1A");
#endif

  last_time = millis();
}

#define SET_POINT 455 // ~ 200v

#define Kp 0.01
#define Ki 0
#define Kd 1

#define ADC_NOISE 3

#define SAMPLE_PERIOD 10 // ms

void loop()
{
  int32_t now = millis();

  if ((now - last_time >= SAMPLE_PERIOD))
  {
    // delta time
    int32_t delta_t = now - last_time;

    // It takes about 112 uS to make one call to analogRead()
    int32_t voltage = analogRead(A0); // 0 - 1023 -> 0 - 5v;
    int32_t error = SET_POINT - voltage;

    // old
    // int32_t correction = error * Kp;
    // correction = abs(correction);

    cum_error += error * delta_t;
    int32_t rate_error = (error - last_error) / delta_t;

    int32_t correction = Kp * error + Ki * cum_error + Kd * rate_error;

    last_time = now;
    last_error = error;

#if LOGGING
    char str[64];
    snprintf(str, 64, "%ld, %ld, %ld, %ld, %ld, %ld, %d", voltage, error, delta_t, cum_error, rate_error, correction, OCR1A);
    Serial.println(str);
#endif

    OCR1A += correction;
  }
#if 0
    // Cap the OCR0A value between 0x010 and 0x1F0 to avoid extremes
    if (voltage > SET_POINT + ADC_NOISE)
    {
      if (OCR1A > 0x0010) {
        // OCR1A -= 1;
        OCR1A -= (correction > 1) ? correction : 1;
      }
    }
    else if (voltage < SET_POINT - ADC_NOISE)
    {
      if (OCR1A < 0x01F0) {
        // OCR1A += 1;
        OCR1A += (correction > 1) ? correction : 1;
      }
    }
  }
#endif
}