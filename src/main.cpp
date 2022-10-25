
/**
 * @brief Use the Atmel 328 to drive a high voltage power supply
*/
#include <Arduino.h>

#if 0
#include <PID_v1.h>

double voltage;
double pwm_duty_cycle = 102.0; // 20% DC
double ref_voltage = 455.0;
int Kp = 1.0;
int Ki = 0.05;
int Kd = 0.25;

PID HV_PID(&voltage, &pwm_duty_cycle, &ref_voltage, Kp, Ki, Kd, DIRECT);
#endif

void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);
  Serial.println("boot");

#if 0
  // Configure the PID controller for the voltage level
  HV_PID.SetMode(AUTOMATIC);
  HV_PID.SetOutputLimits(102.0, 358.0); // 20% to 70%
  HV_PID.SetSampleTime(100);            // 100ms/sample
#endif

  // Set all ports to be outputs
  // Initialize all I/O pins to output, then one for the analog input
  DDRD = B11111111;
  DDRC = B00111111;
  DDRB = B00111111;

  pinMode(A0, INPUT);
  // pinMode(A1, INPUT);

  // Use Timer1 for the HV PS control signal
  // Set the timer to Fast PWM. COM1A1:0 --> 1, 0
  // Set the timer for 10-bit resolution. WGM13:0 --> 0, 1, 1, 1
  // Set the timer for 9-bit resolution. WGM13:0 --> 0, 1, 1, 0
  TCCR1A = _BV(COM1A1) | _BV(WGM11);
  // Set the pre-scaler at 1 (62.5 kHz) and the two high-order bits of WGM
  TCCR1B = _BV(WGM12) | _BV(CS10);

  OCR1A = 0x10; // 10-bit resolution --> 0x0000 - 0x03FF

  Serial.println("voltage, v_delta, correction");
}

// TODO Remove double set_point_pc = 44.7; // percent of 1024 counts
#define SET_POINT 455
//#define Kp_THRESHOLD 20
//#define Kp_THRESHOLD_NEG -20
////#define Kp 0.75
//#define DEFAULT_INCR 2
//#define DEFAULT_DECR -2
#define ADC_NOISE 3
//#define ADC_NOISE_NEG -3

#define SAMPLE_PERIOD 10 // ms
uint32_t last_update = 0;
uint32_t last_voltage = 0;

#if 0
int32_t
compute_pwn_delta(uint32_t voltage)
{
  if (millis() - last_update < SAMPLE_PERIOD)
    return 0;

  // Not perfect, but close enough and avoids saving the return values
  last_update = millis();

  int32_t delta = SET_POINT - voltage;
  int32_t abs_delta = (delta >= 0) ? delta : -delta;

  Serial.print(delta);
  Serial.print(", ");
  Serial.print(abs_delta);
  Serial.print(", ");

  if (abs_delta > ADC_NOISE)
  {
    if (abs_delta > Kp_THRESHOLD)
    {
      return int32_t(delta * Kp);
    }
    else
    {
      if (delta >= 0)
        return DEFAULT_INCR;
      else
        return DEFAULT_DECR;
    }
  }

  return 0;
}
#endif

void loop() {
  // put your main code here, to run repeatedly:

  // It takes about 112 uS to make one call to analogRead()
  uint32_t voltage = analogRead(A0); // 0 - 1023 -> 0 - 5v;

#if 0
  int32_t pwm_delta = compute_pwn_delta(voltage);
  Serial.print(pwm_delta);
  Serial.print(", ");

  if (pwm_delta != 0)
  {

    Serial.print(voltage);

    OCR1A += pwm_delta;

    Serial.print(", ");
    Serial.println(OCR1A);
  }
#endif

#if 0
  if (HV_PID.Compute())
  {
    Serial.print("voltage: ");
    Serial.print(voltage);
    Serial.print(", time: ");
    Serial.println(analog_read_time);

    uint16_t timer_1_counts = uint16_t(pwm_duty_cycle);
    // clamp OCR1A changes to 10 counts
    if (timer_1_counts > OCR1A + 10)
      OCR1A += 10;
    else if (timer_1_counts < OCR1A - 10)
      OCR1A -= 10;
    else
      OCR1A = timer_1_counts;

    Serial.print("  voltage: ");
    Serial.print(voltage);
    Serial.print("  ");
    Serial.print("setPoint: ");
    Serial.print(ref_voltage);
    Serial.print("  ");
    Serial.print("pwn duty cycle: ");
    Serial.println(pwm_duty_cycle);
  }
#endif

#if 1
  
  //Serial.println(voltage_delta);
  if ((millis() - last_update >= SAMPLE_PERIOD))
  {
    // Cap the OCR0A value between 0x010 and 0x1F0 to avoid extremes
    int32_t v_delta = SET_POINT - voltage;
    int32_t correction = v_delta * 0.04;
    correction = abs(correction);

#if 0
    char str[64];
    snprintf(str, 64, "%ld, %ld, %ld, %d", voltage, v_delta, correction, OCR1A);
    Serial.println(str);
#endif

    if (voltage > SET_POINT + ADC_NOISE)
    {
      if (OCR1A > 0x0010) {
        OCR1A -= (correction > 1) ? correction : 1;
      }
    }
    else if (voltage < SET_POINT - ADC_NOISE)
    {
      if (OCR1A < 0x01F0) {
        //OCR1A += 1;
        OCR1A += (correction > 1) ? correction : 1;
      }
    }

    last_update = millis();
    last_voltage = voltage;
  }
#endif
}