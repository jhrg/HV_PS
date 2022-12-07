
/**
 * @brief Use the Atmel 328 to drive a high voltage power supply
 */
#include <Arduino.h>

//#define ADC_NOISE 3      // counts
#define HV_PS_INPUT A0
#define SAMPLE_PERIOD 10 // ms
#define SET_POINT 455    // ~ 200v

#include <PID_v1.h>

double input = 80, output = 50, setpoint = SET_POINT;
double kp = 0.8, ki = 0.4, kd = 0.0;

PID myPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);

void setup() {
    // put your setup code here, to run once:

    Serial.begin(115200);
    Serial.println("boot");

    // Set all ports to be outputs
    // Initialize all I/O pins to output, then one for the analog input
    DDRD = B11111111;
    DDRC = B00111111;
    DDRB = B00111111;

    pinMode(HV_PS_INPUT, INPUT);

    // Use Timer2 for the HV PS control signal
    // COM2B 1:0 --> 1, 0 (non-inverted)
    // WGM22:0 --> 1 (0, 0, 1) (phase-correct PWM, 0xFF top)
    TCCR2A = 0;
    TCCR2A = _BV(COM2B1) | _BV(WGM20);
    // Set the pre-scaler at 1 (62.5 kHz)
    TCCR2B = _BV(CS20);

    // Start out with low voltage
    // OCR2B is Arduino Pin 3
    OCR2B = 0x010; // 0-bit resolution --> 0x00 - 0xFF

    input = analogRead(HV_PS_INPUT);
    // setpoint = SET_POINT;
    myPID.SetOutputLimits(10, 150);
    myPID.SetSampleTime(SAMPLE_PERIOD);
    myPID.SetMode(AUTOMATIC);  // This turns on the PID; MANUAL mode turns it off
}

void loop() {
    input = analogRead(HV_PS_INPUT);
#if PID_DIAGNOSTIC
    PORTD |= _BV(PORTD6);
#endif
    myPID.Compute();
#if PID_DIAGNOSTIC
    PORTD &= ~_BV(PORTD6);
#endif
    OCR2B = (unsigned char)output;
    // analogWrite(3, output);
}