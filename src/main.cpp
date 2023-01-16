
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

    cli();

    TCCR1A = 0;  // set entire TCCR2A register to 0
    TCCR1B = 0;  // same for TCCR0B
    TCNT1 = 0;   // initialize counter value to 0

    // Use Timer1 for the HV PS control signal
    // Set the timer to Fast PWM. COM1A1:0 --> 1, 0
    // Set the timer for 10-bit resolution. WGM13:0 --> 0, 1, 1, 1
    // Set the timer for 9-bit resolution. WGM13:0 --> 0, 1, 1, 0
    TCCR1A = _BV(COM1B1) | _BV(WGM11);
    // Set the pre-scaler at 1 (62.5 kHz) and the two high-order bits of WGM
    TCCR1B = _BV(WGM12) | _BV(CS10);

    OCR1B = 0xFF;  // 9-bit resolution --> 0x0000 - 0x01FF

    sei();

    input = analogRead(HV_PS_INPUT);
    myPID.SetOutputLimits(10, 150);
    myPID.SetSampleTime(SAMPLE_PERIOD);
    myPID.SetMode(AUTOMATIC);  // This turns on the PID; MANUAL mode turns it off
}

/** 
 * @brief The simplest input reader. Always reads a value
 */
bool read_input(double *in) {
    *in = analogRead(HV_PS_INPUT);
    return true;
}

void loop() {
    // input = analogRead(HV_PS_INPUT);
#if PID_DIAGNOSTIC
    PORTD |= _BV(PORTD6);
#endif
    myPID.Compute(read_input);
#if PID_DIAGNOSTIC
    PORTD &= ~_BV(PORTD6);
#endif
    OCR1B = (unsigned char)output;
    // analogWrite(3, output);
}