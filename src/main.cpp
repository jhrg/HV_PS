
/**
 * @brief Use the Atmel 328 to drive a high voltage power supply
 */
#include <Arduino.h>

#define PID_LOGGING 0
#define SIMPLE_LOGGING 1

#define ADC_NOISE 3      // counts
#define SAMPLE_PERIOD 10 // ms
#define SET_POINT 455    // ~ 200v

#if TIMER_1
// works with 14.8 mA, and 1 mA, sort of...
#define Kp 0.04
#define Ki 0.00005
#define Kd 10
#define REGISTER OCR1A
#else
#define Kp 0.04
#define Ki 0.000005
#define Kd 10.0
#define REGISTER OCR2B
#define THRESHOLD 4     // ADC_NOISE + 1
#endif

int32_t last_time = 0;
float last_error = 0;
float cum_error = 0;

int32_t pid_correction(int32_t delta_t, int32_t voltage) {
    // delta time
    //int32_t delta_t = (now - last_time);

    float error = (float)(SET_POINT - voltage);

    cum_error += error * (float)delta_t;
    float rate_error = (error - last_error) / (float)delta_t;

    int32_t correction = Kp * error + Ki * cum_error + Kd * rate_error;

    // last_time is updated by the caller
    last_error = error;

#if PID_LOGGING
    char str[128];
    #define frac(f) ((int)(f*100)%100)
    snprintf(str, 64, "%ld, %ld, %ld, %ld, %ld, %ld, %d", 
            voltage, (long)error, delta_t, (long)cum_error, (long)rate_error, correction, OCR2B);
    Serial.println(str);
#endif

    return correction;
}

#if 1
#define Kp1 0.01

int32_t P_correction(int32_t voltage) {
    int32_t correction = (SET_POINT - voltage) * Kp1;
    // Cap the OCR0A value between 0x010 and 0x1F0 to avoid extremes
    if ((voltage > SET_POINT + ADC_NOISE) && (OCR2B > 0x0010)) {
        return (correction < -1) ? correction : -1;
    } else if ((voltage < SET_POINT - ADC_NOISE) && (OCR2B < 0x01F0)) {
        return (correction > 1) ? correction : 1;
    } else {
        return 0;
    }
}

int32_t two_factor_correction(int32_t voltage) {
    int32_t error = SET_POINT - voltage;

    // Cap the OCR0A value between 0x010 and 0x1F0 to avoid extremes
    if ((voltage > SET_POINT + ADC_NOISE) && (OCR2B > 0x10)) {
        return (error < -50) ? -20 : -1;
    } else if ((voltage < SET_POINT - ADC_NOISE) && (OCR2B < 0x80)) {
        return 1;
    } else {
        return 0;
    }
}

int32_t simple_correction(int32_t voltage) {
    // Cap the OCR0A value between 0x010 and 0x1F0 to avoid extremes
    if ((voltage > SET_POINT + ADC_NOISE) && (OCR2B > 0x0010)) {
        return -1;
    } else if ((voltage < SET_POINT - ADC_NOISE) && (OCR2B < 0x80)) {
        return 1;
    } else {
        return 0;
    }
}
#endif

void setup() {
    // put your setup code here, to run once:

    Serial.begin(115200);
    Serial.println("boot");

    // Set all ports to be outputs
    // Initialize all I/O pins to output, then one for the analog input
    DDRD = B11111111;
    DDRC = B00111111;
    DDRB = B00111111;

    pinMode(A0, INPUT);

#if TIMER_1 // Pin 9
    // Use Timer1 for the HV PS control signal
    // Set the timer to Fast PWM. COM1A1:0 --> 1, 0
    // Set the timer for 9-bit resolution. WGM13:0 --> 0, 1, 1, 0
    TCCR1A = _BV(COM1A1) | _BV(WGM11);
    // Set the pre-scaler at 1 (31.25 kHz) and WGM bit 2
    TCCR1B = _BV(WGM12) | _BV(CS10);

    // Start out with low voltage
    // OCR1A is Arduino Pin 9
    OCR1A = 0x010; // 9-bit resolution --> 0x0000 - 0x01FF
#else              // TIMER_2, Pin 3
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
#endif

#if PID_LOGGING
    Serial.println("voltage, error, delta_t, cum_error, rate_error, correction, OCR2B");
#elif SIMPLE_LOGGING
    Serial.println("voltage, OCR2B");
#endif

    last_time = millis();
}

void loop() {
    int32_t now = millis();

    if ((now - last_time >= SAMPLE_PERIOD)) {
        int32_t voltage = analogRead(A0); // 0 - 1023 -> 0 - 5v;

        // OCR1A += simple_correction(voltage);
        // OCR1A += two_factor_correction(voltage);
        // OCR1A += P_correction(voltage);
#if TIMER_1
        // OCR1A += simple_correction(voltage);
        // OCR1A += two_factor_correction(voltage);
        // OCR1A += P_correction(voltage);
        // OCR1A += pid_correction(now, voltage);
#else
        if (voltage < THRESHOLD)
            REGISTER = 0;
        else {
            REGISTER += simple_correction(voltage);
            // REGISTER += two_factor_correction(voltage);
            // REGISTER += P_correction(voltage);
            //REGISTER += pid_correction(now - last_time, voltage);
        }
#endif

        last_time = millis();

#if SIMPLE_LOGGING
        char str[128];
        snprintf(str, 64, "%ld, %d", voltage, OCR2B);
        Serial.println(str);
#endif
    }
}