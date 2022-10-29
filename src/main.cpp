
/**
 * @brief Use the Atmel 328 to drive a high voltage power supply
*/
#include <Arduino.h>

#define PID_LOGGING 0
#define SIMPLE_LOGGING 0

#define ADC_NOISE 3      //counts
#define SAMPLE_PERIOD 10 // ms
#define SET_POINT 455    // ~ 200v
#define ORC1A_MIN 0x010
#define ORC1A_MAX 0x1F0

#if 0
// these work for no load, but don't respond quickly with a load
#define Kp 0.009
#define Ki 0.0000009
#define Kd 10
#endif

#if 1
// works with 14.8 mA, and 1 mA, sort of...
#define Kp 0.04
#define Ki 0.00005
#define Kd 10
#endif

int32_t last_time = 0;
int32_t last_error = 0;
int32_t cum_error = 0;

int32_t pid_correction(int32_t now, int32_t voltage) {
    // delta time
    int32_t delta_t = now - last_time;

    int32_t error = SET_POINT - voltage;

    // old
    // int32_t correction = error * Kp;
    // correction = abs(correction);

    cum_error += error * delta_t;
    int32_t rate_error = (error - last_error) / delta_t;

    int32_t correction = Kp * error + Ki * cum_error + Kd * rate_error;

    //last_time = now;
    last_error = error;

#if PID_LOGGING
    char str[128];
    snprintf(str, 64, "%ld, %ld, %ld, %ld, %ld, %ld, %d", voltage, error, delta_t, cum_error, rate_error, correction, OCR1A);
    Serial.println(str);
#endif

    return correction;
}

int32_t simple_correction(int32_t voltage) {
    // Cap the OCR0A value between 0x010 and 0x1F0 to avoid extremes
    if ((voltage > SET_POINT + ADC_NOISE) && (OCR1A > 0x0010)) {
        return -1;
    } else if ((voltage < SET_POINT - ADC_NOISE) && (OCR1A < 0x01F0)) {
        return 1;
    } else {
        return 0;
    }
}

int32_t two_factor_correction(int32_t voltage) {
    int32_t error = SET_POINT - voltage;

    // Cap the OCR0A value between 0x010 and 0x1F0 to avoid extremes
    if ((voltage > SET_POINT + ADC_NOISE) && (OCR1A > 0x0010)) {
        return (error < -20) ? -10 : -1;
    } else if ((voltage < SET_POINT - ADC_NOISE) && (OCR1A < 0x01F0)) {
        return (error > 20) ? 10 : 1;
    } else {
        return 0;
    }
}

#define Kp1 0.07

int32_t P_correction(int32_t voltage) {
    int32_t correction = (SET_POINT - voltage) * Kp1;
    // Cap the OCR0A value between 0x010 and 0x1F0 to avoid extremes
    if ((voltage > SET_POINT + ADC_NOISE) && (OCR1A > 0x0010)) {
        return (correction < -1) ? correction : -1;
    } else if ((voltage < SET_POINT - ADC_NOISE) && (OCR1A < 0x01F0)) {
        return (correction > 1) ? correction : 1;
    } else {
        return 0;
    }
}

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

    // Use Timer1 for the HV PS control signal
    // Set the timer to Fast PWM. COM1A1:0 --> 1, 0
    // Set the timer for 9-bit resolution. WGM13:0 --> 0, 1, 1, 0
    TCCR1A = _BV(COM1A1) | _BV(WGM11);
    // Set the pre-scaler at 1 (31.25 kHz) and WGM bit 2
    TCCR1B = _BV(WGM12) | _BV(CS10);

    // Start out with low voltage
    OCR1A = 0x010; // 9-bit resolution --> 0x0000 - 0x01FF

#if PID_LOGGING
    Serial.println("voltage, error, delta_t, cum_error, rate_error, correction, OCR1A");
#elif SIMPLE_LOGGING
    Serial.println("voltage, OCR1A");
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
        OCR1A += pid_correction(now, voltage);

        last_time = millis();

#if SIMPLE_LOGGING
        char str[128];
        snprintf(str, 64, "%ld, %d", voltage, OCR1A);
        Serial.println(str);
#endif
    }
}