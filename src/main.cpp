
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

#include <PID_v1.h>
#include <PID_AutoTune_v0.h>


byte ATuneModeRemember = 2;
double input = 80, output = 50, setpoint = SET_POINT;
double kp = 2, ki = 0.5, kd = 2;

double kpmodel = 1.5, taup = 100, theta[50];
double outputStart = 5;
double aTuneStep = 50, aTuneNoise = 1, aTuneStartValue = 100;
unsigned int aTuneLookBack = 20;

boolean tuning = true;
unsigned long serialTime = 0;

PID myPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);

PID_ATune aTune(&input, &output);
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

void AutoTuneHelper(boolean start) {
    if (start)
        ATuneModeRemember = myPID.GetMode();
    else
        myPID.SetMode(ATuneModeRemember);
}

void changeAutoTune() {
    if (!tuning) {
        // Set the output to the desired starting frequency.
        output = aTuneStartValue;
        aTune.SetNoiseBand(aTuneNoise);
        aTune.SetOutputStep(aTuneStep);
        aTune.SetLookbackSec((int)aTuneLookBack);
        AutoTuneHelper(true);
        tuning = true;
    } else {  // cancel autotune
        aTune.Cancel();
        tuning = false;
        AutoTuneHelper(false);
    }
}

void SerialSend() {
    Serial.print("setpoint: ");
    Serial.print(setpoint);
    Serial.print(" ");
    Serial.print("input: ");
    Serial.print(input);
    Serial.print(" ");
    Serial.print("output: ");
    Serial.print(output);
    Serial.print(" ");
    if (tuning) {
        Serial.println("tuning mode");
    } else {
        Serial.print("kp: ");
        Serial.print(myPID.GetKp());
        Serial.print(" ");
        Serial.print("ki: ");
        Serial.print(myPID.GetKi());
        Serial.print(" ");
        Serial.print("kd: ");
        Serial.print(myPID.GetKd());
        Serial.println();
    }
}

void SerialReceive() {
    if (Serial.available()) {
        char b = Serial.read();
        Serial.flush();
        if ((b == '1' && !tuning) || (b != '1' && tuning)) changeAutoTune();
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

#if PID_LOGGING
    Serial.println("voltage, error, delta_t, cum_error, rate_error, correction, OCR2B");
#elif SIMPLE_LOGGING
    Serial.println("voltage, OCR2B");
#endif

    input = analogRead(A0);
    // setpoint = SET_POINT;
    myPID.SetOutputLimits(10, 200);
    myPID.SetSampleTime(SAMPLE_PERIOD);
    myPID.SetMode(AUTOMATIC);

    if (tuning) {
        tuning = false;
        changeAutoTune();
        tuning = true;
    }

    last_time = millis();
}

void loop() {
    
    input = analogRead(A0);

    if (tuning) {
        byte val = (aTune.Runtime());
        if (val != 0) {
            tuning = false;
        }
        if (!tuning) {  // we're done, set the tuning parameters
            kp = aTune.GetKp();
            ki = aTune.GetKi();
            kd = aTune.GetKd();
            myPID.SetTunings(kp, ki, kd);
            AutoTuneHelper(false);
        }
    } else {
        myPID.Compute();
    }

    analogWrite(3, output);

    // send-receive with processing if it's time
    if (millis() > serialTime) {
        SerialReceive();
        SerialSend();
        serialTime += 500;
    }

    char str[128];
    snprintf(str, 64, "%ld, %d", (long)input, OCR2B);
    Serial.println(str);
}