// Wheel Encoder RPM and Speed Measurement

// === CONFIGURABLE PARAMETERS ===
const int encoderPin = 2;                     // Interrupt pin connected to encoder
const int pulsesPerRevolution = 3;           // Number of encoder pulses per wheel revolution
const float wheelDiameterMeters = 0.105;     // Diameter of the wheel in meters

// === INTERNAL VARIABLES ===
volatile unsigned long lastPulseTime = 0;     // Timestamp of last pulse (microseconds)
volatile unsigned long pulsePeriod = 0;       // Time between two pulses (microseconds)

float rpm = 0;    // Revolutions per minute
float speed = 0;  // Linear speed in meters per second (m/s)

void setup() {
  Serial.begin(9600);

  pinMode(encoderPin, INPUT_PULLUP);  // Set encoder pin with internal pull-up resistor
  lastPulseTime = micros();           // Initialize timestamp

  attachInterrupt(digitalPinToInterrupt(encoderPin), pulseISR, RISING); // Trigger on rising edge
}

void loop() {
  // Safely copy volatile variables
  noInterrupts();
  unsigned long period = pulsePeriod;
  unsigned long lastPulse = lastPulseTime;
  interrupts();

  unsigned long currentTime = micros();
  bool signalTimedOut = (currentTime - lastPulse) > 1000000; // Timeout after 1 second

  if (!signalTimedOut && period > 0 && period < 1000000) {
    float pulsesPerSecond = 1000000.0 / period;
    float revsPerSecond = pulsesPerSecond / pulsesPerRevolution;

    rpm = revsPerSecond * 60.0;  // Convert to RPM
    speed = revsPerSecond * PI * wheelDiameterMeters;  // v = ω * r
  } else {
    rpm = 0;
    speed = 0;
  }

  // Output data
  Serial.print("RPM: ");
  Serial.print(rpm, 2);
  Serial.print(" | Speed: ");
  Serial.print(speed, 3);
  Serial.println(" m/s");

  delay(100); // Update at 10Hz
}

// === INTERRUPT SERVICE ROUTINE ===
void pulseISR() {
  unsigned long currentTime = micros();
  pulsePeriod = currentTime - lastPulseTime;
  lastPulseTime = currentTime;
}
