#include <math.h>

const int thermistorPin = A0;  // Analog pin connected to thermistor
const float Rpullup = 100000;  // Pull-up resistor in ohms
const float B = 3950;          // B-value of thermistor (typical for Traxxas)
const float R0 = 100000;       // Thermistor resistance at 25°C
const float T0 = 298.15;       // 25°C in Kelvin

// Convert resistance to Celsius using B-parameter equation
float resistanceToTempC(float R) {
  float T = 1.0 / (1.0/T0 + (1.0/B) * log(R / R0));
  return T - 273.15;
}

void setup() {
  Serial.begin(9600);
}

void loop() {
  int adcValue = analogRead(thermistorPin);
  float Vout = adcValue * (5.0 / 1023.0);             // Convert ADC to voltage
  float Rtherm = Rpullup * (5.0 / Vout - 1.0);       // Calculate thermistor resistance
  float temperatureC = resistanceToTempC(Rtherm);    // Convert to °C

  Serial.print("Resistance (ohms): ");
  Serial.print(Rtherm);
  Serial.print("  |  Temperature (°C): ");
  Serial.println(temperatureC);

  delay(500);  // Update every 0.5 seconds
}
