#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"          // For checkForBeat() method
#include "spo2_algorithm.h"     // For SpO2 & heart rate calculation

// Create sensor object
MAX30105 particleSensor;

// Buffer length for SpO2/heart rate algorithm
#define BUFFER_LENGTH 100

// Arrays to store red/IR data for SpO2/heart rate calculation
uint32_t irBuffer[BUFFER_LENGTH];
uint32_t redBuffer[BUFFER_LENGTH];

// Output variables for SpO2/heart rate algorithm
int32_t spO2;            // SpO2 value
int8_t  validSpO2;       // SpO2 validity flag
int32_t hrSpO2;          // Heart rate (from SpO2 algorithm)
int8_t  validHrSpO2;     // Heart rate validity flag (from SpO2 algorithm)

// Variables for the simpler heartRate.h method
const byte RATE_SIZE = 4;  // Number of samples to average for BPM
byte rates[RATE_SIZE];     
byte rateSpot = 0;
long lastBeat = 0;         // Track the timing of the last detected beat
float beatsPerMinute = 0;  
int   beatAvg       = 0;   // Rolling average of BPM

void setup()
{
  Serial.begin(115200);
  Serial.println("Initializing MAX30102...");

  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST))  // 400kHz I2C
  {
    Serial.println("MAX30102 not found. Check wiring and power.");
    while (1);
  }

  // Configure MAX30102
  // You can adjust these settings according to your needs:
  byte ledBrightness = 80;   // LED brightness
  byte sampleAverage = 2;    // Number of samples to average
  byte ledMode      = 2;     // 2 = Red + IR
  byte sampleRate   = 100;   // Sample rate in Hz
  int  pulseWidth   = 411;   // Pulse width (69, 118, 215, or 411)
  int  adcRange     = 4096;  // ADC range (2048, 4096, 8192, 16384)

  particleSensor.setup(ledBrightness, sampleAverage, ledMode,
                       sampleRate, pulseWidth, adcRange);

  // Enable on-board temperature reading
  particleSensor.enableDIETEMPRDY();

  // Let user know to place their finger
  Serial.println("Place your finger on the sensor...");
  delay(2000);

  // Collect the initial 100 samples for the SpO2/HR algorithm
  Serial.println("Collecting initial 100 samples...");
  for (byte i = 0; i < BUFFER_LENGTH; i++)
  {
    unsigned long startTime = millis();
    // Wait for data to be available
    while (!particleSensor.available())
    {
      particleSensor.check();
      if (millis() - startTime > 2000) // 2-second timeout
      {
        Serial.println("No data available! Check sensor.");
        return;
      }
    }

    // Get raw readings
    redBuffer[i] = particleSensor.getRed();
    irBuffer[i]  = particleSensor.getIR();

    // We can also do the simpler beat detection simultaneously
    checkBeatAndComputeBPM(irBuffer[i]);

    particleSensor.nextSample();
    
    // Print raw data (optional for initial read)
    Serial.print("Initial red=");
    Serial.print(redBuffer[i]);
    Serial.print(", ir=");
    Serial.println(irBuffer[i]);
  }

  // Compute initial HR & SpO2 from the first 100 samples
  Serial.println("Calculating initial HR & SpO2...");
  maxim_heart_rate_and_oxygen_saturation(
    irBuffer,  BUFFER_LENGTH,
    redBuffer, &spO2, &validSpO2,
    &hrSpO2, &validHrSpO2
  );
}

void loop()
{
  // We keep iterating forever in this loop
  while (1)
  {
    // Shift the last 75 samples in the buffer to the front
    for (byte i = 25; i < BUFFER_LENGTH; i++)
    {
      redBuffer[i - 25] = redBuffer[i];
      irBuffer[i - 25]  = irBuffer[i];
    }

    // Collect 25 new samples
    for (byte i = 75; i < BUFFER_LENGTH; i++)
    {
      unsigned long startTime = millis();
      // Wait for new data
      while (!particleSensor.available())
      {
        particleSensor.check();
        if (millis() - startTime > 2000) // 2-second timeout
        {
          Serial.println("No data available! Restarting...");
          return; // Return to start of loop() => re-initialize
        }
      }

      // Read new samples
      redBuffer[i] = particleSensor.getRed();
      irBuffer[i]  = particleSensor.getIR();

      // Perform the simpler beat detection on this new IR reading
      checkBeatAndComputeBPM(irBuffer[i]);

      // Read temperature in this loop. Doing it once per sample is frequent,
      // but it demonstrates the temperature reading:
      float temperatureC = particleSensor.readTemperature();
      float temperatureF = particleSensor.readTemperatureF();

      // Print out combined data
      Serial.print("red=");
      Serial.print(redBuffer[i]);
      Serial.print(", ir=");
      Serial.print(irBuffer[i]);

      // Heart rate & SpO2 from advanced spo2_algorithm
      Serial.print(", HR_SpO2=");
      Serial.print(hrSpO2);
      Serial.print(", HRvalid=");
      Serial.print(validHrSpO2);

      Serial.print(", SPO2=");
      Serial.print(spO2);
      Serial.print(", SPO2Valid=");
      Serial.print(validSpO2);

      // Heart rate from simpler heartRate.h method
      Serial.print(", BPM_simpler=");
      Serial.print(beatsPerMinute, 1);
      Serial.print(", BPM_avg=");
      Serial.print(beatAvg);

      // If IR is too low, it usually means no finger is present
      if (irBuffer[i] < 50000) {
        Serial.print(" (No finger?)");
      }

      // Temperature readings
      Serial.print(", TempC=");
      Serial.print(temperatureC, 2);
      Serial.print(", TempF=");
      Serial.print(temperatureF, 2);

      Serial.println();

      // Move to the next sample
      particleSensor.nextSample();
    }

    // After collecting 25 fresh samples (and having 75 old + 25 new = 100),
    // recalc HR & SpO2
    maxim_heart_rate_and_oxygen_saturation(
      irBuffer,  BUFFER_LENGTH,
      redBuffer, &spO2, &validSpO2,
      &hrSpO2, &validHrSpO2
    );
  }
}

/**
 * @brief Checks for a beat using the simpler heartRate.h approach,
 *        then updates global beatsPerMinute & beatAvg.
 *
 * @param irValue The latest IR reading from the sensor
 */
void checkBeatAndComputeBPM(long irValue)
{
  // The checkForBeat() function returns true if a beat is detected
  if (checkForBeat(irValue) == true)
  {
    long delta = millis() - lastBeat;
    lastBeat = millis();

    // Calculate instantaneous BPM
    beatsPerMinute = 60.0 / (delta / 1000.0);

    // Simple sanity check on BPM
    if (beatsPerMinute < 255 && beatsPerMinute > 20)
    {
      rates[rateSpot++] = (byte)beatsPerMinute; // Store this reading
      rateSpot %= RATE_SIZE;

      // Compute average BPM over the last RATE_SIZE readings
      beatAvg = 0;
      for (byte x = 0; x < RATE_SIZE; x++)
      {
        beatAvg += rates[x];
      }
      beatAvg /= RATE_SIZE;
    }
  }
}
