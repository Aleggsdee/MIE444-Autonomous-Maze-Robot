#include <Wire.h>
#include <VL53L0X.h>

#define SENSOR_COUNT 7                // Number of VL53L0X sensors
#define TIMING_BUDGET 200000          // Measurement timing budget (us)
float data_to_send[SENSOR_COUNT] = {};  // Array to store readings

VL53L0X sensors[SENSOR_COUNT];

// Resetting I2C addresses for all sensors connected
const int xshut_pins[SENSOR_COUNT] = { 2, 3, 4, 5, 6, 7, 8 };                                 // XSHUT pins for each sensor
const uint8_t sensor_addresses[SENSOR_COUNT] = { 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36 };  // New I2C addresses

// Variables to track samples and timing
unsigned long startTime = 0;
unsigned long currentTime = 0;
unsigned long sampleCounts[SENSOR_COUNT] = {};  // Sample counts for each sensor

// Adjustment factors
const float slope_adjustments[7] = {0.966, 0.966, 1.01051, 0.95511, 0.93791, 1.05296, 0.94384};
const float const_adjustments[7] = {-8.9772, -8.9772, -11.833, -24.426, -4.4361, -41.594, -8.8173};

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Initialize XSHUT pins and set them LOW to turn off sensors
  for (int i = 0; i < SENSOR_COUNT; i++) {
    pinMode(xshut_pins[i], OUTPUT);
    digitalWrite(xshut_pins[i], LOW);  // Keep sensors in reset state
  }

  delay(50);  // Wait for sensors to reset

  // Initialize each sensor individually
  for (int i = 0; i < SENSOR_COUNT; i++) {
    // Turn on the current sensor
    digitalWrite(xshut_pins[i], HIGH);
    delay(50);  // Wait for sensor to boot up

    // Initialize the sensor and assign a new address
    sensors[i].setTimeout(500);
    if (!sensors[i].init()) {
      Serial.print("Failed to detect sensor ");
      Serial.println(i + 1);
      while (1)
        ;
    }
    sensors[i].setAddress(sensor_addresses[i]);            // Set I2C address from sensors array
    sensors[i].setMeasurementTimingBudget(TIMING_BUDGET);  // Set measurement budget
  }

  // Start continuous measurements for each sensor
  for (int i = 0; i < SENSOR_COUNT; i++) {
    sensors[i].startContinuous();

    // Print out key parameters

    // uint32_t timing_budget = sensors[i].getMeasurementTimingBudget();
    // Serial.print("Sensor address 0x");
    // Serial.print(sensor_addresses[i], HEX);
    // Serial.print(" timing budget: ");
    // Serial.println(timing_budget);
  }
  startTime = millis();  // Initialize start time
}

void loop() {
  currentTime = millis();

  // Read and display distances from each sensor
  fetch_sensor_values();

  // Send data over UART
  send_array_over_UART(data_to_send, SENSOR_COUNT);

  // Calculate samples per second every second
  
  // if (currentTime - startTime >= 1000) {
  //   // Calculate and display samples per second for each sensor
  //   print_samples_per_second(currentTime);
  //   startTime = currentTime;
  // }
  
}

void fetch_sensor_values() {
  for (int i = 0; i < SENSOR_COUNT; i++) {
    float distance = (sensors[i].readRangeContinuousMillimeters());  // Get distance measurement from sensor [i]

    float correctedDistance = (distance + const_adjustments[i]) * slope_adjustments[i];
    correctedDistance = correctedDistance * 0.03937007874;
    // Make sure nothing is over the max range
    if (correctedDistance > 47.244094488) {
      correctedDistance = 47.244094488;
    }
    else if (correctedDistance < 0){
      correctedDistance = 0;
    }
    data_to_send[i] = correctedDistance;
    sampleCounts[i]++;  // Increment sample count for this sensor

  }
}

void print_samples_per_second(unsigned long currentTime) {
  Serial.println("Samples/s:");
  for (int i = 0; i < SENSOR_COUNT; i++) {
    float samplesPerSecond = sampleCounts[i] / ((currentTime - startTime) / 1000.0);
    Serial.print("Sensor ");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.println(samplesPerSecond);

    sampleCounts[i] = 0;  // Reset sample count for this sensor
  }
}

void send_array_over_UART(float* array, int length) {
  Serial.write(0xFF);  // Start byte to signal beginning of transmission
  // Send whole array as bytes
  Serial.write((uint8_t *)array, sizeof(float) * length);

}