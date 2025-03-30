/*
 * TMF8801 Advanced ToF Sensor Calibration Script for Teensy 4.1
 * 
 * This script calibrates the lower range (20mm-100mm) of the DFRobot TMF8801 ToF sensor
 * in 5mm increments with linear interpolation between calibration points.
 * 
 * Uses the official DFRobot_TMF8x01 library: https://github.com/DFRobot/DFRobot_TMF8x01
 * 
 * Features:
 * - Guided calibration procedure via serial monitor
 * - Multiple samples at each calibration point for accuracy
 * - Statistical filtering to remove outliers
 * - Linear interpolation between calibration points
 * - EEPROM storage option for calibration data
 * - Testing mode to verify calibration
 */

#include <Wire.h>
#include <EEPROM.h>
#include "DFRobot_TMF8x01.h"
      
#define EN       -1                      //EN pin of of TMF8x01 module is floating, not used in this demo
#define INT      -1                      //INT pin of of TMF8x01 module is floating, not used in this demo

DFRobot_TMF8801 tof(/*enPin =*/EN,/*intPin=*/INT);

// Calibration parameters
const int MIN_DISTANCE = 20;    // Starting calibration distance in mm
const int MAX_DISTANCE = 100;   // Ending calibration distance in mm
const int STEP_SIZE = 5;        // Increment size in mm
const int NUM_SAMPLES = 30;     // Number of samples to take at each position - increased for better accuracy
const int SAMPLE_DELAY = 100;   // Delay between samples in ms
const float OUTLIER_THRESHOLD = 2.0;  // Standard deviations for outlier detection

// EEPROM settings
const int EEPROM_CALIBRATION_ADDR = 0;
const int EEPROM_SIGNATURE = 0xCAB1;  // Signature to check if calibration data exists

// Arrays to store calibration data
const int NUM_CALIBRATION_POINTS = ((MAX_DISTANCE - MIN_DISTANCE) / STEP_SIZE) + 1;
int targetDistances[NUM_CALIBRATION_POINTS];
float measuredDistances[NUM_CALIBRATION_POINTS];
float calibrationOffsets[NUM_CALIBRATION_POINTS];

// State tracking
enum CalibrationState {
  STATE_INIT,
  STATE_CALIBRATING,
  STATE_COMPLETED,
  STATE_TESTING
};

CalibrationState currentState = STATE_INIT;
int currentCalibrationPoint = 0;

// Function declarations
void waitForSpaceKey();
String readSerialCommand();
void calibrateCurrentPoint();
void calculateCalibrationResults();
void displayCalibrationCode();
void testCalibration();
float applyCalibratedDistance(float measuredDistance);
bool checkExistingCalibrationData();
void saveCalibrationToEEPROM();
void loadCalibrationFromEEPROM();

void setup() {
  Serial.begin(115200);
  
  // Wait for serial to connect
  while (!Serial) {
    delay(10);
  }
  
  Serial.println("\n\nTMF8801 Advanced ToF Sensor Calibration");
  Serial.println("=======================================");
  
  // Initialize I2C
  Wire.begin();
  
  // Initialize the sensor
  while (tof.begin() != 0) {
    Serial.println("Failed to initialize TMF8801 sensor!");
    Serial.println("Please check your connection");
    delay(1000);
  }
  
  Serial.println("TMF8801 sensor initialized successfully!");
  
  // Get I2C address (for information only)
  uint8_t addr = tof.getI2CAddress();
  Serial.print("Sensor I2C address: 0x");
  Serial.println(addr, HEX);
  
  // Configure the sensor for measurement mode with no calibration
  tof.startMeasurement(tof.eModeNoCalib);
  Serial.println("Sensor started in measurement mode with no calibration");
  
  // Initialize the target distances array
  for (int i = 0; i < NUM_CALIBRATION_POINTS; i++) {
    targetDistances[i] = MIN_DISTANCE + (i * STEP_SIZE);
  }
  
  // Check if we have existing calibration data
  if (checkExistingCalibrationData()) {
    Serial.println("\nExisting calibration data found in EEPROM.");
    Serial.println("What would you like to do?");
    Serial.println("  1. Use existing calibration data and proceed to testing");
    Serial.println("  2. Perform new calibration");
    Serial.println("Enter your choice (1 or 2)...");
    
    while (true) {
      if (Serial.available()) {
        char choice = Serial.read();
        if (choice == '1') {
          loadCalibrationFromEEPROM();
          currentState = STATE_TESTING;
          break;
        } else if (choice == '2') {
          currentState = STATE_INIT;
          break;
        }
      }
      delay(10);
    }
  }
  
  if (currentState == STATE_INIT) {
    Serial.println("\nThis script will help you calibrate the sensor from 20mm to 100mm in 5mm increments.");
    Serial.println("You will need a precise way to position the sensor at specific distances.");
    Serial.println("\nCommands:");
    Serial.println("  Press SPACE + ENTER to take a measurement");
    Serial.println("  Type 'restart' + ENTER to restart current calibration point");
    Serial.println("  Type 'skip' + ENTER to skip to the next calibration point");
    Serial.println("\nPress SPACE followed by ENTER to start the calibration process...");
    
    waitForSpaceKey();
    
    // Start the calibration procedure
    currentState = STATE_CALIBRATING;
  }
}

void loop() {
  switch (currentState) {
    case STATE_CALIBRATING:
      if (currentCalibrationPoint < NUM_CALIBRATION_POINTS) {
        calibrateCurrentPoint();
      } else {
        // Calibration completed
        calculateCalibrationResults();
        saveCalibrationToEEPROM();
        currentState = STATE_COMPLETED;
      }
      break;
      
    case STATE_COMPLETED:
      Serial.println("\nCalibration completed! What would you like to do next?");
      Serial.println("  1. Test the calibration");
      Serial.println("  2. Exit");
      Serial.println("Enter your choice (1 or 2)...");
      
      while (true) {
        if (Serial.available()) {
          char choice = Serial.read();
          if (choice == '1') {
            currentState = STATE_TESTING;
            break;
          } else if (choice == '2') {
            Serial.println("\nExiting calibration. You can now apply these offsets to your sensor readings.");
            displayCalibrationCode();  // Display the code again before exiting
            while (1) { delay(1000); }  // Effectively exit the program
            break;
          }
        }
        delay(10);
      }
      break;
      
    case STATE_TESTING:
      testCalibration();
      break;
      
    default:
      // Should not get here
      break;
  }
}

void waitForSpaceKey() {
  while (true) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c == ' ' || c == '\n') return;
    }
    delay(10);
  }
}

String readSerialCommand() {
  String command = "";
  while (true) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c == '\n' || c == '\r') {
        if (command.length() > 0) {
          return command;
        }
      } else {
        command += c;
      }
    }
    delay(10);
  }
}

void calibrateCurrentPoint() {
  int targetDistance = targetDistances[currentCalibrationPoint];
  
  Serial.print("\n> Position the sensor at exactly ");
  Serial.print(targetDistance);
  Serial.println("mm from the target");
  Serial.println("  Press SPACE followed by ENTER when ready...");
  
  String cmd = readSerialCommand();
  if (cmd == "skip") {
    Serial.println("  Skipping this calibration point.");
    // Use linear extrapolation if possible, otherwise just use target as measured
    if (currentCalibrationPoint > 0 && currentCalibrationPoint < NUM_CALIBRATION_POINTS - 1) {
      // We have points before and after, so interpolate
      float slope = (measuredDistances[currentCalibrationPoint + 1] - measuredDistances[currentCalibrationPoint - 1]) / 
                     (targetDistances[currentCalibrationPoint + 1] - targetDistances[currentCalibrationPoint - 1]);
      measuredDistances[currentCalibrationPoint] = measuredDistances[currentCalibrationPoint - 1] + 
                                                  slope * (targetDistances[currentCalibrationPoint] - targetDistances[currentCalibrationPoint - 1]);
    } else {
      // Just use the target as the measured (zero offset)
      measuredDistances[currentCalibrationPoint] = targetDistance;
    }
    currentCalibrationPoint++;
    return;
  } else if (cmd == "restart") {
    return;  // Simply restart this point
  }
  
  Serial.print("  Taking ");
  Serial.print(NUM_SAMPLES);
  Serial.println(" measurements...");
  
  // Arrays to store sample data
  float samples[NUM_SAMPLES];
  int validSamples = 0;
  
  // Take multiple samples at this position
  for (int j = 0; j < NUM_SAMPLES; j++) {
    // Wait for a new measurement to be available
    int retryCount = 0;
    while (!tof.isDataReady() && retryCount < 100) {  // Added retry limit
      delay(5);
      retryCount++;
    }
    
    if (retryCount >= 100) {
      Serial.println("    Timeout waiting for sensor data");
      j--;  // Retry this sample
      continue;
    }
    
    // Get distance measurement in millimeters
    uint16_t distance_mm = tof.getDistance_mm();
    
    // Check if the reading is valid (TMF8801 returns 0xFFFF for invalid readings)
    if (distance_mm != 0xFFFF) {
      samples[validSamples] = distance_mm;
      
      Serial.print("    Sample ");
      Serial.print(j + 1);
      Serial.print(": ");
      Serial.print(distance_mm);
      Serial.println(" mm");
      
      validSamples++;
    } else {
      Serial.print("    Sample ");
      Serial.print(j + 1);
      Serial.println(": Invalid reading, skipping");
      j--; // Retry this sample
    }
    
    delay(SAMPLE_DELAY);
  }
  
  // Check if we got any valid samples
  if (validSamples == 0) {
    Serial.println("    No valid samples obtained at this distance!");
    Serial.println("    Please check sensor position and try again.");
    return;  // Don't proceed, retry this point
  }
  
  // Calculate mean and standard deviation to filter outliers
  float totalDistance = 0;
  for (int j = 0; j < validSamples; j++) {
    totalDistance += samples[j];
  }
  float mean = totalDistance / validSamples;
  
  float variance = 0;
  for (int j = 0; j < validSamples; j++) {
    variance += pow(samples[j] - mean, 2);
  }
  variance /= validSamples;
  float stdDev = sqrt(variance);
  
  // Filter out outliers and recalculate mean
  float filteredTotal = 0;
  int filteredSamples = 0;
  
  for (int j = 0; j < validSamples; j++) {
    if (abs(samples[j] - mean) <= OUTLIER_THRESHOLD * stdDev) {
      filteredTotal += samples[j];
      filteredSamples++;
    } else {
      Serial.print("    Excluding outlier: ");
      Serial.println(samples[j]);
    }
  }
  
  // If we filtered out too many points, just use the raw average
  if (filteredSamples < validSamples / 2) {
    Serial.println("    Too many outliers, using raw average.");
    measuredDistances[currentCalibrationPoint] = mean;
  } else {
    measuredDistances[currentCalibrationPoint] = filteredTotal / filteredSamples;
  }
  
  Serial.print("  Final measured distance: ");
  Serial.print(measuredDistances[currentCalibrationPoint]);
  Serial.println(" mm");
  
  currentCalibrationPoint++;
}

void calculateCalibrationResults() {
  Serial.println("\n\nCalibration Results");
  Serial.println("===================");
  Serial.println("Target (mm), Measured (mm), Offset (mm)");
  
  for (int i = 0; i < NUM_CALIBRATION_POINTS; i++) {
    calibrationOffsets[i] = targetDistances[i] - measuredDistances[i];
    
    Serial.print(targetDistances[i]);
    Serial.print(", ");
    Serial.print(measuredDistances[i]);
    Serial.print(", ");
    Serial.println(calibrationOffsets[i]);
  }
  
  // Display the code in a separate function so we can call it again when exiting
  displayCalibrationCode();
}

void displayCalibrationCode() {
  Serial.println("\nCalibration Map (for use in your application)");
  Serial.println("===========================================");
  Serial.println("// Apply this calibration map to your TMF8801 sensor readings");
  Serial.println("const int NUM_CALIBRATION_POINTS = " + String(NUM_CALIBRATION_POINTS) + ";");
  Serial.println("const int calibrationDistances[NUM_CALIBRATION_POINTS] = {");
  
  for (int i = 0; i < NUM_CALIBRATION_POINTS; i++) {
    Serial.print("  ");
    Serial.print(targetDistances[i]);
    if (i < NUM_CALIBRATION_POINTS - 1) {
      Serial.println(",");
    } else {
      Serial.println();
    }
  }
  
  Serial.println("};");
  Serial.println("const float measuredDistances[NUM_CALIBRATION_POINTS] = {");
  
  for (int i = 0; i < NUM_CALIBRATION_POINTS; i++) {
    Serial.print("  ");
    Serial.print(measuredDistances[i], 2); // 2 decimal places
    if (i < NUM_CALIBRATION_POINTS - 1) {
      Serial.println(",");
    } else {
      Serial.println();
    }
  }
  
  Serial.println("};");
  
  Serial.println("const float calibrationOffsets[NUM_CALIBRATION_POINTS] = {");
  
  for (int i = 0; i < NUM_CALIBRATION_POINTS; i++) {
    Serial.print("  ");
    Serial.print(calibrationOffsets[i], 2); // 2 decimal places
    if (i < NUM_CALIBRATION_POINTS - 1) {
      Serial.println(",");
    } else {
      Serial.println();
    }
  }
  
  Serial.println("};");
  
  Serial.println("\n// Example function to apply calibration with linear interpolation:");
  Serial.println("float applyCalibration(float measuredDistance) {");
  Serial.println("  // Handle out-of-range measurements");
  Serial.println("  if (measuredDistance < " + String(measuredDistances[0]) + ") {");
  Serial.println("    return measuredDistance + " + String(calibrationOffsets[0], 2) + ";");
  Serial.println("  }");
  Serial.println("  if (measuredDistance > " + String(measuredDistances[NUM_CALIBRATION_POINTS-1]) + ") {");
  Serial.println("    return measuredDistance + " + String(calibrationOffsets[NUM_CALIBRATION_POINTS-1], 2) + ";");
  Serial.println("  }");
  Serial.println("  ");
  Serial.println("  // Find the closest calibration points for interpolation");
  Serial.println("  int lowerIndex = 0;");
  Serial.println("  int upperIndex = 1;");
  Serial.println("  ");
  Serial.println("  for (int i = 1; i < NUM_CALIBRATION_POINTS; i++) {");
  Serial.println("    if (measuredDistance >= measuredDistances[i-1] && ");
  Serial.println("        measuredDistance <= measuredDistances[i]) {");
  Serial.println("      upperIndex = i;");
  Serial.println("      lowerIndex = i - 1;");
  Serial.println("      break;");
  Serial.println("    }");
  Serial.println("  }");
  Serial.println("  ");
  Serial.println("  // Linear interpolation between the two closest points");
  Serial.println("  float lowerMeasured = measuredDistances[lowerIndex];");
  Serial.println("  float upperMeasured = measuredDistances[upperIndex];");
  Serial.println("  float lowerOffset = calibrationOffsets[lowerIndex];");
  Serial.println("  float upperOffset = calibrationOffsets[upperIndex];");
  Serial.println("  ");
  Serial.println("  float ratio = (measuredDistance - lowerMeasured) / (upperMeasured - lowerMeasured);");
  Serial.println("  float interpolatedOffset = lowerOffset + ratio * (upperOffset - lowerOffset);");
  Serial.println("  ");
  Serial.println("  return measuredDistance + interpolatedOffset;");
  Serial.println("}");
  
  // Flush to ensure all data is sent
  Serial.flush();
}

void testCalibration() {
  Serial.println("\n\nCalibration Testing Mode");
  Serial.println("======================");
  Serial.println("Position the sensor at any distance and press SPACE + ENTER to measure");
  Serial.println("The system will show both raw and calibrated measurements");
  Serial.println("Type 'exit' + ENTER to return to the main menu");
  
  while (true) {
    if (Serial.available()) {
      String cmd = readSerialCommand();
      if (cmd == "exit") {
        currentState = STATE_COMPLETED;
        return;
      } else {
        // Take a measurement
        int retryCount = 0;
        while (!tof.isDataReady() && retryCount < 100) {  // Added retry limit
          delay(5);
          retryCount++;
        }
        
        if (retryCount >= 100) {
          Serial.println("Timeout waiting for sensor data");
          continue;
        }
        
        // Get distance measurement in millimeters
        uint16_t rawDistance = tof.getDistance_mm();
        
        // Check if the reading is valid (TMF8801 returns 0xFFFF for invalid readings)
        if (rawDistance != 0xFFFF) {
          float calibratedDistance = applyCalibratedDistance(rawDistance);
          
          Serial.print("Raw measurement: ");
          Serial.print(rawDistance);
          Serial.print(" mm, Calibrated: ");
          Serial.print(calibratedDistance);
          Serial.println(" mm");
        } else {
          Serial.println("Invalid reading, please try again");
        }
      }
    }
    delay(10);
  }
}

float applyCalibratedDistance(float measuredDistance) {
  // Handle out-of-range measurements
  if (measuredDistance < measuredDistances[0]) {
    return measuredDistance + calibrationOffsets[0];
  }
  if (measuredDistance > measuredDistances[NUM_CALIBRATION_POINTS-1]) {
    return measuredDistance + calibrationOffsets[NUM_CALIBRATION_POINTS-1];
  }
  
  // Find the closest calibration points for interpolation
  int lowerIndex = 0;
  int upperIndex = 1;
  
  for (int i = 1; i < NUM_CALIBRATION_POINTS; i++) {
    if (measuredDistance >= measuredDistances[i-1] && 
        measuredDistance <= measuredDistances[i]) {
      upperIndex = i;
      lowerIndex = i - 1;
      break;
    }
  }
  
  // Linear interpolation between the two closest points
  float lowerMeasured = measuredDistances[lowerIndex];
  float upperMeasured = measuredDistances[upperIndex];
  float lowerOffset = calibrationOffsets[lowerIndex];
  float upperOffset = calibrationOffsets[upperIndex];
  
  float ratio = (measuredDistance - lowerMeasured) / (upperMeasured - lowerMeasured);
  float interpolatedOffset = lowerOffset + ratio * (upperOffset - lowerOffset);
  
  return measuredDistance + interpolatedOffset;
}

bool checkExistingCalibrationData() {
  int signature;
  EEPROM.get(EEPROM_CALIBRATION_ADDR, signature);
  return (signature == EEPROM_SIGNATURE);
}

void saveCalibrationToEEPROM() {
  int eepromAddress = EEPROM_CALIBRATION_ADDR;
  
  // Write signature
  EEPROM.put(eepromAddress, EEPROM_SIGNATURE);
  eepromAddress += sizeof(int);
  
  // Write number of calibration points
  EEPROM.put(eepromAddress, NUM_CALIBRATION_POINTS);
  eepromAddress += sizeof(int);
  
  // Write target distances
  for (int i = 0; i < NUM_CALIBRATION_POINTS; i++) {
    EEPROM.put(eepromAddress, targetDistances[i]);
    eepromAddress += sizeof(int);
  }
  
  // Write measured distances
  for (int i = 0; i < NUM_CALIBRATION_POINTS; i++) {
    EEPROM.put(eepromAddress, measuredDistances[i]);
    eepromAddress += sizeof(float);
  }
  
  // Write calibration offsets
  for (int i = 0; i < NUM_CALIBRATION_POINTS; i++) {
    EEPROM.put(eepromAddress, calibrationOffsets[i]);
    eepromAddress += sizeof(float);
  }
  
  Serial.println("\nCalibration data saved to EEPROM!");
  // Flush to ensure all data is sent
  Serial.flush();
}

void loadCalibrationFromEEPROM() {
  int eepromAddress = EEPROM_CALIBRATION_ADDR;
  
  // Skip signature
  eepromAddress += sizeof(int);
  
  // Read number of calibration points (should match our constant)
  int storedNumPoints;
  EEPROM.get(eepromAddress, storedNumPoints);
  eepromAddress += sizeof(int);
  
  if (storedNumPoints != NUM_CALIBRATION_POINTS) {
    Serial.println("Warning: Stored calibration points don't match expected count!");
  }
  
  int pointsToRead = min(storedNumPoints, NUM_CALIBRATION_POINTS);
  
  // Read target distances
  for (int i = 0; i < pointsToRead; i++) {
    EEPROM.get(eepromAddress, targetDistances[i]);
    eepromAddress += sizeof(int);
  }
  
  // Read measured distances
  for (int i = 0; i < pointsToRead; i++) {
    EEPROM.get(eepromAddress, measuredDistances[i]);
    eepromAddress += sizeof(float);
  }
  
  // Read calibration offsets
  for (int i = 0; i < pointsToRead; i++) {
    EEPROM.get(eepromAddress, calibrationOffsets[i]);
    eepromAddress += sizeof(float);
  }
  
  Serial.println("\nCalibration data loaded from EEPROM!");
  
  // Display the loaded calibration data
  Serial.println("\nLoaded Calibration Data");
  Serial.println("======================");
  Serial.println("Target (mm), Measured (mm), Offset (mm)");
  
  for (int i = 0; i < pointsToRead; i++) {
    Serial.print(targetDistances[i]);
    Serial.print(", ");
    Serial.print(measuredDistances[i]);
    Serial.print(", ");
    Serial.println(calibrationOffsets[i]);
  }
  
  // Flush to ensure all data is sent
  Serial.flush();
}