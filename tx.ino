#include <MPU6050.h>

#include <VirtualWire_Config.h>
#include <VirtualWire.h>

#include <Wire.h>

const int ledCenter = 2;
const int ledLeft = 3;
const int ledRight = 4;
const int ledForward = 5;
const int ledBackward = 6;
const int rfTxPin = 12;
const int commLed = 7;
const int emgPin = A0;

// EMG processing constants
#define SAMPLE_RATE 500
#define BUFFER_SIZE 16
int circular_buffer[BUFFER_SIZE];
int data_index = 0;
int sum = 0;
int emgThreshold = 15;

// MPU6050 sensor
MPU6050 mpu;

// Variables for timing
unsigned long lastSampleTime = 0;
unsigned long sampleInterval = 1000000 / SAMPLE_RATE;
unsigned long lastMpuCheckTime = 0;
unsigned long mpuCheckInterval = 200;

// Command to send
char gesture = 'S';
bool emgActive = false;

void setup() {
  Serial.begin(9600);
  
  // Initialize MPU6050
  Wire.begin();
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    while (1);
  }
  
  // Set up LED pins
  pinMode(ledCenter, OUTPUT);
  pinMode(ledLeft, OUTPUT);
  pinMode(ledRight, OUTPUT);
  pinMode(ledForward, OUTPUT);
  pinMode(ledBackward, OUTPUT);
  pinMode(commLed, OUTPUT);
  
  // Set up wireless transmitter
  vw_set_tx_pin(rfTxPin);
  vw_setup(2000); // 2kbps
  
  // Initialize with stationary state
  showGesture(ledCenter);
  
  Serial.println("Hybrid EMG/MPU6050 Controlled Robot Transmitter");
  Serial.println("Ready for input");
}

void loop() {
  unsigned long currentTime = micros();
  
  // EMG PROCESSING
  // Check if it's time to process EMG
  if (currentTime - lastSampleTime >= sampleInterval) {
    lastSampleTime = currentTime;
    
    // Read and process EMG signal
    int rawEmg = analogRead(emgPin);
    int filteredEmg = EMGFilter(rawEmg);
    int emgEnvelope = getEnvelop(abs(filteredEmg));
    
    // Update EMG active state
    emgActive = (emgEnvelope > emgThreshold);
    
    // Debug EMG values
    Serial.print("EMG: Raw=");
    Serial.print(rawEmg);
    Serial.print(", Filtered=");
    Serial.print(filteredEmg);
    Serial.print(", Envelope=");
    Serial.print(emgEnvelope);
    Serial.print(", Active=");
    Serial.println(emgActive);
  }
  
  // MPU6050 PROCESSING
  // Check if it's time to check MPU6050
  if (currentTime - lastMpuCheckTime >= mpuCheckInterval) {
    lastMpuCheckTime = currentTime;
    
    // Get acceleration data
    int16_t ax, ay, az;
    mpu.getAcceleration(&ax, &ay, &az);
    
    // Debug MPU values
    Serial.print("MPU: ax=");
    Serial.print(ax);
    Serial.print(", ay=");
    Serial.println(ay);
    
    // Determine gesture based on both sensors
    // EMG has priority for forward movement if active
    if (emgActive) {
      gesture = 'L';  // Left movement via EMG
      showGesture(ledForward);
    } 
    // Otherwise use MPU6050 data
    else if (ax > 4000) {
      gesture = 'F';  // Forward movement
      showGesture(ledRight);
    } 
    else if (ax < -4000) {
      gesture = 'B';  // Backward movement
      showGesture(ledLeft);
    } 
    else if (ay < -4000) {
      gesture = 'R';  // Right movement
      showGesture(ledBackward);
    } 
    else {
      gesture = 'S';  // Stop (default)
      showGesture(ledCenter);
    }
    
    // Send the command wirelessly
    vw_send((uint8_t *)&gesture, 1);
    vw_wait_tx(); // Wait until transmission is done
    
    // Blink communication LED
    digitalWrite(commLed, HIGH);
    delay(50);
    digitalWrite(commLed, LOW);
    
    // Debug output
    Serial.print("Sending command: ");
    Serial.println(gesture);
  }
}

// Function to light up the appropriate LED based on gesture
void showGesture(int ledPin) {
  digitalWrite(ledCenter, LOW);
  digitalWrite(ledLeft, LOW);
  digitalWrite(ledRight, LOW);
  digitalWrite(ledForward, LOW);
  digitalWrite(ledBackward, LOW);
  digitalWrite(ledPin, HIGH);
}

// EMG envelope calculation (moving average of absolute value)
int getEnvelop(int abs_emg) {
  sum -= circular_buffer[data_index];
  sum += abs_emg;
  circular_buffer[data_index] = abs_emg;
  data_index = (data_index + 1) % BUFFER_SIZE;
  return (sum / BUFFER_SIZE) * 2;  // Multiply by 2 to amplify signal
}

// Band-Pass Butterworth IIR digital filter for EMG signal
// Sampling rate: 500.0 Hz, frequency: [74.5, 149.5] Hz
float EMGFilter(float input)
{
  float output = input;
  {
    static float z1, z2; // filter section state
    float x = output - 0.05159732*z1 - 0.36347401*z2;
    output = 0.01856301*x + 0.03712602*z1 + 0.01856301*z2;
    z2 = z1;
    z1 = x;
  }
  {
    static float z1, z2; // filter section state
    float x = output - -0.53945795*z1 - 0.39764934*z2;
    output = 1.00000000*x + -2.00000000*z1 + 1.00000000*z2;
    z2 = z1;
    z1 = x;
  }
  {
    static float z1, z2; // filter section state
    float x = output - 0.47319594*z1 - 0.70744137*z2;
    output = 1.00000000*x + 2.00000000*z1 + 1.00000000*z2;
    z2 = z1;
    z1 = x;
  }
  {
    static float z1, z2; // filter section state
    float x = output - -1.00211112*z1 - 0.74520226*z2;
    output = 1.00000000*x + -2.00000000*z1 + 1.00000000*z2;
    z2 = z1;
    z1 = x;
  }
  return output;
}
