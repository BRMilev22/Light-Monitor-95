/*
 * Light-Level Monitoring System - Simplified Arduino Sketch
 *
 * Hardware:
 * - Photoresistor (LDR) on A0 with 10kΩ resistor to GND
 * - LED on Pin 9 with 220Ω resistor
 *
 * Serial Protocol:
 * - Sends: "LIGHT:<value>" every read
 * - Receives: "T:<value>" to set threshold (0-1023)
 */

const int LDR_PIN = A0;
const int LED_PIN = 9;
const int BAUD_RATE = 115200;

int threshold = 512 + 256; // Default threshold

void setup() {
  Serial.begin(BAUD_RATE);
  pinMode(LDR_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  Serial.println("READY");
}

void loop() {
  // Read light level
  int lightLevel = analogRead(LDR_PIN);

  // Send to serial for ROS2
  Serial.print("LIGHT:");
  Serial.println(lightLevel);

  // Control LED based on threshold
  if (lightLevel < threshold) {
    digitalWrite(LED_PIN, HIGH); // Dark - LED ON
  } else {
    digitalWrite(LED_PIN, LOW); // Bright - LED OFF
  }

  // Check for threshold updates from ROS2
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    if (cmd.startsWith("T:")) {
      int newThreshold = cmd.substring(2).toInt();
      if (newThreshold >= 0 && newThreshold <= 1023) {
        threshold = newThreshold;
      }
    }
  }

  delay(100); // 10Hz update rate
}
