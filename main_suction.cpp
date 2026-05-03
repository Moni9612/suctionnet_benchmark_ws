
// #include <Arduino.h>

// int sensorrelayPin = 40;
// int relayPin = 38;
// int sensor_pin = A0; // Analog 0
// int sensorvalue = 0; // number of analog readings to average

// bool loggingEnabled = false;

// void setup() {
//   Serial.begin(9600);
//   pinMode(relayPin, OUTPUT);
// }
// void loop() {
//   if (Serial.available() > 0) {
//     String command = Serial.readStringUntil('\n');
//     command.trim();

//     if (command == "SOFF") {
//       digitalWrite(sensorrelayPin, HIGH);
//       Serial.println("START_LOGGING");
//       loggingEnabled = true;
//     } 
//     else if (command == "SON") {
//       digitalWrite(sensorrelayPin, LOW);
//       Serial.println("STOP_LOGGING");
//       loggingEnabled = false;
//     }
//     else if (command == "ON") {
//             digitalWrite(relayPin, HIGH);
//             Serial.println("Relay is ON");
//         } 
//     else if (command == "OFF") {
//             digitalWrite(relayPin, LOW);
//             Serial.println("Relay is OFF");
//         }    
//   }

//   int  y = 0.0;  // default value if fsrReading == 0

//   if (loggingEnabled) {
//   int sensorvalue = analogRead(sensor_pin);
//     // if (sensorvalue > 90.0) {
//     //     y = 0.0;
//     //   } else {
//     //     y = sensorvalue;
//     //   }

//   Serial.print("FSR:");
//   Serial.print(sensorvalue);
//   Serial.print(",TIME:");
//   Serial.println(millis());
//   delay(100); // ~10 samples/sec
// }
// }