#include <Arduino.h>
#include <BluetoothSerial.h>
#include <ESP32Servo.h> // Good if your library uses this header name, or <Servo.h> if the ESP32Servo lib provides that

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to enable it
#endif

BluetoothSerial SerialBT;
String deviceName = "ESP32_RC_Bus";

// --- Pin Definitions ---
const int motorEnablePin = 5;  // ENA for PWM
const int motorPin1 = 18;      // IN1
const int motorPin2 = 19;      // IN2
const int servoPin = 13;
const int headlightsPin = 23;
const int backlightsPin = 22;
const int blinkerLeftPin = 21;
const int blinkerRightPin = 4;
const int lichthupePin = 15;

// --- PWM Configuration (for motor) ---
const int pwmChannel = 0;        // LEDC channel 0
const int pwmFrequency = 1000;   // 1 kHz
const int pwmResolution = 8;     // 8-bit resolution (0-255 duty cycle)

// --- Servo Configuration ---
Servo steeringServo; // If ESP32Servo provides Servo.h, this is fine. If it provides ESP32Servo.h, use that type if necessary.
const int servoCenter = 90;
const int servoLeft = 45;
const int servoRight = 135;
int currentSteerAngle = servoCenter;

// --- Control State Variables ---
bool motorMovingForward = false;
bool motorMovingBackward = false;
bool appGasErlaubt = true;
bool sensorGasErlaubt = true; // Placeholder for actual sensor

// Blinker States
enum BlinkerState { OFF, LEFT_ON, RIGHT_ON };
BlinkerState currentBlinkerState = OFF;
unsigned long blinkerLastChange = 0;
const long blinkInterval = 500; // ms
bool blinkerPinState = false;

// --- Helper Function for Feedback (to USB Serial Monitor) ---
void printCommandFeedback(String command, String action) {
  String feedback = "CMD: " + command + " -> Action: " + action;
  Serial.println(feedback);
}

// --- Motor Control Functions ---
void stoppeMotor() {
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, LOW);
  ledcWrite(pwmChannel, 0); // Set duty cycle to 0 for motor stop
  motorMovingForward = false;
  motorMovingBackward = false;
  printCommandFeedback("Internal", "Motor Stopped");
}

void motorVorwaerts() {
  if (appGasErlaubt && sensorGasErlaubt) {
    digitalWrite(motorPin1, HIGH);
    digitalWrite(motorPin2, LOW);
    ledcWrite(pwmChannel, 200); // Set speed (0-255)
    motorMovingForward = true;
    motorMovingBackward = false;
    printCommandFeedback("Internal", "Motor Forward");
  } else {
    stoppeMotor();
    if (!appGasErlaubt) printCommandFeedback("Internal", "Motor Forward Blocked by App");
    if (!sensorGasErlaubt) printCommandFeedback("Internal", "Motor Forward Blocked by Sensor");
  }
}

void motorRueckwaerts() {
  if (appGasErlaubt) { // Sensor usually doesn't block reverse
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, HIGH);
    ledcWrite(pwmChannel, 150); // Set speed (0-255)
    motorMovingForward = false;
    motorMovingBackward = true;
    printCommandFeedback("Internal", "Motor Backward");
  } else {
    stoppeMotor();
    if (!appGasErlaubt) printCommandFeedback("Internal", "Motor Backward Blocked by App");
  }
}

// --- Steering Control Functions ---
void lenkeLinks() {
  currentSteerAngle = servoLeft;
  steeringServo.write(currentSteerAngle);
  printCommandFeedback("L", "Steer Left");
}

void lenkeRechts() {
  currentSteerAngle = servoRight;
  steeringServo.write(currentSteerAngle);
  printCommandFeedback("R", "Steer Right");
}

void zentriereLenkung() {
  currentSteerAngle = servoCenter;
  steeringServo.write(currentSteerAngle);
  printCommandFeedback("C", "Steer Center");
}

// --- Light Control Functions ---
void blinkerLinksEin() {
  currentBlinkerState = LEFT_ON;
  digitalWrite(blinkerRightPin, LOW); // Turn off other blinker
  blinkerLastChange = millis();
  blinkerPinState = true;
  digitalWrite(blinkerLeftPin, blinkerPinState);
  printCommandFeedback("1", "Blinker Left ON");
}

void blinkerLinksAus() {
  if (currentBlinkerState == LEFT_ON) {
    currentBlinkerState = OFF;
    digitalWrite(blinkerLeftPin, LOW);
    printCommandFeedback("2", "Blinker Left OFF");
  }
}

void blinkerRechtsEin() {
  currentBlinkerState = RIGHT_ON;
  digitalWrite(blinkerLeftPin, LOW); // Turn off other blinker
  blinkerLastChange = millis();
  blinkerPinState = true;
  digitalWrite(blinkerRightPin, blinkerPinState);
  printCommandFeedback("3", "Blinker Right ON");
}

void blinkerRechtsAus() {
  if (currentBlinkerState == RIGHT_ON) {
    currentBlinkerState = OFF;
    digitalWrite(blinkerRightPin, LOW);
    printCommandFeedback("4", "Blinker Right OFF");
  }
}

void scheinwerferEin() {
  digitalWrite(headlightsPin, HIGH);
  printCommandFeedback("5", "Headlights ON");
}

void scheinwerferAus() {
  digitalWrite(headlightsPin, LOW);
  printCommandFeedback("6", "Headlights OFF");
}

void ruecklichterEin() {
  digitalWrite(backlightsPin, HIGH);
  printCommandFeedback("7", "Backlights ON");
}

void ruecklichterAus() {
  digitalWrite(backlightsPin, LOW);
  printCommandFeedback("8", "Backlights OFF");
}

void lichthupe() {
  digitalWrite(lichthupePin, HIGH);
  printCommandFeedback("H", "Lichthupe (Flash ON)");
  delay(300);
  digitalWrite(lichthupePin, LOW);
  printCommandFeedback("H", "Lichthupe (Flash OFF)");
}

void updateBlinkers() {
  if (currentBlinkerState == OFF) {
    return;
  }
  if (millis() - blinkerLastChange > blinkInterval) {
    blinkerLastChange = millis();
    blinkerPinState = !blinkerPinState;
    if (currentBlinkerState == LEFT_ON) {
      digitalWrite(blinkerLeftPin, blinkerPinState);
    } else if (currentBlinkerState == RIGHT_ON) {
      digitalWrite(blinkerRightPin, blinkerPinState);
    }
  }
}


// --- Setup ---
void setup() {
  Serial.begin(115200);
  SerialBT.begin(deviceName);
  Serial.println("ESP32 RC Bus Ready. Waiting for Bluetooth commands...");
  SerialBT.println("ESP32 RC Bus Ready."); // Shorter message for BT

  // Configure Motor Pins
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  // Configure PWM for motorEnablePin (ENA)
  pinMode(motorEnablePin, OUTPUT); // Set as output first
  ledcSetup(pwmChannel, pwmFrequency, pwmResolution);
  ledcAttachPin(motorEnablePin, pwmChannel);
  stoppeMotor(); // Initialize motor in stopped state

  // Configure Servo
  steeringServo.attach(servoPin); // Attach servo to pin
  // steeringServo.setPeriodHertz(50); // Optional: Standard for analog servos if ESP32Servo lib supports it
  zentriereLenkung(); // Center servo at start

  // Configure Light Pins
  pinMode(headlightsPin, OUTPUT);
  pinMode(backlightsPin, OUTPUT);
  pinMode(blinkerLeftPin, OUTPUT);
  pinMode(blinkerRightPin, OUTPUT);
  pinMode(lichthupePin, OUTPUT);

  // Initialize lights to OFF
  digitalWrite(headlightsPin, LOW);
  digitalWrite(backlightsPin, LOW);
  digitalWrite(blinkerLeftPin, LOW);
  digitalWrite(blinkerRightPin, LOW);
  digitalWrite(lichthupePin, LOW);

  appGasErlaubt = true;
  sensorGasErlaubt = true; // Default: sensor allows movement (no obstacle)
}

// --- Main Loop ---
void loop() {
  if (SerialBT.available()) {
    char cmd = SerialBT.read();
    Serial.print("Received via BT: '"); Serial.print(cmd); Serial.println("'"); // Echo to USB Serial

    switch (cmd) {
      // Movement
      case 'F': motorVorwaerts(); break;
      case 'B': motorRueckwaerts(); break;
      case 'S': stoppeMotor(); printCommandFeedback(String(cmd), "Command Stop Motor"); break; // Explicit feedback
      case 'L': lenkeLinks(); break;
      case 'R': lenkeRechts(); break;
      case 'C': zentriereLenkung(); break;

      // Blinkers
      case '1': blinkerLinksEin(); break;
      case '2': blinkerLinksAus(); break;
      case '3': blinkerRechtsEin(); break;
      case '4': blinkerRechtsAus(); break;

      // Other Lights
      case '5': scheinwerferEin(); break;
      case '6': scheinwerferAus(); break;
      case '7': ruecklichterEin(); break;
      case '8': ruecklichterAus(); break;
      case 'H': lichthupe(); break;

      // Auto-Stop Control (App Gas Erlaubt)
      case 'A':
        appGasErlaubt = true;
        printCommandFeedback(String(cmd), "App Gas Allowed (Sensor Control Active)");
        break;
      case 'D':
        appGasErlaubt = false;
        stoppeMotor(); // Explicitly stop the motor
        printCommandFeedback(String(cmd), "App Gas Disallowed (Force Stop by App)");
        break;

      default:
        printCommandFeedback(String(cmd), "Unknown Command Received");
        break;
    }
  }

  updateBlinkers(); // Continuously update blinker state for flashing

  // --- Auto-Stop Logic based on flags ---
  // If motor was moving FORWARD and a state (app or sensor) changed to forbid movement
  if ((!appGasErlaubt || !sensorGasErlaubt) && motorMovingForward) {
      stoppeMotor(); // This will print its own "Motor Stopped" message
      if(!appGasErlaubt) printCommandFeedback("StateCheck", "Motor stopped (App disallowed forward)");
      if(!sensorGasErlaubt) printCommandFeedback("StateCheck", "Motor stopped (Sensor disallowed forward)");
  }
  // If motor was moving BACKWARD and app disallows it
  if (!appGasErlaubt && motorMovingBackward) {
      stoppeMotor(); // This will print its own "Motor Stopped" message
      printCommandFeedback("StateCheck", "Motor stopped (App disallowed backward)");
  }

  delay(10); // Small delay to prevent loop from running too fast
}