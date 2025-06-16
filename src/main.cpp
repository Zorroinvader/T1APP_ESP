#include "BluetoothSerial.h" // For Bluetooth Classic SPP

// Check if Bluetooth components are enabled in ESP32 configuration
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `idf.py menuconfig` (for ESP-IDF) or check your board/SDK settings (for Arduino) to enable it.
#endif

BluetoothSerial SerialBT; // Create an instance of the BluetoothSerial class

// --- Status LED Configuration (for Bluetooth connection status) ---
#ifndef LED_BUILTIN
  #define LED_BUILTIN 2 // Default to pin 2 if not defined by board package
#endif
const int statusLedPin = LED_BUILTIN;
bool isClientConnected = false;

// --- Motor Control & Blinker Pin Definitions (Example Placeholder) ---
// IMPORTANT: Replace these with actual pins if you connect LEDs/motors
// const int MOTOR_PIN_FORWARD_LED = 13;
// const int MOTOR_PIN_BACKWARD_LED = 12;
// const int MOTOR_PIN_LEFT_LED = 14;
// const int MOTOR_PIN_RIGHT_LED = 27;
// const int MOTOR_PIN_STOP_LED = 26;
const int BLINKER_LEFT_LED_PIN = 15;  // << EXAMPLE - Actual GPIO for Left Blinker LED
const int BLINKER_RIGHT_LED_PIN = 16; // << EXAMPLE - Actual GPIO for Right Blinker LED


// --- Placeholder Motor Control Functions ---
void allMotorsStop() {
  Serial.println("SIM: All motors STOPPED.");
  // --- Example LED Control (Uncomment and set pins if used) ---
  // digitalWrite(MOTOR_PIN_FORWARD_LED, LOW);
  // digitalWrite(MOTOR_PIN_BACKWARD_LED, LOW);
  // digitalWrite(MOTOR_PIN_LEFT_LED, LOW);
  // digitalWrite(MOTOR_PIN_RIGHT_LED, LOW);
  // digitalWrite(MOTOR_PIN_STOP_LED, HIGH); // Turn ON stop indicator
}

void moveForward() {
  Serial.println("SIM: Moving FORWARD.");
  allMotorsStop(); // Good practice
  // --- Example LED Control ---
  // digitalWrite(MOTOR_PIN_FORWARD_LED, HIGH);
  // digitalWrite(MOTOR_PIN_STOP_LED, LOW);
}

void moveBackward() {
  Serial.println("SIM: Moving BACKWARD.");
  allMotorsStop();
  // --- Example LED Control ---
  // digitalWrite(MOTOR_PIN_BACKWARD_LED, HIGH);
  // digitalWrite(MOTOR_PIN_STOP_LED, LOW);
}

void turnLeft() {
  Serial.println("SIM: Turning LEFT.");
  allMotorsStop();
  // --- Example LED Control ---
  // digitalWrite(MOTOR_PIN_LEFT_LED, HIGH);
  // digitalWrite(MOTOR_PIN_STOP_LED, LOW);
}

void turnRight() {
  Serial.println("SIM: Turning RIGHT.");
  allMotorsStop();
  // --- Example LED Control ---
  // digitalWrite(MOTOR_PIN_RIGHT_LED, HIGH);
  // digitalWrite(MOTOR_PIN_STOP_LED, LOW);
}

// --- NEW Blinker Control Functions ---
void activateLeftBlinker() {
  Serial.println("SIM: Blinker Left ACTIVATED (Command 'X').");
  // --- Actual Physical Control (e.g., LED) ---
  // digitalWrite(BLINKER_LEFT_LED_PIN, HIGH);
  Serial.println("ESP32: Left Blinker Light ON"); // Terminal output
  delay(3000); // Wait for 3 seconds
  // digitalWrite(BLINKER_LEFT_LED_PIN, LOW);
  Serial.println("ESP32: Left Blinker Light OFF"); // Terminal output
  Serial.println("SIM: Blinker Left DEACTIVATED after 3s.");
}

void activateRightBlinker() {
  Serial.println("SIM: Blinker Right ACTIVATED (Command 'Y').");
  // --- Actual Physical Control (e.g., LED) ---
  // digitalWrite(BLINKER_RIGHT_LED_PIN, HIGH);
  Serial.println("ESP32: Right Blinker Light ON"); // Terminal output
  delay(3000); // Wait for 3 seconds
  // digitalWrite(BLINKER_RIGHT_LED_PIN, LOW);
  Serial.println("ESP32: Right Blinker Light OFF"); // Terminal output
  Serial.println("SIM: Blinker Right DEACTIVATED after 3s.");
}


// --- Bluetooth Event Callback Functions ---
// (This is your existing reliable callback)
void btClientConnectedCallback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
  if (event == ESP_SPP_SRV_OPEN_EVT) { // Client connected
    Serial.println(">>> Android Controller Connected! <<<");
    isClientConnected = true;
    digitalWrite(statusLedPin, HIGH); // Turn status LED ON solid
    allMotorsStop(); // Ensure motors are stopped on new connection
  } else if (event == ESP_SPP_CLOSE_EVT) { // Client disconnected
    Serial.println(">>> Android Controller Disconnected. Waiting for new connection... <<<");
    isClientConnected = false;
    digitalWrite(statusLedPin, LOW); // Turn status LED OFF
    allMotorsStop(); // Ensure motors are stopped when client disconnects
  }
}

// --- Setup Function: Runs once when ESP32 boots or resets ---
void setup() {
  // Initialize USB Serial for debugging
  Serial.begin(115200);
  while (!Serial); // Wait for Serial port to connect
  Serial.println("\n\n--- ESP32 Controller (Single Char Commands + Blinkers) ---");

  // Initialize Status LED
  pinMode(statusLedPin, OUTPUT);
  digitalWrite(statusLedPin, LOW);
  Serial.println("Status LED Initialized.");

  // --- Initialize Motor/Blinker "Indicator" Pins (Example with LEDs) ---
  Serial.println("Initializing Motor/Blinker Indicator LEDs (if enabled in code)...");
  // pinMode(MOTOR_PIN_FORWARD_LED, OUTPUT); digitalWrite(MOTOR_PIN_FORWARD_LED, LOW);
  // pinMode(MOTOR_PIN_BACKWARD_LED, OUTPUT); digitalWrite(MOTOR_PIN_BACKWARD_LED, LOW);
  // pinMode(MOTOR_PIN_LEFT_LED, OUTPUT); digitalWrite(MOTOR_PIN_LEFT_LED, LOW);
  // pinMode(MOTOR_PIN_RIGHT_LED, OUTPUT); digitalWrite(MOTOR_PIN_RIGHT_LED, LOW);
  // pinMode(MOTOR_PIN_STOP_LED, OUTPUT); digitalWrite(MOTOR_PIN_STOP_LED, LOW);
  pinMode(BLINKER_LEFT_LED_PIN, OUTPUT); digitalWrite(BLINKER_LEFT_LED_PIN, LOW);   // << NEW - For physical LED
  pinMode(BLINKER_RIGHT_LED_PIN, OUTPUT); digitalWrite(BLINKER_RIGHT_LED_PIN, LOW); // << NEW - For physical LED

  // --- Initialize Bluetooth Serial ---
  String deviceName = "ESP32_Ready"; // Your working device name
  Serial.print("Initializing Bluetooth with device name: '");
  Serial.print(deviceName);
  Serial.println("'...");

  SerialBT.register_callback(btClientConnectedCallback); // Register unified callback

  if (!SerialBT.begin(deviceName)) {
    Serial.println("!!! CRITICAL ERROR: Bluetooth Serial begin() FAILED !!!");
    while (true) { // Halt on critical error
      digitalWrite(statusLedPin, HIGH); delay(100);
      digitalWrite(statusLedPin, LOW);  delay(100);
    }
  } else {
    Serial.println("*** Bluetooth Serial Initialized SUCCESSFULLY! ***");
    Serial.println("*** Waiting for Android app to connect... ***");
  }

  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_BT);
  String macAddress = "";
  for(int i=0; i<6; i++){
    if(mac[i]<0x10) macAddress += "0";
    macAddress += String(mac[i], HEX);
    if(i<5) macAddress += ":";
  }
  macAddress.toUpperCase();
  Serial.print("My Bluetooth MAC Address is: ");
  Serial.println(macAddress);
  Serial.println("-------------------------------------------------");
  Serial.println("Send commands (F, B, L, R, S, X, Y) from the Android app."); // Updated help

  allMotorsStop(); // Initial state: motors stopped
}

// --- Loop Function: Simple single-character processing ---
void loop() {
  if (isClientConnected) {
    if (SerialBT.available()) {
      char command = (char)SerialBT.read(); // Read the single incoming byte

      Serial.print("Android App sent command: '");
      Serial.print(command);
      Serial.println("'");

      switch (command) {
        case 'F':
          moveForward();
          break;
        case 'B':
          moveBackward();
          break;
        case 'L':
          turnLeft();
          break;
        case 'R':
          turnRight();
          break;
        case 'S':
          allMotorsStop();
          break;
        // --- NEW BLINKER COMMANDS ---
        case 'X': // Command for Left Blinker
          activateLeftBlinker();
          break;
        case 'Y': // Command for Right Blinker
          activateRightBlinker();
          break;
        // --- END OF NEW BLINKER COMMANDS ---
        default:
          Serial.print("Unknown command: '");
          Serial.print(command);
          Serial.println("'. No action taken.");
          break;
      }
    }
  } else {
    // Optional: "Waiting for connection" LED pattern for statusLedPin
    // digitalWrite(statusLedPin, HIGH); delay(700);
    // digitalWrite(statusLedPin, LOW);  delay(700);
  }
  // Other non-blocking code can go here
}