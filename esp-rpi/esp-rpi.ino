#define BLYNK_TEMPLATE_ID "TMPL2NQ1TOX4M"
#define BLYNK_TEMPLATE_NAME "carcontroller"
#define BLYNK_AUTH_TOKEN "K0lZwlgK7UZ73QVsQ9Q4W2zyrREp4tkY"

#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <SoftwareSerial.h>

TFT_eSPI tft = TFT_eSPI(); // TFT instance

// Motor control pins
#define ENA 5   // GPIO5 (D1)
#define IN1 4   // GPIO4 (D2)
#define IN2 14  // GPIO14 (D5)
#define IN3 12  // GPIO12 (D6)
#define IN4 13  // GPIO13 (D7)
#define ENB 15  // GPIO15 (D8)

// WiFi credentials
char auth[] = BLYNK_AUTH_TOKEN;
char ssid[] = "yusuff";
char pass[] = "3189b[E0";

SoftwareSerial rpiSerial(3, 1);  // RX = D9 (GPIO3), TX = D10 (GPIO1)
String incomingMsg = "";
bool autoParking = false;
bool parkingDone = false;

// Blynk control flags
bool forward = 0, backward = 0, left = 0, right = 0;
bool isForwardLeft = 0, isForwardRight = 0;
bool isBackwardLeft = 0, isBackwardRight = 0;
int Speed = 130;
int turnSpeed = 130;
int reverseSpeed = 130;

unsigned long lastDisplayUpdate = 0;
const int displayUpdateInterval = 500;

void setup() {
  Serial.begin(115200);
  rpiSerial.begin(9600);

  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);
  tft.setCursor(0, 0);
  tft.println("Manual Car Control");
  tft.println("Connecting...");

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  WiFi.begin(ssid, pass);
  unsigned long startTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startTime < 15000) {
    delay(500);
    tft.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    tft.println("\nWiFi connected!");
    tft.println("Connecting to Blynk...");
    Blynk.begin(auth, ssid, pass);
    tft.println("Blynk connected!");
  } else {
    tft.println("\nWiFi Failed. Offline.");
  }

  delay(2000);
  updateDisplay();
}

BLYNK_WRITE(V0) { forward = param.asInt(); updateDisplay(); }
BLYNK_WRITE(V1) { backward = param.asInt(); updateDisplay(); }
BLYNK_WRITE(V2) { left = param.asInt(); updateDisplay(); }
BLYNK_WRITE(V3) { right = param.asInt(); updateDisplay(); }
BLYNK_WRITE(V4) { Speed = param.asInt(); updateDisplay(); }
BLYNK_WRITE(V6) { isForwardLeft = param.asInt(); updateDisplay(); }
BLYNK_WRITE(V7) { isForwardRight = param.asInt(); updateDisplay(); }
BLYNK_WRITE(V8) { isBackwardLeft = param.asInt(); updateDisplay(); }
BLYNK_WRITE(V9) { isBackwardRight = param.asInt(); updateDisplay(); }

void loop() {
  if (WiFi.status() == WL_CONNECTED) {
    Blynk.run();
  }

  handleSerialFromRPi();

  if (!autoParking) {
    manualControl();
  } else if (!parkingDone) {
    performAutoParking();
    parkingDone = true;
    autoParking = false;
  } else {
    carStop(); // Maintain stop after parking
  }

  if (millis() - lastDisplayUpdate > displayUpdateInterval) {
    updateDisplay();
    lastDisplayUpdate = millis();
  }
}

void handleSerialFromRPi() {
  while (rpiSerial.available()) {
    char c = rpiSerial.read();
    if (c == '\n') {
      incomingMsg.trim();

      if (incomingMsg == "clear") {
        if (!parkingDone) {
          autoParking = true;
          tft.println("RPi: clear → START AUTO PARK");
          Serial.println("Received: clear");
        }
      } else if (incomingMsg == "not clear") {
        autoParking = false;
        parkingDone = false;
        tft.println("RPi: not clear");
        Serial.println("Received: not clear");
      }

      incomingMsg = "";
    } else {
      incomingMsg += c;
    }
  }
}

void performAutoParking() {
  int parkSpeed = 80;

  tft.println("AUTO: Step 1 FWD 30cm");
  analogWrite(ENA, parkSpeed);
  analogWrite(ENB, parkSpeed);
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  delay(1250);
  carStop(); 
  delay(300);

  tft.println("AUTO: Step 2 left 7cm");
   analogWrite(ENA, turnSpeed);
  analogWrite(ENB, turnSpeed);
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  delay(150);
  carStop(); 
  delay(300);

  tft.println("AUTO: Step 3 BWD right 35cm");
    analogWrite(ENA, reverseSpeed);
  analogWrite(ENB, reverseSpeed / 2);
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  delay(870);

    carStop(); 
  delay(300);

  tft.println("AUTO: Step 4 Right 7cm");
   analogWrite(ENA, turnSpeed);
  analogWrite(ENB, turnSpeed);
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  delay(235);

    carStop(); 
  delay(300);


  

    tft.println("AUTO: Step 1 FWD 30cm");
  analogWrite(ENA, parkSpeed);
  analogWrite(ENB, parkSpeed);
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  delay(400);

  carStop();
  tft.println("AUTO: PARKING DONE");
}

void manualControl() {
  if (isForwardLeft) carForwardLeft();
  else if (isForwardRight) carForwardRight();
  else if (isBackwardLeft) carBackwardLeft();
  else if (isBackwardRight) carBackwardRight();
  else if (forward) carForward();
  else if (backward) carBackward();
  else if (left) carTurnLeft();
  else if (right) carTurnRight();
  else carStop();
}

void updateDisplay() {
  tft.fillRect(0, 50, tft.width(), 90, TFT_BLACK);
  tft.setCursor(0, 50);
  tft.println(autoParking ? "Mode: AUTO" : "Mode: MANUAL");
  tft.print("Speed: "); tft.println(Speed);
  tft.print("Movement: ");
  if (forward) tft.println("FORWARD");
  else if (backward) tft.println("BACKWARD");
  else if (left) tft.println("LEFT");
  else if (right) tft.println("RIGHT");
  else if (isForwardLeft) tft.println("FWD-LEFT");
  else if (isForwardRight) tft.println("FWD-RIGHT");
  else if (isBackwardLeft) tft.println("BACK-LEFT");
  else if (isBackwardRight) tft.println("BACK-RIGHT");
  else tft.println("STOPPED");
}

// Movement Functions
void carForward() {
  analogWrite(ENA, Speed);
  analogWrite(ENB, Speed);
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}

void carBackward() {
  analogWrite(ENA, reverseSpeed);
  analogWrite(ENB, reverseSpeed);
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
}

void carTurnLeft() {
  analogWrite(ENA, turnSpeed);
  analogWrite(ENB, turnSpeed);
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}

void carTurnRight() {
  analogWrite(ENA, turnSpeed);
  analogWrite(ENB, turnSpeed);
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
}

void carStop() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}

void carForwardLeft() {
  analogWrite(ENA, Speed / 2);
  analogWrite(ENB, Speed);
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}

void carForwardRight() {
  analogWrite(ENA, Speed);
  analogWrite(ENB, Speed / 2);
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}

void carBackwardLeft() {
  analogWrite(ENA, reverseSpeed / 2);
  analogWrite(ENB, reverseSpeed);
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
}

void carBackwardRight() {
  analogWrite(ENA, reverseSpeed);
  analogWrite(ENB, reverseSpeed / 2);
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
}