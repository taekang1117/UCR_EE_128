#include <SPI.h>
#include <MFRC522.h>
#include <Servo.h>
#include <SoftwareSerial.h>

// --- Pin definitions ---
#define SS_PIN   10
#define RST_PIN   9
#define SERVO_PIN 6

MFRC522 mfrc522(SS_PIN, RST_PIN);
Servo lockServo;

// K64 serial on pins 2 (RX) and 3 (TX)
SoftwareSerial k64Serial(2, 3);  // RX, TX

const byte allowedUID[] = { 0xFB, 0xA8, 0xC9, 0x06 };
const byte allowedUIDLength = 4;

const int LOCK_POS   = 0;
const int UNLOCK_POS = 90;

bool isUnlocked = false;
unsigned long lastCardTime = 0;
const unsigned long cardCooldownMs = 500;

bool isAllowedCard(const byte *uid, byte uidSize) {
  if (uidSize != allowedUIDLength) return false;
  for (byte i = 0; i < uidSize; i++) {
    if (uid[i] != allowedUID[i]) return false;
  }
  return true;
}

void lockDoor() {
  lockServo.write(LOCK_POS);
  isUnlocked = false;
}

void unlockDoor() {
  lockServo.write(UNLOCK_POS);
  isUnlocked = true;
}

void setup() {
  Serial.begin(9600);      // USB debug
  k64Serial.begin(9600);   // to K64

  SPI.begin();
  mfrc522.PCD_Init();

  lockServo.attach(SERVO_PIN);
  lockDoor();  // start locked
}

void loop() {
  // 1) Check for LOCK command from K64F
  if (k64Serial.available() > 0) {
    char cmd = k64Serial.read();
    if (cmd == 'L') {
      lockDoor();
      Serial.println("Got L from K64 -> locking");
    }
  }

  // cooldown
  if (millis() - lastCardTime < cardCooldownMs) {
    return;
  }

  // 2) RFID read
  if (!mfrc522.PICC_IsNewCardPresent()) return;
  if (!mfrc522.PICC_ReadCardSerial())   return;

  lastCardTime = millis();

  if (isAllowedCard(mfrc522.uid.uidByte, mfrc522.uid.size)) {
    if (!isUnlocked) {
      unlockDoor();
      Serial.println("Card OK -> unlock + send '1' to K64");
      k64Serial.write('1');   // notify K64F
    }
  }

  mfrc522.PICC_HaltA();
  mfrc522.PCD_StopCrypto1();
}
