#include <SPI.h>
#include <MFRC522.h>
#include <Servo.h>

// --- Pin definitions ---
#define SS_PIN   10    // RFID SDA/SS
#define RST_PIN   9    // RFID RST
#define SERVO_PIN 6    // Servo signal pin

MFRC522 mfrc522(SS_PIN, RST_PIN);
Servo lockServo;

const byte allowedUID[] = { 0xFB, 0xA8, 0xC9, 0x06 }; // our card key 
const byte allowedUIDLength = 4;


const int LOCK_POS   = 0;    // 12 o'clock (locked)
const int UNLOCK_POS = 90;   // 3 o'clock (unlocked)

bool isUnlocked = false;
unsigned long lastCardTime = 0;     // prevent spamming
const unsigned long cardCooldownMs = 500;  // 0.5s cool time

bool isAllowedCard(const byte *uid, byte uidSize) {
  if (uidSize != allowedUIDLength) return false;
  for (byte i = 0; i < uidSize; i++) {
    if (uid[i] != allowedUID[i]) {
      return false;
    }
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
  Serial.begin(9600);
  SPI.begin();
  mfrc522.PCD_Init();
  lockServo.attach(SERVO_PIN);
  lockDoor();    // start locked
}

void loop() {

  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == 'L') {     
      lockDoor();
    }
  }

  if (millis() - lastCardTime < cardCooldownMs) {
    return;
  }

  if (!mfrc522.PICC_IsNewCardPresent()) return;
  if (!mfrc522.PICC_ReadCardSerial())   return;

  lastCardTime = millis();   

  if (isAllowedCard(mfrc522.uid.uidByte, mfrc522.uid.size)) {
    if (!isUnlocked) {
      unlockDoor();
      Serial.write('1');   
    }
  }


  mfrc522.PICC_HaltA();
  mfrc522.PCD_StopCrypto1();
}
