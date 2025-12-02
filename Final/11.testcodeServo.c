#include <SPI.h>
#include <MFRC522.h>
#include <Servo.h>

// --- Pin definitions ---
#define SS_PIN   10    // RFID SDA/SS
#define RST_PIN   9    // RFID RST
#define SERVO_PIN 6    // Servo signal pin

MFRC522 mfrc522(SS_PIN, RST_PIN);
Servo lockServo;

// --- Your valid key fob/card UID ---
// Use your previously tested UID
const byte allowedUID[] = { 0xFB, 0xA8, 0xC9, 0x06 }; 
const byte allowedUIDLength = 4;

// --- Servo angles ---
const int LOCK_POS   = 0;    // Locked position
const int UNLOCK_POS = 90;   // Unlocked position

bool isUnlocked = false; // Initial state is Locked (0°)
unsigned long lastCardTime = 0;
const unsigned long cardCooldownMs = 500; // 0.5s ignore after a read

// ------------------------------------------------------------------
// Helper: compare card UID with allowedUID
// ------------------------------------------------------------------
bool isAllowedCard(const byte *uid, byte uidSize) {
  if (uidSize != allowedUIDLength) return false;
  for (byte i = 0; i < uidSize; i++) {
    if (uid[i] != allowedUID[i]) {
      return false;
    }
  }
  return true;
}

// ------------------------------------------------------------------
// Lock / unlock helpers
// ------------------------------------------------------------------
void lockDoor() {
  lockServo.write(LOCK_POS);
  isUnlocked = false;
  Serial.println("--- LOCKED (0 degrees) ---");
}

void unlockDoor() {
  lockServo.write(UNLOCK_POS);
  isUnlocked = true;
  Serial.println("--- UNLOCKED (90 degrees) ---");
}

// ------------------------------------------------------------------
// Setup
// ------------------------------------------------------------------
void setup() {
  // Use Serial for debugging only
  Serial.begin(9600);
  Serial.println("RFID Toggle Lock System Initialized.");

  // RFID init
  SPI.begin();
  mfrc522.PCD_Init();

  // Servo init
  lockServo.attach(SERVO_PIN);
  lockDoor();    // start locked at 0 degrees
}

// ------------------------------------------------------------------
// Main loop
// ------------------------------------------------------------------
void loop() {

  // Simple cooldown so we don't re-trigger repeatedly with one tap
  if (millis() - lastCardTime < cardCooldownMs) {
    return;
  }

  // 1) Check for new RFID card
  if (!mfrc522.PICC_IsNewCardPresent()) return;
  if (!mfrc522.PICC_ReadCardSerial())   return;

  lastCardTime = millis();   // mark last successful read

  if (isAllowedCard(mfrc522.uid.uidByte, mfrc522.uid.size)) {
    Serial.print("Access Granted. Card UID: ");
    for (byte i = 0; i < mfrc522.uid.size; i++) {
        Serial.print(mfrc522.uid.uidByte[i] < 0x10 ? " 0" : " ");
        Serial.print(mfrc522.uid.uidByte[i], HEX);
    }
    Serial.println();

    // Toggle Logic: If unlocked, lock it. If locked, unlock it.
    if (isUnlocked) {
      lockDoor();
    } else {
      unlockDoor();
    }
  } else {
    Serial.println("Access Denied.");
  }

  // Halt communication with this card
  mfrc522.PICC_HaltA();
  mfrc522.PCD_StopCrypto1();
}
