#include <SPI.h>
#include <MFRC522.h>
// NOTE: Servo library removed

// --- Pin definitions ---
#define SS_PIN   10    // RFID SDA/SS
#define RST_PIN   9    // RFID RST
// Servo pin definition removed

MFRC522 mfrc522(SS_PIN, RST_PIN);
// Servo object removed

// --- Your valid key fob/card UID ---
// ... (allowedUID remains the same)

unsigned long lastCardTime = 0;
const unsigned long cardCooldownMs = 500;  // 0.5s ignore after a read

// ------------------------------------------------------------------
// Helper: compare card UID with allowedUID
// ------------------------------------------------------------------
// ... (isAllowedCard remains the same)

// ------------------------------------------------------------------
// Setup
// ------------------------------------------------------------------
void setup() {
  Serial.begin(9600);
  Serial.println("Arduino Initialized: RFID Reader/K64F Trigger.");
  SPI.begin();
  mfrc522.PCD_Init();
  // Servo attachment removed
}

// ------------------------------------------------------------------
// Main loop
// ------------------------------------------------------------------
void loop() {

  // K64F has no need to send anything back, so we remove the RX check.
  
  // Simple cooldown so we don't re-trigger repeatedly with one tap
  if (millis() - lastCardTime < cardCooldownMs) {
    return;
  }

  // Check for new RFID card
  if (!mfrc522.PICC_IsNewCardPresent()) return;
  if (!mfrc522.PICC_ReadCardSerial())   return;

  lastCardTime = millis();   // mark last successful read

  if (isAllowedCard(mfrc522.uid.uidByte, mfrc522.uid.size)) {
    // Access granted: send UNLOCK command to K64F
    Serial.write('1');
    Serial.println("TX: 1 - Sent UNLOCK trigger to K64F.");
  } else {
    Serial.println("Access Denied.");
  }

  // Halt communication with this card
  mfrc522.PICC_HaltA();
  mfrc522.PCD_StopCrypto1();
}
