#include <SPI.h>
#include <MFRC522.h>

// --- Pin definitions ---
#define SS_PIN   10    // RFID SDA/SS
#define RST_PIN   9    // RFID RST

MFRC522 mfrc522(SS_PIN, RST_PIN);

// --- Your valid key fob/card UID ---
// From your debug output: FB A8 C9 06
const byte allowedUID[] = { 0xFB, 0xA8, 0xC9, 0x06 };
const byte allowedUIDLength = 4;

unsigned long lastCardTime = 0;     // to avoid spamming reads
const unsigned long cardCooldownMs = 500;  // 0.5s ignore after a read

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
// Setup
// ------------------------------------------------------------------
void setup() {
  // UART to K64F (set same baud on K64F)
  Serial.begin(9600);

  // RFID init
  SPI.begin();
  mfrc522.PCD_Init();
}

// ------------------------------------------------------------------
// Main loop
// ------------------------------------------------------------------
void loop() {
  // Simple cooldown so we don't re-trigger repeatedly with one tap
  if (millis() - lastCardTime < cardCooldownMs) {
    return;
  }

  // Check for new RFID card
  if (!mfrc522.PICC_IsNewCardPresent()) return;
  if (!mfrc522.PICC_ReadCardSerial())   return;

  lastCardTime = millis();   // mark last successful read

  if (isAllowedCard(mfrc522.uid.uidByte, mfrc522.uid.size)) {
    // Send '1' via UART when correct key fob is presented
    Serial.write('1');
  }

  // Halt communication with this card
  mfrc522.PICC_HaltA();
  mfrc522.PCD_StopCrypto1();
}
