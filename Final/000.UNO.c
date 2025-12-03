#include <SPI.h>
#include <MFRC522.h>

#define SS_PIN   10   // SDA/SS pin on RFID module
#define RST_PIN   9   // RST pin on RFID module

MFRC522 mfrc522(SS_PIN, RST_PIN);

// --- Your valid key fob/card UID (replace with your real one) ---
const byte allowedUID[] = { 0xDE, 0xAD, 0xBE, 0xEF };  // example
const byte allowedUIDSize = sizeof(allowedUID);

// Cooldown between sends
unsigned long lastSendTime = 0;
const unsigned long cooldownMs = 500;   // 0.5 seconds

// Compare card UID with allowedUID
bool isAllowedCard(const MFRC522::Uid &uid) {
  if (uid.size != allowedUIDSize) {
    return false;
  }
  for (byte i = 0; i < uid.size; i++) {
    if (uid.uidByte[i] != allowedUID[i]) {
      return false;
    }
  }
  return true;
}

void setup() {
  Serial.begin(9600);        // Must match K64F UART1 baud
  SPI.begin();
  mfrc522.PCD_Init();

  // Optional debug (only visible if Arduino is connected to PC, not K64F)
  Serial.println("RFID Arduino: ready to send '1' on valid key fob.");
}

void loop() {
  // 1) Check for a new card
  if (!mfrc522.PICC_IsNewCardPresent()) {
    return;
  }
  if (!mfrc522.PICC_ReadCardSerial()) {
    return;
  }

  // 2) We have a UID; check if it's allowed
  bool ok = isAllowedCard(mfrc522.uid);

  if (ok) {
    unsigned long now = millis();

    // 3) Enforce 0.5s cooldown between sends
    if (now - lastSendTime >= cooldownMs) {
      lastSendTime = now;

      // Send ASCII '1' to K64F
      Serial.write('1');    // <-- this is what K64F sees on UART RX

      // Optional: debug to Serial Monitor
      Serial.println("Sent '1' to K64F (valid key fob).");
    } else {
      // Optional: debug for cooldown hits
      // Serial.println("Valid card but in cooldown; not sending.");
    }
  } else {
    // Optional: debug for invalid card
    // Serial.println("Unknown card; not sending.");
  }

  // 4) Stop communication with this card
  mfrc522.PICC_HaltA();
  mfrc522.PCD_StopCrypto1();
}
