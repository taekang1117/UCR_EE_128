#include <SPI.h>
#include <MFRC522.h>

#define SS_PIN 10     // SDA pin on RFID
#define RST_PIN 9     // RST pin on RFID

MFRC522 mfrc522(SS_PIN, RST_PIN);

void setup() {
  Serial.begin(9600);
  SPI.begin();
  mfrc522.PCD_Init();

  Serial.println("Scan your RFID tag...");
}

void loop() {
  // Check for new card
  if (!mfrc522.PICC_IsNewCardPresent()) return;
  if (!mfrc522.PICC_ReadCardSerial()) return;

  Serial.print("UID: ");

  // Print each byte of UID
  for (byte i = 0; i < mfrc522.uid.size; i++) {
    Serial.print(mfrc522.uid.uidByte[i], HEX);

    // Add spaces between bytes
    if (i < mfrc522.uid.size - 1) {
      Serial.print(" ");
    }
  }

  Serial.println();

  // Stop communication with this card
  mfrc522.PICC_HaltA();
  mfrc522.PCD_StopCrypto1();
}
