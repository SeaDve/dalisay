#include <SPI.h>
#include <MFRC522.h>

#define SS_PIN 5
#define RST_PIN 0

MFRC522 rfid(SS_PIN, RST_PIN); // Instance of the class

void setup()
{
    Serial.begin(9600);

    SPI.begin();
    rfid.PCD_Init();
}

void loop()
{
    if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial())
    {
        String content = "";
        for (byte i = 0; i < rfid.uid.size; i++)
        {
            content += rfid.uid.uidByte[i] < 0x10 ? "0" : "";
            content += String(rfid.uid.uidByte[i], HEX);
        }

        Serial.println(content);

        rfid.PICC_HaltA();
        rfid.PCD_StopCrypto1();
    }
}
