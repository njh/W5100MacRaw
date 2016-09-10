#include "w5100.h"


void printPaddedHex(uint8_t byte)
{
    char str[2];
    str[0] = (byte >> 4) & 0x0f;
    str[1] = byte & 0x0f;

    for (int i=0; i<2; i++) {
        // base for converting single digit numbers to ASCII is 48
        // base for 10-16 to become lower-case characters a-f is 87
        if (str[i] > 9) str[i] += 39;
        str[i] += 48;
        Serial.print(str[i]);
    }
}

void printMACAddress(const uint8_t address[6])
{
    for (uint8_t i = 0; i < 6; ++i) {
        printPaddedHex(address[i]);
        if (i < 5)
            Serial.print(':');
    }
    Serial.println();
}



const byte mac_address[] = {
    0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
};

Wiznet5100 w5100;

void setup() {
    // Setup serial port for debugging
    Serial.begin(115200);
    Serial.println("[W5100MacRaw]");

    w5100.begin(mac_address);
}


uint8_t buffer[800];

void loop() {

    uint16_t len = w5100.readFrame(buffer, sizeof(buffer));
    if ( len > 0 ) {
        Serial.print("len=");
        Serial.println(len, DEC);

        Serial.print("Dest=");
        printMACAddress(&buffer[0]);
        Serial.print("Src=");
        printMACAddress(&buffer[6]);

        // 0x0800 = IPv4
        // 0x0806 = ARP
        // 0x86DD = IPv6
        Serial.print("Type=0x");
        printPaddedHex(buffer[12]);
        printPaddedHex(buffer[13]);
        Serial.println();

        Serial.println();
    }


    static unsigned long nextMessage = millis();
    if ((long)(millis() - nextMessage) >= 0) {
        Serial.println("Sending test message.");
        memcpy(&buffer[0], "\xff\xff\xff\xff\xff\xff", 6);
        memcpy(&buffer[6], mac_address, 6);
        buffer[12] = 0x88;    // Local Experimental Ethertype
        buffer[13] = 0xB5;
        memcpy(&buffer[14], "Test", 4);

        w5100.sendFrame(buffer, 6 + 6 + 2 + 4);
        nextMessage = millis() + 5000;
    }
}
