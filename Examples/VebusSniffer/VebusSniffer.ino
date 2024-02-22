#include <Arduino.h>

#include <vector>
#include <VEBusDefinition.h>
#include <VEBus.h>
#include "config.h"

#define sizeofarray(x) sizeof x / sizeof x[0]

VEBus _vEBus(Serial1, RS485_RX_PIN, RS485_TX_PIN, RS485_EN_PIN);

unsigned long lastSendTime = 0;

//Blacklist for receive callback
VEBus::Blacklist _blacklist[] = { {.value = 0xE4, .at = 4}, {.value = 0x55, .at = 4} };

void Receive(std::vector<uint8_t>& buffer)
{
    _vEBus.DestuffingFAtoFF(buffer);
    Serial.printf("Res: ");
    for (uint32_t j = 0; j < buffer.size(); j++) Serial.printf("%02X ", buffer[j]);
    Serial.println();
}

void setup()
{
    pinMode(RS485_SE_PIN, OUTPUT);
    digitalWrite(RS485_SE_PIN, HIGH);

    pinMode(PIN_5V_EN, OUTPUT);
    digitalWrite(PIN_5V_EN, HIGH);

    Serial.begin(256000);

    _vEBus.SetReceiveCallback(Receive, _blacklist, sizeofarray(_blacklist));
    _vEBus.SetLogLevel(VEBus::LogLevel::None);
    _vEBus.Setup();
}

void loop()
{
    _vEBus.Maintain();
    delay(10);
}