#include <Arduino.h>

#include <vector>
#include <VEBusDefinition.h>
#include <VEBus.h>
#include "config.h"

#define sizeofarray(x) sizeof x / sizeof x[0]

VEBus _vEBus(Serial1, RS485_RX_PIN, RS485_TX_PIN, RS485_EN_PIN);

unsigned long lastSendTime = 0;

void Response(VEBus::ResponseData& data)
{
    if (data.command == WinmonCommand::SendSoftwareVersionPart0)
    {
        Serial.printf("Res: id %d, Version: %lu\n", data.id, (unsigned long)data.valueUint32);
        return;
    }

    Serial.printf("Res: id %d, command: %d, address %d ", data.id, data.command, data.address);

    switch (data.dataType)
    {
    case VEBusDefinition::none:
        break;
    case VEBusDefinition::floatingPoint:
        Serial.printf("floatingPoint: %0.2f\n", data.valueFloat);
        break;
    case VEBusDefinition::unsignedInteger:
        Serial.printf("unsignedInteger: %lu\n", (unsigned long)data.valueUint32);
        break;
    case VEBusDefinition::signedInteger:
        Serial.printf("signedInteger: %d\n", data.valueint32);
        break;
    default:
        break;
    }
}

void setup()
{
    pinMode(RS485_SE_PIN, OUTPUT);
    digitalWrite(RS485_SE_PIN, HIGH);

    pinMode(PIN_5V_EN, OUTPUT);
    digitalWrite(PIN_5V_EN, HIGH);

    Serial.begin(256000);

    _vEBus.SetResponseCallback(Response);
    _vEBus.SetLogLevel(VEBus::LogLevel::None);
    _vEBus.Setup();
}

void loop()
{
    _vEBus.Maintain();
    if (millis() - lastSendTime > 5000)
    {
        _vEBus.Read(Settings::Flags0);
        _vEBus.Read(Settings::IMainsLimit);
        auto id = _vEBus.ReadSoftwareVersion();
        Serial.printf("Fifo size: %d, id:%d\n", _vEBus.GetFifoSize(), id);
        lastSendTime = millis();
    }


    delay(10);
}