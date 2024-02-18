#include <VEBusDefinition.h>
#include <VEBus.h>
#include "config.h"

#define sizeofarray(x) sizeof x / sizeof x[0]

VEBus _vEBus(Serial1, RS485_RX_PIN, RS485_TX_PIN, RS485_EN_PIN);

unsigned long lastSendTime = 0;
float value = 5.0f;

//Blacklist for receive callback
VEBus::Blacklist _blacklist[] = { {.value = 0xE4, .at = 4}, {.value = 0x55, .at = 4} };

void Response(VEBus::ResponseData& data)
{
    Serial.printf("Res: id %d, value: %0.2f\n",data.id, data.value);
}

void Receive(std::vector<uint8_t>& buffer)
{
    // std::string message;
    // for (uint32_t p = 0; p < buffer.size(); p++)
    // {
    //     char tempBuffer[6];
    //     sprintf(tempBuffer, "%02X ", buffer[p]);
    //     message += tempBuffer;
    // }
    // Serial.printf("Rec: %s\n",message.c_str());
}

void setup()
{
	//LILYGO T-CAN485 Specific
    pinMode(RS485_SE_PIN, OUTPUT);
    digitalWrite(RS485_SE_PIN, HIGH);
    pinMode(PIN_5V_EN, OUTPUT);
    digitalWrite(PIN_5V_EN, HIGH);

    Serial.begin(256000);

    _vEBus.SetResponseCallback(Response);
    _vEBus.SetReceiveCallback(Receive, _blacklist, sizeofarray(_blacklist));
    _vEBus.SetLogLevel(VEBus::LogLevel::Information);
    _vEBus.Setup();
}

void loop()
{
    _vEBus.Maintain();
    if (millis() - lastSendTime > 5000)
    {
        //_vEBus.WriteViaID(Settings::IMainsLimit, value);
        value += 0.5f;
		//read EEPROM saved value
        _vEBus.Read(Settings::IMainsLimit);
        lastSendTime = millis();
    }
    delay(10);
}