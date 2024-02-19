#include <Arduino.h>

#include <vector>
#include <VEBusDefinition.h>
#include <VEBus.h>
#include "config.h"

#define sizeofarray(x) sizeof x / sizeof x[0]

VEBus _vEBus(Serial1, RS485_RX_PIN, RS485_TX_PIN, RS485_EN_PIN);

unsigned long lastSendTime = 0;
bool FinishPrinted = false;
uint8_t errors = 0;

//Blacklist for receive callback
VEBus::Blacklist _blacklist[] = { {.value = 0xE4, .at = 4}, {.value = 0x55, .at = 4} };

enum CommandType
{
    Setting = 1,
    RamVar = 2
};

struct InfoType
{
    uint8_t id;
    CommandType command;
    uint8_t address;
};

std::vector<InfoType> _SentInfoReads(32);

void destuffingFAtoFF(std::vector<uint8_t>& buffer)
{
    for (uint8_t i = 4; i < buffer.size(); i++)
    {
        if (buffer[i] == 0xFA && i != buffer.size() - 1)
        {
            buffer[i] = buffer[i + 1] + 0x80;
            buffer.erase(buffer.begin() + i + 1);
        }
    }
}

void Receive(std::vector<uint8_t>& buffer)
{
    destuffingFAtoFF(buffer);
    for (size_t i = 0; i < _SentInfoReads.size(); i++)
    {
        if (0x00 != buffer[4] || _SentInfoReads[i].id != buffer[5]) continue;

        switch (_SentInfoReads[i].command)
        {
        case CommandType::Setting:
        {
            if (buffer.size() == 9) {
                Serial.printf("SettingInfo %d:\t {      0, 0, false}, \\ \n", _SentInfoReads[i].address);
                break;
            }
            if (buffer.size() != 20) {
                Serial.printf("SettingInfo %d wrong size %d\n", _SentInfoReads[i].address, buffer.size());
                for (uint32_t j = 0; j < buffer.size(); j++) Serial.printf("%02X ", buffer[j]);
                Serial.println();
                break;
            }
            SettingInfo settingInfo;
            settingInfo.Scale = ((int16_t)buffer[8] << 8) | buffer[7];
            settingInfo.Offset = ((int16_t)buffer[10] << 8) | buffer[9];
            settingInfo.Default = ((uint16_t)buffer[12] << 8) | buffer[11];
            settingInfo.Minimum = ((uint16_t)buffer[14] << 8) | buffer[13];
            settingInfo.Maximum = ((uint16_t)buffer[16] << 8) | buffer[15];
            settingInfo.AccessLevel = buffer[17];
            Serial.printf("SettingInfo %d:\t {%d, %d, %d, %d, %d, %d, true}, \\ \n",
                _SentInfoReads[i].address,
                settingInfo.Scale,
                settingInfo.Offset,
                settingInfo.Default,
                settingInfo.Minimum,
                settingInfo.Maximum,
                settingInfo.AccessLevel);
        }
            break;
        case CommandType::RamVar:
        {
            if (buffer.size() == 9) {
                Serial.printf("RamVarInfo %d:\t {      0, 0, false}, \\ \n", _SentInfoReads[i].address);
                break;
            }
            if (buffer.size() != 13) {
                Serial.printf("RamVarInfo %d wrong size %d\n", _SentInfoReads[i].address, buffer.size());
                for (uint32_t j = 0; j < buffer.size(); j++) Serial.printf("%02X ", buffer[j]);
                Serial.println();
                break;
            }
            RAMVarInfo ramVarInfo;
            ramVarInfo.Scale = ((int16_t)buffer[8] << 8) | buffer[7];
            ramVarInfo.Offset = ((int16_t)buffer[10] << 8) | buffer[9];
            Serial.printf("RamVarInfo %d:\t {%d, %d, true}, \\ \n", _SentInfoReads[i].address, ramVarInfo.Scale, ramVarInfo.Offset);
        }
            break;
        default:
            Serial.println("Something went wrong");
            break;
        }
        _SentInfoReads.erase(_SentInfoReads.begin() + i);
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

    _vEBus.SetReceiveCallback(Receive, _blacklist, sizeofarray(_blacklist));
    _vEBus.SetLogLevel(VEBus::LogLevel::None);
    _vEBus.Setup();
    _SentInfoReads.clear();
    delay(1000);
    Serial.println("Start all Info requests");

    for (uint8_t i = 0; i < RamVariables::SizeOfRamVarStruct; i++)
    {
        InfoType info;
        info.id = _vEBus.ReadInfo((RamVariables)i);
        if (info.id == 0)
        {
            Serial.printf("No free ID for RamVariables address %d\n", info.id);
            errors++;
            continue;
        }
        info.address = i;
        info.command = CommandType::RamVar;
        _SentInfoReads.push_back(info);
    }

    for (uint8_t i = 0; i < Settings::SizeOfSettingsStruct; i++)
    {
        InfoType info;
        info.id = _vEBus.ReadInfo((Settings)i);
        if (info.id == 0)
        {
            Serial.printf("No free ID for Settings address %d\n", info.id);
            errors++;
            continue;
        }
        info.address = i;
        info.command = CommandType::Setting;
        _SentInfoReads.push_back(info);
    }

}

void loop()
{
    _vEBus.Maintain();
    if (millis() - lastSendTime > 1000 && FinishPrinted == false)
    {
        Serial.printf("Fifo size: %d\n", _vEBus.GetFifoSize());
        lastSendTime = millis();
    }

    if (FinishPrinted == false && _SentInfoReads.empty())
    {
        FinishPrinted = true;
        Serial.printf("Finish read info, %d errors\n", errors);
    }


    delay(10);
}