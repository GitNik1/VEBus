/*
 Name:		VEBus.h
 Created:	07.02.2024 20:07:36
 Author:	nriedle
*/

#ifndef _VEBus_h
#define _VEBus_h

#include "arduino.h"


#define UART_MODE_RS485
#ifdef UART_MODE_RS485
#include "hal/uart_types.h"
#include <vector>
#endif


//#include <stdint.h>
#include "VEBusDefinition.h"

using namespace VEBusDefinition;

class VEBus
{
public:
    enum LogLevel
    {
        None,
        Warning,
        Information,
        Debug
    };

    struct ResponseData
    {
        uint8_t id;
        float valueFloat;
        uint32_t valueUint32;
    };

    struct Blacklist
    {
        uint8_t value;
        uint8_t at;
    };

    enum RequestError
    {
        Success,
        FifoFull,
        OutsideLowerRange,
        OutsideUpperRange,
        ConvertError
    };

    struct RequestResult
    {
        uint8_t id;
        RequestError error;
    };

    friend void communication_task(void* handler_args);

    VEBus(HardwareSerial& serial, int8_t rxPin, int8_t txPin, int8_t rePin);
    ~VEBus();

    void Setup(bool autostart = true);
    void Maintain();

    void SetLogLevel(LogLevel level);
    LogLevel GetLogLevel();

    void SetResponseCallback(std::function<void(ResponseData&)> cb);
    void SetReceiveCallback(std::function<void(std::vector<uint8_t>& buffer)> cb);
    void SetReceiveCallback(std::function<void(std::vector<uint8_t>& buffer)> cb, Blacklist* blacklist, size_t size);

    void StartCommunication();
    void StopCommunication();
    uint32_t GetFifoSize();

    //*Be careful when repeatedly writing EEPROM (loop)
    //*EEPROM writes are limited
    //*Returns 0 if failed
    uint8_t WriteViaID(RamVariables variable, int16_t rawValue, bool eeprom = false);
    uint8_t WriteViaID(RamVariables variable, uint16_t rawValue, bool eeprom = false);
    RequestResult WriteViaID(RamVariables variable, float value, bool eeprom = false);

    uint8_t WriteViaID(Settings setting, int16_t rawValue, bool eeprom = false);
    uint8_t WriteViaID(Settings setting, uint16_t rawValue, bool eeprom = false);
    RequestResult WriteViaID(Settings setting, float value, bool eeprom = false);

    //TODO test
    uint8_t Write(Settings setting, uint16_t rawValue);
    uint8_t Write(RamVariables variable, uint16_t rawValue);

    //*Read EEPROM saved Value
    //*Returns 0 if failed
    uint8_t Read(RamVariables variable);
    uint8_t Read(RamVariables* variable, uint8_t size);
    uint8_t Read(Settings setting);

    uint8_t ReadInfo(RamVariables variable);
    uint8_t ReadInfo(Settings setting);

    void SetSwitch(SwitchState state);

    RAMVarInfo GetRamVarInfo(RamVariables variable);
    SettingInfo GetSettingInfo(Settings setting);

    bool NewMasterMultiLedAvailable();
    MasterMultiLed GetMasterMultiLed();

    bool NewMultiPlusStatusAvailable();
    MultiPlusStatus GetMultiPlusStatus();

    //Get VE.BUS Version
    uint8_t ReadSoftwareVersion();

    void DestuffingFAtoFF(std::vector<uint8_t>& buffer);

private:
    struct Data
    {
        bool responseExpected;
        bool IsSent = false;
        bool IsLogged = false;
        uint8_t id = 0;
        uint8_t command;
        uint8_t address;
        uint8_t expectedResponseCode = 0;
        uint32_t sentTimeMs;
        uint32_t resendCount = 0;
        std::vector<uint8_t> requestData;
        std::vector<uint8_t> responseData;
        Data() : requestData(32), responseData(32){}
    };

    HardwareSerial& _serial;
    SemaphoreHandle_t _semaphoreDataFifo;
    SemaphoreHandle_t _semaphoreStatus;
    SemaphoreHandle_t _semaphoreReceiveData;
    int8_t _rxPin, _txPin, _rePin;
    uint8_t _id;
    std::vector<Data> _dataFifo;
    //Runs on core 0. not thread save.
    std::vector<uint8_t> _receiveBuffer;
    std::vector<std::vector<uint8_t>> _receiveBufferList;
    SettingInfo _settingInfoList[Settings::SizeOfSettingsStruct] = { DefaultSettingInfoList };
    RAMVarInfo _ramVarInfoList[RamVariables::SizeOfRamVarStruct] = { DefaultRamVarInfoList };
    Blacklist _blacklist[20];
    size_t _blacklistSize = 0;

    MasterMultiLed _masterMultiLedCore0;
    MasterMultiLed _masterMultiLed;
    bool _masterMultiLedLogged = false;

    MultiPlusStatus _multiPlusStatusCore0;
    MultiPlusStatus _multiPlusStatus;
    bool _multiPlusStatusLogged = false;

    LogLevel _logLevel = LogLevel::None;

    std::function<void(ResponseData&)> _onResponseCb;
    std::function<void(std::vector<uint8_t>&)> _onReceiveCb;

    bool _communitationIsRunning = false;
    volatile bool _communitationIsResumed = false;

    void addOrUpdateFifo(Data data, bool updateIfExist = true);
    bool getNextFreeId_1(uint8_t& id);

    void prepareCommand(std::vector<uint8_t>& buffer, uint8_t frameNr);
    void prepareCommandWriteViaID(std::vector<uint8_t>& buffer, uint8_t id, uint8_t winmonCommand, uint8_t address, int16_t value, StorageType storageType);
    void prepareCommandWriteViaID(std::vector<uint8_t>& buffer, uint8_t id, uint8_t winmonCommand, uint8_t address, uint16_t value, StorageType storageType);
    uint8_t prepareCommandReadMultiRAMVar(std::vector<uint8_t>& buffer, uint8_t id, uint8_t* addresses, uint8_t addressSize);
    void prepareCommandReadSetting(std::vector<uint8_t>& buffer, uint8_t id, uint16_t address);
    void prepareCommandWriteAddress(std::vector<uint8_t>& buffer, uint8_t id, uint8_t winmonCommand, uint16_t address);
    void prepareCommandWriteData(std::vector<uint8_t>& buffer, uint8_t id, uint16_t value);
    void prepareCommandReadInfo(std::vector<uint8_t>& buffer, uint8_t id, uint8_t winmonCommand, uint16_t setting);
    void prepareCommandReadSoftwareVersion(std::vector<uint8_t>& buffer, uint8_t id, uint8_t winmonCommand);

    void prepareCommandSetSwitchState(std::vector<uint8_t>& buffer, SwitchState switchState);

    void stuffingFAtoFF(std::vector<uint8_t>& buffer);
    void appendChecksum(std::vector<uint8_t>& buffer);

    uint16_t convertRamVarToRawValue(RamVariables variable, float value);
    float convertRamVarToValue(RamVariables variable, uint16_t rawValue);

    int16_t convertRamVarToRawValueSigned(RamVariables variable, float value);
    float convertRamVarToValueSigned(RamVariables variable, int16_t rawValue);

    uint16_t convertSettingToRawValue(Settings setting, float value);
    float convertSettingToValue(Settings setting, uint16_t rawValue);

    ReceivedMessageType decodeVEbusFrame(std::vector<uint8_t>& buffer);
    void decodeChargerInverterCondition(std::vector<uint8_t>& buffer); //0x80
    void decodeBatteryCondition(std::vector<uint8_t>& buffer); //0x70
    void decodeMasterMultiLed(std::vector<uint8_t>& buffer); //0x41
    void saveSettingInfoData(Data& data);
    void saveRamVarInfoData(Data& data);
    void commandHandling();

    void sendData(VEBus::Data& data, uint8_t& frameNr);
    void checkResponseMessage();
    void saveResponseData(Data data);
    void saveStatusFromCore0();
    void garbageCollector();
    void logging();
};
#endif

