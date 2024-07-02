// VEBusDefinition.h

#ifndef _VEBUSDEFINITION_h
#define _VEBUSDEFINITION_h

//Default for Multiplus-II 12/3000
#define MULTIPLUS_II_12_3000

#include "arduino.h"

namespace VEBusDefinition
{
    enum WinmonCommand : uint8_t
    {
        SendSoftwareVersionPart0 = 0x05,
        SendSoftwareVersionPart1 = 0x06,
        GetSetDeviceState = 0x0E,
        ReadRAMVar = 0x30,
        ReadSetting = 0x31,
        WriteRAMVar = 0x32,
        WriteSetting = 0x33,
        WriteData = 0x34,
        GetSettingInfo = 0x35,
        GetRAMVarInfo = 0x36,
        WriteViaID = 0x37,
        ReadSnapShot = 0x38
    };

    enum CommandDeviceState
    {
        Inquire = 0,
        ForceToEqualise = 1,
        ForceToAbsorption = 2,
        ForceToFloat = 3,
    };

    enum StateDescription
    {
        DeviceDown = 0,
        DeviceStarup = 1,
        DeviceOff = 2,
        DeviceSlaveMode = 3,
        DeviceInvertFull = 4,
        DeviceInvertHalf = 5,
        DeviceInvertAES = 6,
        DevicePowerAssist = 7,
        DeviceBypass = 8,
        DeviceChargeInit = 9,
        DeviceChargeBulk = 10,
        DeviceChargeAbsorption = 11
    };

    enum RamVariables : uint8_t
    {
        UMainsRMS = 0,
        IMainsRMS = 1,
        UInverterRMS = 2,
        IInverterRMS = 3,
        UBat = 4,
        IBat = 5,
        //(= RMS value of ripple voltage)
        UBatRMS = 6,
        //(time - base 0.1s)
        InverterPeriodTime = 7,
        //(time - base 0.1s)
        MainsPeriodTime = 8,
        SignedACLoadCurrent = 9,
        VirtualSwitchPosition = 10,  // No RamVarInfo available
        IgnoreACInputState = 11,
        MultiFunctionalRelayState = 12,
        //(battery monitor function)
        ChargeState = 13,
        //(filtered)
        InverterPower = 14,
        InverterPower2 = 15,
        OutputPower = 16,
        InverterPowerNF = 17,
        InverterPower2NF = 18,
        OutputPowerNF = 19,
        SizeOfRamVarStruct
    };

    enum Settings : uint8_t
    {
        //(see SettingsFlag0)
        Flags0 = 0,
        //(not implemented yet)
        Flags1 = 1,
        UBatAbsorption = 2,
        UBatFloat = 3,
        IBatBulk = 4,
        UInvSetpoint = 5,
        IMainsLimit = 6,
        RepeatedAbsorptionTime = 7,
        RepeatedAbsorptionInterval = 8,
        MaximumAbsorptionDuration = 9,
        ChargeCharacteristic = 10,
        UBatLowLimitForInverter = 11,
        UBatLowHysteresisForInverter = 12,
        NumberOfSlavesConnected = 13,  // No SettingInfo available
        SpecialThreePhaseSetting = 14, // No SettingInfo available
        SizeOfSettingsStruct
    };

    enum SettingsFlag0 : uint8_t
    {
        MultiPhaseSystem = 0,
        MultiPhaseLeader = 1,
        Freq60Hz = 2,
        //(fast input voltage detection). IMPORTANT: Keep SettingsFlag0::InvertedValue consistent.
        DisableWaveCheck = 3,
        DoNotStopAfter10HrBulk = 4,
        AssistEnabled = 5,
        DisableCharge = 6,
        //IMPORTANT : Must have inverted value of SettingsFlag0::DisableWaveCheck.
        InvertedValue = 7,
        DisableAES = 8,
        //NotPromotedOption = 9,
        //NotPromotedOption = 10,

        EnableReducedFloat =11,
        //NotPromotedOption = 12,

        DisableGroundRelay = 13,
        WeakACInput = 14,
        RemoteOverrulesAC2 = 15,
        SizeOfStruct
    };

    enum VariableType : uint8_t
    {
        RamVar = 0x00,
        Setting = 0x01
    };

    enum SwitchState : uint8_t
    {
        Sleep = 0x04,
        ChargerOnly = 0x05,
        //(turn AC-in off!)
        InverterOnly = 0x06,
        //(normal ON mode)
        ChargerInverter = 0x07
    };

    enum ReceivedMessageType
    {
        Unknown = 0,
        Known,
        AcPhaseInformation,
        sync
    };

    enum ResponseDataType
    {
        none = 0,
        floatingPoint,
        unsignedInteger,
        signedInteger
    };

    enum StorageType
    {
        Eeprom = 0x00,
        NoEeprom = 0x02
    };

    struct SettingInfo
    {
        int16_t Scale;
        int16_t Offset;
        uint16_t Default;
        uint16_t Minimum;
        uint16_t Maximum;
        uint8_t AccessLevel;
        bool available;
        ResponseDataType dataType;
    };

    struct RAMVarInfo
    {
        int16_t Scale;
        int16_t Offset;
        bool available;
        ResponseDataType dataType;
    };

    union LEDData
    {
        uint8_t value;
        struct {
            bool MainsOn : 1;
            bool Absorption : 1;
            bool Bulk : 1;
            bool Float : 1;
            bool InverterOn : 1;
            bool Overload : 1;
            bool LowBattery : 1;
            bool Temperature : 1;
        };
    };

    struct MasterMultiLed
    {
        LEDData LEDon;
        //(LEDon=1 && LEDblink=1) = blinking
        //(LEDon=0 && LEDblink=1) = blinking_inverted
        LEDData LEDblink;
        bool LowBattery; //0=ok, 2=battery low
        uint8_t    AcInputConfiguration;
        float   MinimumInputCurrentLimitA;
        float   MaximumInputCurrentLimitA;
        float   ActualInputCurrentLimitA;
        uint8_t    SwitchRegister;
    };

    struct MultiPlusStatus
    {
        float   Temp;
        float   DcCurrentA;
        int16_t BatterieAh;
        bool    DcLevelAllowsInverting;
    };

    enum PhaseInfo
    {
        L4 = 0x05,
        L3 = 0x06,
        L2 = 0x07,
        S_L1 = 0x08,
        S_L2 = 0x09,
        S_L3 = 0x0A,
        S_L4 = 0x0B,
        DC = 0x0C,
    };

    enum PhaseState
    {
        Down = 0x00,
        Startup = 0x01,
        Off = 0x02,
        Slave = 0x03,
        InvertFull = 0x04,
        InvertHalf = 0x05,
        InvertAES = 0x06,
        PowerAssist = 0x07,
        Bypass = 0x08,
        StateCharge = 0x09
    };

    struct DcInfo
    {
        bool newInfo;
        float Voltage;
        float CurrentInverting;
        float CurrentCharging;
        //float InverterFrequency;
        //uint8_t InverterPeriod;

        bool operator==(const DcInfo& a) const {
            return (Voltage == a.Voltage) &&
                (CurrentInverting == a.CurrentInverting) &&
                (CurrentCharging == a.CurrentCharging);//&&
                //(InverterFrequency == a.InverterFrequency) &&
                //(InverterPeriod == a.InverterPeriod);
        }
    };

    struct AcInfo
    {
        bool newInfo;
        PhaseInfo Phase;
        PhaseState State;
        float MainVoltage;
        float MainCurrent;
        float InverterVoltage;
        float InverterCurrent;
        //float MainFrequency;
        //uint8_t MainPeriod;

        bool operator==(const AcInfo& a)
        const {
            return (State == a.State) &&
                (MainVoltage == a.MainVoltage) &&
                (MainCurrent == a.MainCurrent) &&
                (InverterVoltage == a.InverterVoltage) &&
                (InverterCurrent == a.InverterCurrent);//&&
                //(MainFrequency == a.MainFrequency) &&
                //(MainPeriod == a.MainPeriod);
        }
    };

#ifdef MULTIPLUS_II_12_3000
//GetSettingInfo 13 wrong size 9 [83 83 FE 32 00 8D 89 BB FF]
//GetSettingInfo 14 wrong size 9 [83 83 FE 09 00 8E 89 E3 FF]
//                                    sc, offset,default,    min,    max, access,
#define DefaultSettingInfoList  {      1,      0,  35248,      0,  28668,      0,  true, ResponseDataType::unsignedInteger}, \
                                {      2,      0,  19966,      0,  65535,      0,  true, ResponseDataType::unsignedInteger}, \
                                {   -100,      0,   1440,   1200,   1600,      0,  true, ResponseDataType::floatingPoint}, \
                                {   -100,      0,   1380,   1200,   1600,      0,  true, ResponseDataType::floatingPoint}, \
                                {      1,      0,    120,      0,    120,      0,  true, ResponseDataType::floatingPoint}, \
                                {      1,      0,    230,    210,    245,      0,  true, ResponseDataType::floatingPoint}, \
                                {    -10,      0,    320,     10,    500,      0,  true, ResponseDataType::floatingPoint}, \
                                {     15,      0,      4,      1,     96,      0,  true, ResponseDataType::floatingPoint}, \
                                {    360,      0,     28,      1,    180,      0,  true, ResponseDataType::floatingPoint}, \
                                {     60,      0,      8,      1,     24,      0,  true, ResponseDataType::floatingPoint}, \
                                {      1,      0,      3,      1,      3,      0,  true, ResponseDataType::floatingPoint}, \
                                {   -100,      0,    930,    930,   1300,    128,  true, ResponseDataType::floatingPoint}, \
                                {   -100,      0,    160,     25,    600,      0,  true, ResponseDataType::floatingPoint}, \
                                {      0,      0,      0,      0,      0,      0, false, ResponseDataType::none}, \
                                {      0,      0,      0,      0,      0,      0, false, ResponseDataType::none}


//Default for Multiplus-II 12/3000
//GetRAMVarInfo 10 wrong size 9 [83 83 FE 5C 00 8A 8E 8F FF]
//                                    sc, offset,
#define DefaultRamVarInfoList   {  32668,      0,  true, ResponseDataType::floatingPoint}, \
                                { -32668,      0,  true, ResponseDataType::floatingPoint}, \
                                {  32668,      0,  true, ResponseDataType::floatingPoint}, \
                                {  32668,      0,  true, ResponseDataType::floatingPoint}, \
                                {  32668,      0,  true, ResponseDataType::floatingPoint}, \
                                { -32758,      0,  true, ResponseDataType::floatingPoint}, \
                                {  32668,      0,  true, ResponseDataType::floatingPoint}, \
                                {  30815,    256,  true, ResponseDataType::floatingPoint}, \
                                {  31791,      0,  true, ResponseDataType::floatingPoint}, \
                                { -32668,      0,  true, ResponseDataType::floatingPoint}, \
                                {      0,      0, false, ResponseDataType::none}, \
                                {      5, -32768,  true, ResponseDataType::floatingPoint}, \
                                {      6, -32768,  true, ResponseDataType::floatingPoint}, \
                                {  32568,      0,  true, ResponseDataType::floatingPoint}, \
                                {     -1,      0,  true, ResponseDataType::floatingPoint}, \
                                {     -1,      0,  true, ResponseDataType::floatingPoint}, \
                                {     -1,      0,  true, ResponseDataType::floatingPoint}, \
                                {     -1,      0,  true, ResponseDataType::floatingPoint}, \
                                {     -1,      0,  true, ResponseDataType::floatingPoint}, \
                                {     -1,      0,  true, ResponseDataType::floatingPoint },
#endif // MULTIPLUS_II_12_3000
}

#endif

