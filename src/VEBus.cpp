/*
 Name:		VEBus.cpp
 Created:	07.02.2024 20:07:36
 Author:	nriedle
*/

#include "VEBus.h"

#define NEXT_FRAME_NR(x) (x + 1) & 0x7F
#define MK3_ID_0 0x98 //8F?
#define MK3_ID_1 0xF7
#define MP_ID_0 0x83
#define MP_ID_1 0x83
#define SYNC_BYTE 0x55
#define SYNC_FRAME 0xFD
#define DATA_FRAME 0xFE
#define END_OF_FRAME 0xFF
#define LOW_BATTERY 0x02

#define VEBUS_BAUD 256000

#define RESPONSE_TIMEOUT 10000
#define MAX_RESEND 2

//Runs on core 0
void communication_task(void* handler_args)
{
	auto mk3Instance = static_cast<VEBus*>(handler_args);

	while (true)
	{
		mk3Instance->commandHandling();
		//vTaskDelay(1 / portTICK_RATE_MS);
		taskYIELD();
	}
}

VEBus::VEBus(HardwareSerial& serial, int8_t rxPin, int8_t txPin, int8_t rePin) :
	_serial(serial),
	_rxPin(rxPin),
	_txPin(txPin),
	_rePin(rePin)
{
	_semaphoreDataFifo = xSemaphoreCreateMutex();
	_semaphoreStatus = xSemaphoreCreateMutex();
	_semaphoreReceiveData = xSemaphoreCreateMutex();
	SetReceiveCallback([this](std::vector<uint8_t>&) {});
	SetResponseCallback([this](ResponseData&) {});
	_dataFifo.reserve(10);
	_receiveBufferList.reserve(10);
}

VEBus::~VEBus()
{
}

void VEBus::Setup(bool autostart)
{

	_serial.begin(VEBUS_BAUD, SERIAL_8N1, _rxPin, _txPin);
#ifdef UART_MODE_RS485
	_serial.setPins(-1, -1, -1, _rePin);
	_serial.setMode(UART_MODE_RS485_HALF_DUPLEX);

#elif
	pinMode(_rePin, OUTPUT);
	digitalWrite(_rePin, LOW);
#endif

	if (autostart) StartCommunication();
	xTaskCreatePinnedToCore(communication_task, "vebus_task", 4096, this, 0, NULL, 0);   //Priority 0, CPU 0
}

void VEBus::Maintain()
{
	logging();
	garbageCollector();
	checkResponseMessage();

	xSemaphoreTake(_semaphoreReceiveData, portMAX_DELAY);
	while (!_receiveBufferList.empty())
	{
		_onReceiveCb(_receiveBufferList.front());
		_receiveBufferList.erase(_receiveBufferList.begin());
	}
	xSemaphoreGive(_semaphoreReceiveData);
}

void VEBus::SetLogLevel(LogLevel level)
{
	_logLevel = level;
}

VEBus::LogLevel VEBus::GetLogLevel()
{
	return _logLevel;
}

void VEBus::SetResponseCallback(std::function<void(ResponseData&)> cb)
{
	_onResponseCb = cb;
}

void VEBus::SetReceiveCallback(std::function<void(std::vector<uint8_t>&)> cb)
{
	_onReceiveCb = cb;
}

void VEBus::SetReceiveCallback(std::function<void(std::vector<uint8_t>&)> cb, Blacklist* blacklist, size_t size)
{
	if (size > 20) size = 20;
	for (size_t i = 0; i < size; i++) _blacklist[i] = blacklist[i];
	_blacklistSize = size;
	_onReceiveCb = cb;
}

void VEBus::SetReceiveCallback(std::function<void(std::vector<uint8_t>&)> cb, Whitelist* whitelist, size_t size)
{
	if (size > 20) size = 20;
	for (size_t i = 0; i < size; i++) _whitelist[i] = whitelist[i];
	_whitelistSize = size;
	_onReceiveCb = cb;
}

void VEBus::StartCommunication()
{
	_communitationIsRunning = true;
	_communitationIsResumed = true;

}

void VEBus::StopCommunication()
{
	_communitationIsRunning = false;
}

uint32_t VEBus::GetFifoSize()
{
	return _dataFifo.size();
}

char rxbuf[256];

uint8_t VEBus::WriteViaID(RamVariables variable, int16_t rawValue, bool eeprom)
{
	Data data;
	if (!getNextFreeId_1(data.id)) return 0;
	data.responseExpected = true;
	data.command = WinmonCommand::WriteRAMVar;
	data.address = variable;
	data.expectedResponseCode = 0x87;
	StorageType storageType = (eeprom == false) ? StorageType::NoEeprom : StorageType::Eeprom;
	prepareCommandWriteViaID(data.requestData, data.id, data.command, variable, rawValue, storageType);
	addOrUpdateFifo(data);
	return data.id;
}

uint8_t VEBus::WriteViaID(RamVariables variable, uint16_t rawValue, bool eeprom)
{
	Data data;
	if (!getNextFreeId_1(data.id)) return 0;
	data.responseExpected = true;
	data.command = WinmonCommand::WriteRAMVar;
	data.address = variable;
	data.expectedResponseCode = 0x87;
	StorageType storageType = (eeprom == false) ? StorageType::NoEeprom : StorageType::Eeprom;
	prepareCommandWriteViaID(data.requestData, data.id, data.command, variable, rawValue, storageType);
	addOrUpdateFifo(data);
	return data.id;
}

VEBus::RequestResult VEBus::WriteViaID(RamVariables variable, float value, bool eeprom)
{
	if (!_ramVarInfoList[variable].available) return {0 , RequestError::ConvertError };

	uint16_t UnsignedRawValue;
	int16_t signedRawValue;
	uint8_t id;

	if (_ramVarInfoList[variable].Scale < 0)
	{
		signedRawValue = convertRamVarToRawValueSigned(variable, value);
		id = WriteViaID(variable, signedRawValue, eeprom);
	}
	else
	{
		UnsignedRawValue = convertRamVarToRawValue(variable, value);
		id = WriteViaID(variable, UnsignedRawValue, eeprom);
	}

	if (id == 0) return { 0, RequestError::FifoFull };
	return { id , RequestError::Success };
}

uint8_t VEBus::WriteViaID(Settings setting, int16_t rawValue, bool eeprom)
{
	Data data;
	if (!getNextFreeId_1(data.id)) return 0;
	data.responseExpected = true;
	data.command = WinmonCommand::WriteSetting;
	data.address = setting;
	data.expectedResponseCode = 0x88;
	StorageType storageType = (eeprom == false) ? StorageType::NoEeprom : StorageType::Eeprom;
	prepareCommandWriteViaID(data.requestData, data.id, data.command, setting, rawValue, storageType);
	addOrUpdateFifo(data);

	return data.id;
}

uint8_t VEBus::WriteViaID(Settings setting, uint16_t rawValue, bool eeprom)
{
	Data data;
	if (!getNextFreeId_1(data.id)) return 0;
	data.responseExpected = true;
	data.command = WinmonCommand::WriteSetting;
	data.address = setting;
	data.expectedResponseCode = 0x88;
	StorageType storageType = (eeprom == false) ? StorageType::NoEeprom : StorageType::Eeprom;
	prepareCommandWriteViaID(data.requestData, data.id, data.command, setting, rawValue, storageType);
	addOrUpdateFifo(data);
	return data.id;
}

VEBus::RequestResult VEBus::WriteViaID(Settings setting, float value, bool eeprom)
{
	uint16_t rawValue = convertSettingToRawValue(setting, value);
	if (_settingInfoList[setting].Maximum < rawValue) return {0, RequestError::OutsideUpperRange};
	if (_settingInfoList[setting].Minimum > rawValue) return {0, RequestError::OutsideLowerRange};
	uint8_t id = WriteViaID(setting, rawValue, eeprom);
	if(id == 0) return { 0, RequestError::FifoFull };
	return { id , RequestError::Success };
}

uint8_t VEBus::Write(Settings setting, uint16_t value)
{
	Data data;
	if (!getNextFreeId_1(data.id)) return 0;
	data.responseExpected = false;
	data.command = WinmonCommand::WriteSetting;
	data.address = setting;
	prepareCommandWriteAddress(data.requestData, data.id, data.command, setting);
	addOrUpdateFifo(data, false);

	data.responseExpected = true;
	data.command = WinmonCommand::WriteData;
	data.address = setting;
	data.expectedResponseCode = 0x88;
	prepareCommandWriteData(data.requestData, data.id, value);
	addOrUpdateFifo(data , false);
	return data.id;
}

uint8_t VEBus::Write(RamVariables variable, uint16_t value)
{
	Data data;
	if (!getNextFreeId_1(data.id)) return 0;
	data.responseExpected = false;
	data.command = WinmonCommand::WriteRAMVar;
	data.address = variable;
	prepareCommandWriteAddress(data.requestData, data.id, data.command, variable);
	addOrUpdateFifo(data);

	data.responseExpected = true;
	data.command = WinmonCommand::WriteData;
	data.expectedResponseCode = 0x87;
	data.address = variable;
	prepareCommandWriteData(data.requestData, data.id, value);
	addOrUpdateFifo(data);
	return data.id;
}

uint8_t VEBus::Read(RamVariables variable)
{
	Data data;
	if (!getNextFreeId_1(data.id)) return 0;
	data.responseExpected = true;
	data.command = WinmonCommand::ReadRAMVar;
	data.address = variable;
	data.expectedResponseCode = 0x85;
	uint8_t address[] = { variable };
	prepareCommandReadMultiRAMVar(data.requestData, data.id, address, 1);
	addOrUpdateFifo(data);
	return data.id;
}

//* variable size up to 6
uint8_t VEBus::Read(RamVariables* variable, uint8_t size)
{
	if (size > 6) return 0;
	Data data;
	uint8_t addresses[6]{};
	for (uint8_t i = 0; i < size; i++) addresses[i] = variable[i];

	if (!getNextFreeId_1(data.id)) return 0;


	data.responseExpected = true;
	data.command = WinmonCommand::ReadRAMVar;
	data.address = variable[0]; //TODO
	data.expectedResponseCode = 0x85;
	prepareCommandReadMultiRAMVar(data.requestData, data.id, addresses, size);
	addOrUpdateFifo(data);
	return data.id;
}

// Command: 0x31 < Lo(Setting ID) > <Hi(Setting ID)> 
// Response : 0x86 / 91 < Lo(Value) > <Hi(Value)>  
// <Value> is an unsigned 16 - bit quantity.  
// 0x86 = SettingReadOK. 
// 0x91 = Setting not supported(in which case <Value> is not valid).
uint8_t VEBus::Read(Settings setting)
{
	Data data;
	if (!getNextFreeId_1(data.id)) return 0;
	data.responseExpected = true;
	data.command = WinmonCommand::ReadSetting;
	data.address = setting;
	data.expectedResponseCode = 0x86;
	prepareCommandReadSetting(data.requestData, data.id, setting);
	addOrUpdateFifo(data);
	return data.id;
}

uint8_t VEBus::ReadInfo(RamVariables variable)
{
	Data data;
	if (!getNextFreeId_1(data.id)) return 0;
	data.responseExpected = true;
	data.command = WinmonCommand::GetRAMVarInfo;
	data.address = variable;
	data.expectedResponseCode = 0x8E;
	prepareCommandReadInfo(data.requestData, data.id, data.command, variable);
	addOrUpdateFifo(data);
	return data.id;
}

uint8_t VEBus::ReadInfo(Settings setting)
{
	Data data;
	if (!getNextFreeId_1(data.id)) return 0;
	data.responseExpected = true;
	data.command = WinmonCommand::GetSettingInfo;
	data.address = setting;
	data.expectedResponseCode = 0x89;
	prepareCommandReadInfo(data.requestData, data.id, data.command, setting);
	addOrUpdateFifo(data);
	return data.id;
}

void VEBus::SetSwitch(SwitchState state)
{
	Data data;
	data.responseExpected = false;
	//data.command //add stwichCommandNr
	prepareCommandSetSwitchState(data.requestData, state);
	addOrUpdateFifo(data);
}

RAMVarInfo VEBus::GetRamVarInfo(RamVariables variable)
{
	return _ramVarInfoList[variable];
}

SettingInfo VEBus::GetSettingInfo(Settings setting)
{
	return _settingInfoList[setting];
}

bool VEBus::NewMasterMultiLedAvailable()
{
	return _masterMultiLedNewData;
}

MasterMultiLed VEBus::GetMasterMultiLed()
{
	MasterMultiLed multiLed;
	xSemaphoreTake(_semaphoreStatus, portMAX_DELAY);
	_masterMultiLedNewData = false;
	multiLed = _masterMultiLed;
	xSemaphoreGive(_semaphoreStatus);

	return multiLed;
}

bool VEBus::NewMultiPlusStatusAvailable()
{
	return _multiPlusStatusNewData;
}

MultiPlusStatus VEBus::GetMultiPlusStatus()
{
	MultiPlusStatus multiStatus;
	xSemaphoreTake(_semaphoreStatus, portMAX_DELAY);
	_multiPlusStatusNewData = false;
	multiStatus = _multiPlusStatus;
	xSemaphoreGive(_semaphoreStatus);

	return multiStatus;
}

bool VEBus::NewDcInfoAvailable()
{
	return _dcInfo.newInfo;
}

DcInfo VEBus::GetDcInfo()
{
	DcInfo info;
	xSemaphoreTake(_semaphoreStatus, portMAX_DELAY);
	_dcInfo.newInfo = false;
	info = _dcInfo;
	xSemaphoreGive(_semaphoreStatus);
	return info;
}

AcInfo VEBus::GetAcInfo(uint8_t type)
{
	AcInfo info;
	xSemaphoreTake(_semaphoreStatus, portMAX_DELAY);
	for (auto& element : _acInfo) {
		if (element.Phase != type) continue;
		element.newInfo = false;
		info = element;
		break;
	}
	xSemaphoreGive(_semaphoreStatus);
	return info;
}

uint8_t VEBus::NewAcInfoAvailable()
{
	uint8_t phaseId = 0;
	for (auto& element : _acInfo) {
		if (element.newInfo == false) continue;
		phaseId = element.Phase;
		break;
	}

	return phaseId;
}

uint8_t VEBus::ReadSoftwareVersion()
{
	Data data;
	if (!getNextFreeId_1(data.id)) return 0;
	data.responseExpected = true;
	data.command = WinmonCommand::SendSoftwareVersionPart0;
	data.address = 0;
	data.expectedResponseCode = 0x82;
	prepareCommandReadSoftwareVersion(data.requestData, data.id, data.command);
	addOrUpdateFifo(data);
	return data.id;
}

uint8_t VEBus::CommandReadDeviceState()
{
	Data data;
	if (!getNextFreeId_1(data.id)) return 0;
	data.responseExpected = true;
	data.command = WinmonCommand::GetSetDeviceState;
	data.address = 0;
	data.expectedResponseCode = 0x94;
	prepareCommandSetGetDeviceState(data.requestData, data.id, CommandDeviceState::Inquire);
	addOrUpdateFifo(data);
	return data.id;
}

void VEBus::addOrUpdateFifo(Data data, bool updateIfExist)
{
	data.responseData.clear();
	data.sentTimeMs = millis();

	xSemaphoreTake(_semaphoreDataFifo, portMAX_DELAY);
	if (updateIfExist)
	{
		for (auto& element : _dataFifo) {

			if (element.address == data.address && element.command == data.command)
			{
				element = data;
				xSemaphoreGive(_semaphoreDataFifo);
				return;
			}
		}
	}
	
	_dataFifo.push_back(data);
	xSemaphoreGive(_semaphoreDataFifo);
}

//possible ID_1 between 0x80 and 0xFF (0xE4-0xE7 used from Venus OS)
//* return false if no ID free
bool VEBus::getNextFreeId_1(uint8_t& id)
{
	bool idUsed = false;
	for (uint8_t i = 0; i < 127; i++)
	{
		++_id;
		if (_id < 0x80) _id = 0x80;

		for (auto& element : _dataFifo) {

			if (element.id == _id)
			{
				idUsed = true;
				break;
			}
		}

		if (!idUsed)
		{
			id = _id;
			return true;
		}
	}

	return false;
}

void VEBus::prepareCommand(std::vector<uint8_t>& buffer, uint8_t frameNr)
{
	buffer.insert(buffer.begin(), NEXT_FRAME_NR(frameNr));
	buffer.insert(buffer.begin(), DATA_FRAME);
	buffer.insert(buffer.begin(), MK3_ID_1);
	buffer.insert(buffer.begin(), MK3_ID_0);
}

void VEBus::prepareCommandWriteViaID(std::vector<uint8_t>& buffer, uint8_t id, uint8_t winmonCommand, uint8_t address, int16_t value, StorageType storageType)
{
	buffer.clear();
	buffer.push_back(0x00);
	buffer.push_back(id);
	buffer.push_back(WinmonCommand::WriteViaID);
	buffer.push_back(((winmonCommand == WinmonCommand::WriteRAMVar) ? VariableType::RamVar : VariableType::Setting) | storageType); // 0x02 -> no eeprom write
	buffer.push_back(address);
	buffer.push_back(value & 0xFF);
	buffer.push_back(value >> 8);
}

void VEBus::prepareCommandWriteViaID(std::vector<uint8_t>& buffer, uint8_t id, uint8_t winmonCommand, uint8_t address, uint16_t value, StorageType storageType)
{
	buffer.clear();
	buffer.push_back(0x00);
	buffer.push_back(id);
	buffer.push_back(WinmonCommand::WriteViaID);
	buffer.push_back(((winmonCommand == WinmonCommand::WriteRAMVar) ? VariableType::RamVar : VariableType::Setting) | storageType); // 0x02 -> no eeprom write
	buffer.push_back(address);
	buffer.push_back(value & 0xFF);
	buffer.push_back(value >> 8);
}

// * address size up to 6
// Response: 0x85 / 0x90 < Lo(Value) > < Hi(Value)>  
// 0x85 = RamReadOK. 
// 0x90 = Variable not supported(in which case <Value> is not valid).
uint8_t VEBus::prepareCommandReadMultiRAMVar(std::vector<uint8_t>& buffer, uint8_t id, uint8_t* addresses, uint8_t addressSize)
{
	buffer.clear();
	buffer.push_back(0x00);
	buffer.push_back(id);
	buffer.push_back(WinmonCommand::ReadRAMVar);
	for (uint8_t i = 0; i < addressSize; i++)
	{
		buffer.push_back(addresses[i]);
	}

	return addressSize;
}

void VEBus::prepareCommandReadSetting(std::vector<uint8_t>& buffer, uint8_t id, uint16_t address)
{
	buffer.clear();
	buffer.push_back(0x00);
	buffer.push_back(id);
	buffer.push_back(WinmonCommand::ReadSetting);
	buffer.push_back(address & 0xFF);
	buffer.push_back(address >> 8);
}

// This command must be followed by writeData (CommandWriteData). 
// Response: None
void VEBus::prepareCommandWriteAddress(std::vector<uint8_t>& buffer, uint8_t id, uint8_t winmonCommand, uint16_t address)
{
	buffer.clear();
	buffer.push_back(0x00);
	buffer.push_back(id);
	buffer.push_back(winmonCommand);
	buffer.push_back(address & 0xFF);
	buffer.push_back(address >> 8);
}

// This command must be after writeAddress (CommandWriteSetting or CommandWriteRAMVar).
// Response:  0x87 / 0x88 XX XX  
// 0x87 = successful RAM write. 
// 0x88 = successful setting write.
void VEBus::prepareCommandWriteData(std::vector<uint8_t>& buffer, uint8_t id, uint16_t value)
{
	buffer.clear();
	buffer.push_back(0x00);
	buffer.push_back(id);
	buffer.push_back(WinmonCommand::WriteData);
	buffer.push_back(value & 0xFF);
	buffer.push_back(value >> 8);
}

void VEBus::prepareCommandReadInfo(std::vector<uint8_t>& buffer, uint8_t id, uint8_t winmonCommand, uint16_t setting)
{
	buffer.clear();
	buffer.push_back(0x00);
	buffer.push_back(id);
	buffer.push_back(winmonCommand);
	buffer.push_back(setting & 0xFF);
	buffer.push_back(setting >> 8);
}

//long Winmon frames
void VEBus::prepareCommandReadSoftwareVersion(std::vector<uint8_t>& buffer, uint8_t id, uint8_t winmonCommand)
{
	buffer.clear();
	buffer.push_back(0x00);
	buffer.push_back(id);
	buffer.push_back(winmonCommand);
}

void VEBus::prepareCommandSetGetDeviceState(std::vector<uint8_t>& buffer, uint8_t id, CommandDeviceState command, uint8_t state)
{
	buffer.clear();
	buffer.push_back(0x00);
	buffer.push_back(id);
	buffer.push_back(WinmonCommand::GetSetDeviceState);
	buffer.push_back(command);
	buffer.push_back(state);
}

//prepareCommand without ID
void VEBus::prepareCommandSetSwitchState(std::vector<uint8_t>& buffer, SwitchState switchState)
{
	buffer.clear();
	buffer.push_back(0x3F);
	buffer.push_back(switchState);
	buffer.push_back(0x00);
	buffer.push_back(0x00);
	buffer.push_back(0x00);
}

void VEBus::stuffingFAtoFF(std::vector<uint8_t>& buffer)
{
	for (uint8_t i = 4; i < buffer.size(); i++)
	{
		if (buffer[i] >= 0xFA)
		{
			buffer[i] = 0x70 | (buffer[i] & 0x0F);
			buffer.insert(buffer.begin() + i, 0xFA);
		}
	}
}

void VEBus::DestuffingFAtoFF(std::vector<uint8_t>& buffer)
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

void VEBus::appendChecksum(std::vector<uint8_t>& buffer)
{
	//calculate checksum without MK3_ID
	uint8_t cs = 1;
	if (buffer.size() < 2) return;

	for (int i = 2; i < buffer.size(); i++) cs -= buffer[i];

	if (cs >= 0xFB)
	{
		buffer.push_back(0xFA);
		buffer.push_back(cs - 0xFA);
	}
	else
	{
		buffer.push_back(cs);
	}

	buffer.push_back(END_OF_FRAME);  //End Of Frame
}

uint16_t VEBus::convertRamVarToRawValue(RamVariables variable, float value)
{
	uint16_t rawValue;
	int16_t scale = abs(_ramVarInfoList[variable].Scale);
	if (scale >= 0x4000) scale = (0x8000 - scale);

	rawValue = value * (scale + 0.0f);
	rawValue -= _ramVarInfoList[variable].Offset;
	return rawValue;
}

float VEBus::convertRamVarToValue(RamVariables variable, uint16_t rawValue)
{
	float value;
	int16_t scale = abs(_ramVarInfoList[variable].Scale);
	if (scale >= 0x4000) scale = (0x8000 - scale);

	value = rawValue / (scale + 0.0f);
	value += _ramVarInfoList[variable].Offset;
	return value;
}

int16_t VEBus::convertRamVarToRawValueSigned(RamVariables variable, float value)
{
	int16_t rawValue;
	int16_t scale = abs(_ramVarInfoList[variable].Scale);
	if (scale >= 0x4000) scale = (0x8000 - scale);

	rawValue = value * (scale + 0.0f);
	rawValue -= _ramVarInfoList[variable].Offset;
	return rawValue;
}

float VEBus::convertRamVarToValueSigned(RamVariables variable, int16_t rawValue)
{
	float value;
	int16_t scale = abs(_ramVarInfoList[variable].Scale);
	if (scale >= 0x4000) scale = (0x8000 - scale);

	value = rawValue / (scale + 0.0f);
	value += _ramVarInfoList[variable].Offset;
	return value;
}

uint16_t VEBus::convertSettingToRawValue(Settings setting, float value)
{
	uint16_t rawValue;
	if (_settingInfoList[setting].Scale > 0) rawValue =  (uint16_t)(value / _settingInfoList[setting].Scale);
	else rawValue = (uint16_t)(value / (1.0f / -(_settingInfoList[setting].Scale)));

	rawValue -= _settingInfoList[setting].Offset;
	return rawValue;
}

float VEBus::convertSettingToValue(Settings setting, uint16_t rawValue)
{
	float value;
	if (_settingInfoList[setting].Scale > 0) value = rawValue * _settingInfoList[setting].Scale;
	else value = rawValue * (1.0f / -(_settingInfoList[setting].Scale));

	value += _settingInfoList[setting].Offset;
	return value;
}










//Runs on core 0
ReceivedMessageType VEBus::decodeVEbusFrame(std::vector<uint8_t>& buffer)
{
	ReceivedMessageType result = ReceivedMessageType::Unknown;
	if ((buffer[0] != MP_ID_0) || (buffer[1] != MP_ID_1)) return ReceivedMessageType::Unknown;
	if ((buffer[2] == SYNC_FRAME) && (buffer.size() == 10) && (buffer[4] == SYNC_BYTE)) return ReceivedMessageType::sync;
	if (buffer[2] != DATA_FRAME) return ReceivedMessageType::Unknown;

	switch (buffer[4]) {
	case 0x00:
	{
		if (buffer.size() < 6) return ReceivedMessageType::Unknown;
		xSemaphoreTake(_semaphoreDataFifo, portMAX_DELAY);
		for (uint8_t i = 0; i < _dataFifo.size(); i++)
		{
			if (_dataFifo[i].id != buffer[5]) continue;
			_dataFifo[i].responseData = buffer;
			break;
		}
		xSemaphoreGive(_semaphoreDataFifo);
		break;
	}
	case 0x20: //Info Frame
	{
		if (buffer.size() < 20) return ReceivedMessageType::Unknown;
		decodeInfoFrame(buffer);
		break;
	}
	case 0x41:
	{
		if ((buffer.size() == 19) && (buffer[5] == 0x10))
		{
			decodeMasterMultiLed(buffer);
			result = ReceivedMessageType::Known;
		}
		break;
	}
	case 0x70:
	{
		if ((buffer.size() == 15) && (buffer[5] == 0x81) && (buffer[6] == 0x64) && (buffer[7] == 0x14) && (buffer[8] == 0xBC) && (buffer[9] == 0x02) && (buffer[12] == 0x00))
		{
			decodeBatteryCondition(buffer);
			result = ReceivedMessageType::Known;
		}
		break;
	}
	case 0x80:
	{
		decodeChargerInverterCondition(buffer);
		result = ReceivedMessageType::Known;
		break;
	}
	case 0xE4:
	{
		if (buffer.size() == 21) result = ReceivedMessageType::AcPhaseInformation;
		break;
	}
	}

	return result;
}

void VEBus::decodeChargerInverterCondition(std::vector<uint8_t>& buffer)
{
	if ((buffer.size() == 19) && (buffer[5] == 0x80) && ((buffer[6] & 0xFE) == 0x12) && (buffer[8] == 0x80) && ((buffer[11] & 0x10) == 0x10) && (buffer[12] == 0x00))
	{
		if (_masterMultiLed.LowBattery != (buffer[7] == LOW_BATTERY))
		{
			xSemaphoreTake(_semaphoreStatus, portMAX_DELAY);
			_masterMultiLed.LowBattery = (buffer[7] == LOW_BATTERY);
			_masterMultiLedNewData = true;
			_masterMultiLedLogged = false;
			xSemaphoreGive(_semaphoreStatus);

		}

		bool dcLevelAllowsInverting = (buffer[6] & 0x01);
		float dcCurrentA = (((uint16_t)buffer[10] << 8) | buffer[9]) / 10.0f;
		float temp = 0;
		if ((buffer[11] & 0xF0) == 0x30) temp = buffer[15] / 10.0f;

		bool newValue = false;
		newValue |= _multiPlusStatus.DcLevelAllowsInverting != dcLevelAllowsInverting;
		newValue |= _multiPlusStatus.DcCurrentA != dcCurrentA;
		if ((buffer[11] & 0xF0) == 0x30) newValue |= _multiPlusStatus.Temp != temp;

		if (newValue)
		{
			xSemaphoreTake(_semaphoreStatus, portMAX_DELAY);
			_multiPlusStatus.DcLevelAllowsInverting = dcLevelAllowsInverting;
			_multiPlusStatus.DcCurrentA = dcCurrentA;
			_multiPlusStatusNewData = true;
			_multiPlusStatusLogged = false;
			if ((buffer[11] & 0xF0) == 0x30) _multiPlusStatus.Temp = temp;
			xSemaphoreGive(_semaphoreStatus);
		}
	}
}

void VEBus::decodeBatteryCondition(std::vector<uint8_t>& buffer)
{
	if ((buffer.size() == 15) && (buffer[5] == 0x81) && (buffer[6] == 0x64) && (buffer[7] == 0x14) && (buffer[8] == 0xBC) && (buffer[9] == 0x02) && (buffer[12] == 0x00))
	{
		float multiplusAh = (((uint16_t)buffer[11] << 8) | buffer[10]);
		if (multiplusAh != _multiPlusStatus.BatterieAh)
		{
			xSemaphoreTake(_semaphoreStatus, portMAX_DELAY);
			_multiPlusStatus.BatterieAh = multiplusAh;
			_multiPlusStatusNewData = true;
			_multiPlusStatusLogged = false;
			xSemaphoreGive(_semaphoreStatus);
		}
	}
}

void VEBus::decodeMasterMultiLed(std::vector<uint8_t>& buffer)
{
	LEDData lEDon{};
	LEDData lEDblink{};
	lEDon.value = buffer[6];
	lEDblink.value = buffer[7];
	bool lowBattery = (buffer[8] == LOW_BATTERY);
	uint8_t lED_AcInputConfiguration = buffer[9];
	float minimumInputCurrentLimit = (((uint16_t)buffer[11] << 8) | buffer[10]) / 10.0f;
	float maximumInputCurrentLimit = (((uint16_t)buffer[13] << 8) | buffer[12]) / 10.0f;
	float actualInputCurrentLimit = (((uint16_t)buffer[15] << 8) | buffer[14]) / 10.0f;
	uint8_t switchRegister = buffer[16];

	bool newValue = false;

	newValue |= _masterMultiLed.LEDon.value != lEDon.value;
	newValue |= _masterMultiLed.LEDblink.value != lEDblink.value;
	newValue |= _masterMultiLed.LowBattery != lowBattery;
	newValue |= _masterMultiLed.AcInputConfiguration != lED_AcInputConfiguration;
	newValue |= _masterMultiLed.MinimumInputCurrentLimitA != minimumInputCurrentLimit;
	newValue |= _masterMultiLed.MaximumInputCurrentLimitA != maximumInputCurrentLimit;
	newValue |= _masterMultiLed.ActualInputCurrentLimitA != actualInputCurrentLimit;
	newValue |= _masterMultiLed.SwitchRegister != switchRegister;

	if (newValue)
	{
		xSemaphoreTake(_semaphoreStatus, portMAX_DELAY);
		_masterMultiLed.LEDon.value = lEDon.value;
		_masterMultiLed.LEDblink.value = lEDblink.value;
		_masterMultiLed.LowBattery = lowBattery;
		_masterMultiLed.AcInputConfiguration = lED_AcInputConfiguration;
		_masterMultiLed.MinimumInputCurrentLimitA = minimumInputCurrentLimit;
		_masterMultiLed.MaximumInputCurrentLimitA = maximumInputCurrentLimit;
		_masterMultiLed.ActualInputCurrentLimitA = actualInputCurrentLimit;
		_masterMultiLed.SwitchRegister = switchRegister;
		_masterMultiLedNewData = true;
		_masterMultiLedLogged = false;
		xSemaphoreGive(_semaphoreStatus);
	}
}

void VEBus::decodeInfoFrame(std::vector<uint8_t>& buffer)
{
	switch (buffer[9])
	{
	case VEBusDefinition::L4:
	case VEBusDefinition::L3:
	case VEBusDefinition::L2:
	case VEBusDefinition::S_L1: // 83 83 FE 1B 20 01 01 00 04 08 00 00 00 00 C6 59 1E 00 00 7D FF 
	case VEBusDefinition::S_L2:
	case VEBusDefinition::S_L3:
	case VEBusDefinition::S_L4:
	{
		AcInfo info{};
		info.Phase = (PhaseInfo)buffer[9];
		info.State = (PhaseState)buffer[8];
		info.MainVoltage = convertRamVarToValueSigned(RamVariables::UBat, (buffer[11] << 8 | buffer[10]));
		info.MainCurrent = convertRamVarToValueSigned(RamVariables::IInverterRMS, (buffer[13] << 8 | buffer[12])) * buffer[5]; // buffer[5] -> BF factor
		info.InverterVoltage = convertRamVarToValueSigned(RamVariables::UBat, (buffer[15] << 8 | buffer[14]));
		info.InverterCurrent = convertRamVarToValueSigned(RamVariables::IInverterRMS, (buffer[17] << 8 | buffer[16])) * buffer[6]; // buffer[6] -> Inverter factor
		//info.MainFrequency = convertSettingToValue(Settings::RepeatedAbsorptionTime,buffer[18]);

		xSemaphoreTake(_semaphoreStatus, portMAX_DELAY);
		bool phaseInfoFound = false;
		for (auto& element : _acInfo) {
			if (info.Phase != element.Phase) continue;
			phaseInfoFound = true;
			if (info == element)
			{
				xSemaphoreGive(_semaphoreStatus);
				return;
			}
			element = info;
			element.newInfo = true;
			break;
		}

		if (phaseInfoFound == false)
		{
			info.newInfo = true;
			_acInfo.push_back(info);
		}
		xSemaphoreGive(_semaphoreStatus);
		break;
	}
	case VEBusDefinition::DC: // 83 83 FE 72 20 40 A5 C4 01 0C 33 05 12 00 00 00 00 00 86 EB FF
	{
		DcInfo info{};
		info.Voltage = convertRamVarToValueSigned(RamVariables::UBat, (buffer[11] << 8 | buffer[10]));
		info.CurrentInverting = convertRamVarToValueSigned(RamVariables::IBat, (buffer[12] | (buffer[13] << 8) | (buffer[14] << 16)));
		info.CurrentCharging = convertRamVarToValueSigned(RamVariables::IBat, (buffer[15] | (buffer[16] << 8) | (buffer[17] << 16)));
		//info.InverterFrequency = 1 / convertSettingToValue(Settings::RepeatedAbsorptionTime, buffer[18]) * 10;

		if (info == _dcInfo) break;
		info.newInfo = true;
		xSemaphoreTake(_semaphoreStatus, portMAX_DELAY);
		_dcInfo = info;
		xSemaphoreGive(_semaphoreStatus);
		break;
	}
	default:
		break;
	}
	xSemaphoreTake(_semaphoreStatus, portMAX_DELAY);

	xSemaphoreGive(_semaphoreStatus);
}

//Runs on core 0
void VEBus::commandHandling()
{
	if (!_communitationIsRunning) return;

	if (_communitationIsResumed)
	{
		_communitationIsResumed = false;
		_serial.flush(false);
	}

	int nr = _serial.available();
	if (nr == 0) return;

	_serial.read(rxbuf, nr);
	for (int n = 0; n < nr; n++)
	{
		_receiveBuffer.push_back(rxbuf[n]);
		if (_receiveBuffer.back() != END_OF_FRAME) continue;

		bool saveToReceiveBufferList = true;

		for (size_t i = 0; i < _whitelistSize; i++)
		{
			saveToReceiveBufferList = false;
			if (_whitelist[i].at > _receiveBuffer.size()) continue;
			if (_whitelist[i].value != _receiveBuffer[_whitelist[i].at]) continue;

			saveToReceiveBufferList = true;
			break;
		}

		for (size_t i = 0; i < _blacklistSize; i++)
		{
			if (_blacklist[i].at > _receiveBuffer.size()) continue;
			if (_blacklist[i].value != _receiveBuffer[_blacklist[i].at]) continue;

			saveToReceiveBufferList = false;
			break;
		}

		xSemaphoreTake(_semaphoreReceiveData, portMAX_DELAY);
		if (_receiveBufferList.size() < 10 && saveToReceiveBufferList) {
			_receiveBufferList.push_back(_receiveBuffer);
		}
		xSemaphoreGive(_semaphoreReceiveData);

		DestuffingFAtoFF(_receiveBuffer);
		auto messageType = decodeVEbusFrame(_receiveBuffer);
		_receiveBuffer.clear();

		if (messageType == ReceivedMessageType::Unknown)
		{
		}

		if (messageType == ReceivedMessageType::sync)
		{
			if (_dataFifo.empty()) continue;
			if ((n != nr - 1))
			{
				//Not the best idea to log on core 0
				if (_logLevel >= LogLevel::Warning) Serial.println("too late");
				continue;
			}

			uint8_t frameNr = _receiveBuffer[3];
			xSemaphoreTake(_semaphoreDataFifo, portMAX_DELAY);
			uint32_t i = 0;
			bool dataToSend = false;
			for (i = 0; i < _dataFifo.size(); i++)
			{
				if (_dataFifo[i].IsSent == false)
				{
					dataToSend = true;
					break;
				}
			}

			if (!dataToSend)
			{
				xSemaphoreGive(_semaphoreDataFifo);
				continue;
			}

			Data& data = _dataFifo.at(i);
			sendData(data, frameNr);

			if (data.responseExpected == false) _dataFifo.erase(_dataFifo.begin() + i);

			xSemaphoreGive(_semaphoreDataFifo);
		}
	}
}

void VEBus::sendData(VEBus::Data& data, uint8_t& frameNr)
{
	Data sendData = data;
	prepareCommand(sendData.requestData, frameNr);
	stuffingFAtoFF(sendData.requestData);
	appendChecksum(sendData.requestData);

	uint8_t* buffer = &sendData.requestData[0];

#ifndef UART_MODE_RS485
	digitalWrite(_rePin, HIGH);
#endif
	_serial.write(buffer, sendData.requestData.size());
#ifndef UART_MODE_RS485
	_serial.flush();
	digitalWrite(_rePin, LOW);
#endif

	data.sentTimeMs = millis();
	data.IsSent = true;
	data.IsLogged = false;
}

void VEBus::checkResponseMessage()
{
	bool dataToSave = false;
	Data data;
	xSemaphoreTake(_semaphoreDataFifo, portMAX_DELAY);
	for (uint8_t i = 0; i < _dataFifo.size();)
	{
		if (_dataFifo[i].responseData.size() == 0) {
			i++;
			continue;
		}

		if (_dataFifo[i].expectedResponseCode == 0)
		{
		}

		if (_dataFifo[i].responseData[6] == _dataFifo[i].expectedResponseCode)
		{
			data = _dataFifo[i];
			dataToSave = true;
			_dataFifo.erase(_dataFifo.begin() + i);
			break;
		}

		if (_dataFifo[i].resendCount >= MAX_RESEND) {
			_dataFifo.erase(_dataFifo.begin() + i);
			break;
		}
		else {
			_dataFifo[i].resendCount++;
			_dataFifo[i].IsSent = false;
			_dataFifo[i].sentTimeMs = millis();
			break;
		}
	}
	xSemaphoreGive(_semaphoreDataFifo);

	if (dataToSave) saveResponseData(data);
}

void VEBus::saveResponseData(Data data)
{
	bool callResponseCb = false;
	ResponseData responseData;
	responseData.id = data.id;
	responseData.command = data.command;
	responseData.address = data.address;

	switch (data.command)
	{
	case VEBusDefinition::SendSoftwareVersionPart0:
	{
		if (data.responseData.size() != 19) {
			if (_logLevel >= LogLevel::Warning) Serial.printf("SendSoftwareVersionPart0 wrong size %d\n", data.responseData.size());
			break;
		}
		callResponseCb = true;
		responseData.valueUint32 = (data.responseData[7]) | (data.responseData[8] << 8) | (data.responseData[9] << 16) | (data.responseData[10] << 24);
		responseData.dataType = ResponseDataType::unsignedInteger;
		//[11] [12] [13] [14] [15] [16] still unknown
		// 08   1D 	 00   00   39   10
		break;
	}
	case VEBusDefinition::SendSoftwareVersionPart1:
		break;
	case VEBusDefinition::GetSetDeviceState:
		if (data.responseData.size() != 11) {
			if (_logLevel >= LogLevel::Warning) Serial.printf("GetSetDeviceState wrong size %d\n", data.responseData.size());
			break;
		}
		callResponseCb = true;
		responseData.dataType = ResponseDataType::unsignedInteger;
		if (data.responseData[7] == 9) responseData.valueUint32 = data.responseData[7] + data.responseData[8];
		else responseData.valueUint32 = data.responseData[7];
		break;
	case VEBusDefinition::ReadRAMVar:
	{
		if (data.responseData.size() != 11) {
			if (_logLevel >= LogLevel::Warning) Serial.printf("ReadRAMVar wrong size %d\n", data.responseData.size());
			break;
		}
		callResponseCb = true;
		responseData.dataType = _ramVarInfoList[data.address].dataType;
		uint16_t UnsignedRawValue = (((uint16_t)data.responseData[8] << 8) | data.responseData[7]);
		int16_t signedRawValueint = ((int16_t)data.responseData[8] << 8) | data.responseData[7];
		if (!_ramVarInfoList[data.address].available) break;
		switch (_ramVarInfoList[data.address].dataType)
		{
		case VEBusDefinition::none:
			break;
		case VEBusDefinition::floatingPoint:
			if (_ramVarInfoList[data.address].Scale < 0) responseData.valueFloat = convertRamVarToValueSigned((RamVariables)data.address, signedRawValueint);
			else responseData.valueFloat = convertRamVarToValue((RamVariables)data.address, UnsignedRawValue);

			responseData.valueFloat += _ramVarInfoList[data.address].Offset;
			break;
		case VEBusDefinition::unsignedInteger:
			responseData.valueUint32 = UnsignedRawValue;
			break;
		case VEBusDefinition::signedInteger:
			responseData.valueUint32 = signedRawValueint;
			break;
		default:
			break;
		}
		
		break;
	}
	case VEBusDefinition::ReadSetting:
	{
		if (data.responseData.size() != 11) {
			if (_logLevel >= LogLevel::Warning) Serial.printf("ReadSetting wrong size %d\n", data.responseData.size());
			break;
		}
		callResponseCb = true;
		responseData.dataType = _settingInfoList[data.address].dataType;
		uint16_t rawValue;
		rawValue = ((uint16_t)data.responseData[8] << 8) | data.responseData[7];
		if (!_settingInfoList[data.address].available) break;
		switch (_settingInfoList[data.address].dataType)
		{
		case VEBusDefinition::none:
			break;
		case VEBusDefinition::floatingPoint:
			responseData.valueFloat = convertSettingToValue((Settings)data.address, rawValue);
			break;
		case VEBusDefinition::unsignedInteger:
			responseData.valueUint32 = rawValue;
			break;
		default:
			break;
		}
		
		break;
	}
	case VEBusDefinition::WriteRAMVar:
		break;
	case VEBusDefinition::WriteSetting:
		break;
	case VEBusDefinition::WriteData:
		break;
	case VEBusDefinition::GetSettingInfo:
		if (data.responseData.size() != 20) {
			if (_logLevel >= LogLevel::Warning) Serial.printf("GetSettingInfo wrong size %d\n", data.responseData.size());
			break;
		}
		saveSettingInfoData(data);
		break;
	case VEBusDefinition::GetRAMVarInfo:
		if (data.responseData.size() != 13) {
			if (_logLevel >= LogLevel::Warning) Serial.printf("GetRAMVarInfo wrong size %d\n", data.responseData.size());
			break;
		}
		saveRamVarInfoData(data);
		break;
	case VEBusDefinition::WriteViaID:
		break;
	case VEBusDefinition::ReadSnapShot:
		break;
	default:
		break;
	}

	if (callResponseCb)
	{
		_onResponseCb(responseData);
	}

	if (_logLevel < LogLevel::Debug) return;
	Serial.print("Res: ");
	for (uint32_t j = 0; j < data.responseData.size(); j++) Serial.printf("%02X ", data.responseData[j]);
	Serial.println();
}

void VEBus::saveSettingInfoData(Data& data)
{
	SettingInfo settingInfo;
	settingInfo.Scale = ((int16_t)data.responseData[8] << 8) | data.responseData[7];
	settingInfo.Offset = ((int16_t)data.responseData[10] << 8) | data.responseData[9];
	settingInfo.Default = ((uint16_t)data.responseData[12] << 8) | data.responseData[11];
	settingInfo.Minimum = ((uint16_t)data.responseData[14] << 8) | data.responseData[13];
	settingInfo.Maximum = ((uint16_t)data.responseData[16] << 8) | data.responseData[15];
	settingInfo.AccessLevel = data.responseData[17];
	_settingInfoList[data.address] = settingInfo;

	if (_logLevel < LogLevel::Debug) return;
	Serial.printf("SettingInfo %d, sc: %d, offset: %d, default: %d, min: %d, max: %d, access: %d\n", data.address, settingInfo.Scale, settingInfo.Offset, settingInfo.Default, settingInfo.Minimum, settingInfo.Maximum, settingInfo.AccessLevel);
}

void VEBus::saveRamVarInfoData(Data& data)
{
	RAMVarInfo ramVarInfo;
	ramVarInfo.Scale = ((int16_t)data.responseData[8] << 8) | data.responseData[7];
	ramVarInfo.Offset = ((int16_t)data.responseData[10] << 8) | data.responseData[9];
	_ramVarInfoList[data.address] = ramVarInfo;

	if (_logLevel < LogLevel::Debug) return;
	Serial.printf("RamVarInfo %d, sc: %d, offset: %d\n", data.address, ramVarInfo.Scale, ramVarInfo.Offset);
}

void VEBus::garbageCollector()
{

	xSemaphoreTake(_semaphoreDataFifo, portMAX_DELAY);
	if (_dataFifo.empty()) {
		xSemaphoreGive(_semaphoreDataFifo);
		return;
	}

	for (auto it = _dataFifo.begin(); it != _dataFifo.end();)
	{
		if (millis() - it->sentTimeMs > RESPONSE_TIMEOUT)
		{
			if (_logLevel >= LogLevel::Warning) Serial.printf("Timeout id: %d command %d resend count: %d\n", it->id, it->command, it->resendCount);
			if (it->resendCount >= MAX_RESEND) {
				it = _dataFifo.erase(it);
				if (_logLevel >= LogLevel::Warning) Serial.println("The message is deleted.");
				continue;
			}
			else {
				it->resendCount++;
				it->IsSent = false;
				it->sentTimeMs = millis();
				++it;
				continue;
			}
		}
		else ++it;
	}
	xSemaphoreGive(_semaphoreDataFifo);
}

void VEBus::logging()
{
	if (_logLevel < LogLevel::Debug) return;

	xSemaphoreTake(_semaphoreDataFifo, portMAX_DELAY);
	if (!_dataFifo.empty())
	{
		if (_dataFifo[0].IsSent && !_dataFifo[0].IsLogged)
		{
			_dataFifo[0].IsLogged = true;
			Serial.print("Req: ");
			for (uint32_t i = 0; i < _dataFifo[0].requestData.size(); i++) Serial.printf("%02X ", _dataFifo[0].requestData[i]);
			Serial.println();

		}
	}
	xSemaphoreGive(_semaphoreDataFifo);

	if (_masterMultiLedNewData && !_masterMultiLedLogged) {
		xSemaphoreTake(_semaphoreStatus, portMAX_DELAY);
		_masterMultiLedLogged = true;
		xSemaphoreGive(_semaphoreStatus);
		Serial.println("new _masterMultiLed data");
	}

	if (_masterMultiLedNewData && !_multiPlusStatusLogged) {
		xSemaphoreTake(_semaphoreStatus, portMAX_DELAY);
		_multiPlusStatusLogged = true;
		xSemaphoreGive(_semaphoreStatus);
		Serial.println("new _multiPlusStatus data");
	}
}

