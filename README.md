# VEBus
This library is used to work with an ESP32 Arduino (not for the AVR family as the library uses std::vector).

## Built With
* [Arduino IDE 2.3.2]
* [Visual Studio 2022 and visual micro]

## Dependencies
arduino-esp32 boardlibrary >= 2.0.8 
for < 2.0.8 undefine UART_MODE_RS485 in the vebus.h file
```ruby
//#define UART_MODE_RS485
#ifdef UART_MODE_RS485
#include "hal/uart_types.h"
#endif
```

## Getting Started
This is a sample guide for setting up your project locally.

### Hardware
* ESP32 Board
* RS485 IC

The RS485 IC can be a MAX485 or max1348

> [!TIP]
> For a simple start (PlugAndPlay) you can use an esp32 arduino board with integrated RS485
> [LILYGO T-CAN485](https://a.aliexpress.com/_EzyFDFB)

### Software

There are a few examples to get you started with the library

* [ReadWriteSettings](https://github.com/GitNik1/VEBus/tree/master/Examples/ReadWriteSettings)
This example is a minimized program for reading out some settings on a Multiplus-II 12/3000

* [VebusSniffer](https://github.com/GitNik1/VEBus/tree/master/Examples/VebusSniffer)
This example can be used to listen to all traffic on the VE.BUS.

* [ReadAllVariableInfos](https://github.com/GitNik1/VEBus/tree/master/Examples/ReadAllVariableInfos)
This example can be used to read all variable information. This is necessary to interpret the value sent by the VE.BUS device.
If you have a device, it is not in the library. Send me the serial output to integrate the new device.


## Function descriptions
* [Callback for received messages](https://github.com/GitNik1/VEBus?tab=readme-ov-file#callback-for-received-messages)
* [Callback for response messages](https://github.com/GitNik1/VEBus?tab=readme-ov-file#callback-for-response-messages)
* [Write a value to Multiplus](https://github.com/GitNik1/VEBus?tab=readme-ov-file#write-a-value-to-multiplus)
* [Read a value to Multiplus](https://github.com/GitNik1/VEBus?tab=readme-ov-file#read-a-value-to-multiplus)
### Callback for received messages
```ruby
void SetReceiveCallback(std::function<void(std::vector<uint8_t>& buffer)> cb);
```
To parse all received messages, the callback can be used as follows

*.ino
```ruby
void Receive(std::vector<uint8_t>& buffer)
{
    _vEBus.DestuffingFAtoFF(buffer);
    Serial.printf("Res: ");
    for (uint32_t j = 0; j < buffer.size(); j++) Serial.printf("%02X ", buffer[j]);
    Serial.println();
}

void setup()
{
	_vEBus.SetReceiveCallback(Receive);
}
```
> [!TIP]
> If you don't want to see some messages, you can skip them using a blacklist or use the whitelist to see specific messages.
```ruby
VEBus::Blacklist _blacklist[] = { {.value = 0xE4, .at = 4}, {.value = 0x55, .at = 4} };

void setup()
{
	_vEBus.SetReceiveCallback(Receive, _blacklist, sizeofarray(_blacklist));
}
```

### Callback for response messages
```ruby
void SetResponseCallback(std::function<void(ResponseData&)> cb);
```
To parse all response messages, the callback can be used as follows.
data.id is specified by the query to indicate that it is the response of the read operation.

*.ino
```ruby
void Response(VEBus::ResponseData& data)
{
	if(requestId == data.id){}
	if(WinmonCommand == data.command && Settings == data.address){}
    //...to some stuff
}

void setup()
{
	_vEBus.SetResponseCallback(Response);
}
```

### Write a value to Multiplus
```ruby
uint8_t WriteViaID(RamVariables variable, int16_t rawValue, bool eeprom = false);
uint8_t WriteViaID(RamVariables variable, uint16_t rawValue, bool eeprom = false);
RequestResult WriteViaID(RamVariables variable, float value, bool eeprom = false);

uint8_t WriteViaID(Settings setting, int16_t rawValue, bool eeprom = false);
uint8_t WriteViaID(Settings setting, uint16_t rawValue, bool eeprom = false);
RequestResult WriteViaID(Settings setting, float value, bool eeprom = false);
```
To write a value, you have three ways to send it. 
You can store a value to a non-volatile memory. To do this you need to set the eeprom true
> [!CAUTION]
> Be careful when repeatedly writing EEPROM (loop)
> EEPROM writes are limited

* library supported device with value interpretations

*.ino
```ruby
void loop()
{
	if(dataToWrite == true)
	{
		_vEBus.WriteViaID(Settings::IMainsLimit, 7.2); //set 7.2A
	}
}
```

* no library supported device with value interpretations

*.ino
```ruby
void setup()
{
	_vEBus.ReadInfo(Settings::IMainsLimit);
}

void loop()
{
	if(dataToWrite == true)
	{
		_vEBus.WriteViaID(Settings::IMainsLimit, 7.2); //set 7.2A
	}
}
```

* raw without value interpretations

*.ino
```ruby
void loop()
{
	if(dataToWrite == true)
	{
		_vEBus.WriteViaID(Settings::IMainsLimit, 72); //set 7.2A
	}
}
```

### Read a value to Multiplus
```ruby
uint8_t Read(RamVariables variable);
uint8_t Read(Settings setting);
```
To read a value

*.ino
```ruby
void loop()
{
	if(dataToRead == true)
	{
		uint8_t requestId = _vEBus.Read(Settings::IMainsLimit);
	}
}
```

## Supported devices with value interpretations
- [X] Multiplus-II 12/3000

## To do lists
- [ ] add more integrated devices

## Known issues
- [ ] VENUS::Write() is currently not working. (Use VENUS::WriteViaID() instead)
- [ ] extract more GX commands

## License

This project is licensed under the GNU General Public License v3.0 or later - see the LICENSE.md file for details

## Acknowledgments

* [Hadmut] (https://www.mikrocontroller.net/topic/561834)
* [pv-baxi] (https://github.com/pv-baxi/esp32ess)