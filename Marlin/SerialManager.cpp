#include "SerialManager.h"

SerialManager::SerialManager()
{
	_cur = &MSerial;
}

void SerialManager::ChangeSerial(MarlinSerial* s)
{
	_cur = s;
}

MarlinSerial* SerialManager::cur()
{
	return _cur;
}

SerialManager SerialMgr;