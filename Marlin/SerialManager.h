#ifndef SerialManager_h
#define SerialManager_h
#include <WProgram.h>
#include "MarlinSerial.h"

class SerialManager{
	public:
		SerialManager(void);
		void ChangeSerial(MarlinSerial* s);
		MarlinSerial* cur();
	
	private:
		MarlinSerial* _cur;
};

extern SerialManager SerialMgr;
#endif
