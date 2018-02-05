#ifndef _DEF_INC_PMDRPperiph
#define _DEF_INC_PMDRPperiph

//
//  PMDRPperiph.h -- PMD Prodigy/CME Resource Protocol wrapper class for use with a periph handle
//
//  Performance Motion Devices, Inc.
//

#include "PMDRPtypes.h"
#include "PMDRP.h"
#include "PMDperiph.h"

class PMDRPperiph : public PMDResourceProtocol
{
public:
	PMDPeriphHandle* m_hPeriph;

	PMDRPperiph(PMDPeriphHandle* hPeriph);
	~PMDRPperiph(){};

	void WriteByte(char byte);
	void ReadByte(char* byte);

	virtual PMDresult SendPacket( char* pbuff, int nMaxCount);
	virtual PMDresult ReceivePacket(char* pbuff, int nMaxCount, int timeout, int* nReceived);

};

class PMDRPperiphCOM : public PMDRPperiph
{
public:
	PMDRPperiphCOM(PMDPeriphHandle* hPeriph);
	~PMDRPperiphCOM(){};

	int m_Address;
	void SetAddress(int address);

	virtual PMDresult SendPacket( char* pbuff, int nMaxCount);
	virtual PMDresult ReceivePacket(char* pbuff, int nMaxCount, int timeout, int* nReceived);

};

class PMDRPperiphCAN : public PMDRPperiph
{
public:
	PMDRPperiphCAN(PMDPeriphHandle* hPeriph);
	~PMDRPperiphCAN(){};

	int m_Address;
	void SetAddress(int address);

	virtual PMDresult SendPacket( char* pbuff, int nMaxCount);
	virtual PMDresult ReceivePacket(char* pbuff, int nMaxCount, int timeout, int* nReceived);

};


#endif

