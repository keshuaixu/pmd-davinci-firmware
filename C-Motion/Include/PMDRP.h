#ifndef _DEF_INC_PMDRP
#define	_DEF_INC_PMDRP

//
//  PMDRP.h - PMD Prodigy/CME Resource Protocol class
//
//  Performance Motion Devices, Inc.
//

#include "PMDRPtypes.h"
#include "PMDtypes.h"

class PMDResourceProtocol
{
#pragma pack(4) // 32-bit align in order to efficiently copy 32 bit words into the packet
	char m_txBuffer[TOTAL_PACKET_LENGTH];
	char m_txHeader[PACKET_HEADER_LENGTH];
#pragma pack()
	char* m_prxBuffer;
	char* m_ptxBuffer;
	int m_txbufindex;
	int m_rxbufindex;
	int m_rxbufcount;
	int m_bIgnoreResetError;

public:
	PMDResourceProtocol();
	~PMDResourceProtocol();

	PMDresult	CheckErrorCode();
	void		DisplayReceivedPacket();
	char*		FormPacketHeader(char resource, char action, char address = 0);
	void		AddSubAction(int command);
	void		AddSubCommand(int command);
	DWORD		ExtractDWord();
	WORD		ExtractWord();
	BYTE		ExtractByte();
	char*		ExtractString();
	void		ExtractDWords(DWORD* pData, int nDWords);
	void		ExtractWords(WORD* pData, int nWords);
	void		AddDWord(DWORD data);
	void		AddWord(WORD data);
	void		AddByte(BYTE data);
	char*		GetBuffer();
	int			GetNumberBytesRead();

	PMDresult	Send();
	PMDresult	ProcessPacket(int timeout = RP_TIMEOUT);

	virtual PMDresult SendPacket(char* pbuff, int nMaxCount){return PMD_ERR_InterfaceNotInitialized;}
	virtual PMDresult ReceivePacket(char* pbuff, int nMaxCount, int timeout, int* nReceived){return PMD_ERR_InterfaceNotInitialized;}

	PMDresult	NoOperation();
	PMDresult	Reset();
	PMDresult	ResetMagellan();
	PMDresult	Command( char axis, char cmd, int xCt=0, WORD *xDat=NULL, int rCt=0, WORD *rDat=NULL );
	PMDresult	Command( int xCt=0, WORD *xDat=NULL, int rCt=0, WORD *rDat=NULL );

	PMDresult	ReadMemory (int address, DWORD* data, DWORD offset_in_dwords, DWORD dwords_to_read);
	PMDresult	WriteMemory(int address, DWORD* data, DWORD offset_in_dwords, DWORD dwords_to_write);
	PMDresult	ReadMemory (int address, WORD* data, DWORD offset_in_words, DWORD words_to_read);
	PMDresult	WriteMemory(int address, WORD* data, DWORD offset_in_words, DWORD words_to_write);

	PMDresult	SendCME(char* pbuff, int nCount, int timeout);
	PMDresult	ReceiveCME(char* pbuff, int nMaxExpected, int timeout, PMDparam* nReceived);
	PMDresult	SendPeriph(int address, char* pbuff, int nCount, int timeout);
	PMDresult	ReceivePeriph(int address, char* pbuff, int nMaxExpected, int timeout, PMDparam* nReceived);
	PMDresult	WritePeriph(int address, WORD* pdata, DWORD offset_in_bytes, DWORD words_to_write);
	PMDresult	ReadPeriph(int address, WORD* pdata, DWORD offset_in_bytes, DWORD words_to_read);

	PMDresult	SetConsole(int periphaddress);
	PMDresult	OpenPeriphTCP(int* periphaddress, DWORD ipaddress, DWORD port);
	PMDresult	OpenPeriphUDP(int* periphaddress, DWORD ipaddress, DWORD port);
	PMDresult	OpenPeriphSerial(int* periphaddress, int portno, int baud, int parity, int stopbits);
	PMDresult	OpenPeriphCAN(int* periphaddress, DWORD transmitid, DWORD receiveid, DWORD eventid);
	PMDresult	OpenPeriphPAR(int* periphaddress, WORD address, BYTE irq, BYTE datasize);
	PMDresult	OpenPeriphISA(int* periphaddress, WORD address, BYTE irq, BYTE datasize);
    PMDresult	OpenPeriphMultiDrop(int parentaddress, int nodeID);
	PMDresult	OpenMemory(int* resourceaddress, PMDDataSize datasize, PMDMemoryType memorytype);

	PMDresult	ClosePeriph(int address);
	PMDresult	CloseMemory(int address);
				
	PMDresult	GetDefault(DWORD dwParamNo, DWORD* dwParam);
	PMDresult	SetDefault(DWORD dwParamNo, DWORD dwParam);
				
	PMDresult	StartUserTasks();
	PMDresult	StopUserTasks();
	PMDresult	GetUserCodeTaskName(int taskno, char*);
	PMDresult	GetUserCodeFileName(char*);
	PMDresult	GetUserCodeFileDate(char*);
	PMDresult	GetUserCodeFileChecksum(DWORD*);
	PMDresult	GetUserCodeFileVersion(DWORD*);
	PMDresult	GetTaskStatusAll(char*);
	PMDresult 	GetUserCodeStatus(int taskno, int*);
	PMDresult	GetFirmwareVersion(DWORD*);
	PMDresult	EraseUserCode();
	PMDresult	StoreUserCodeBegin(int filesize);
	PMDresult	StoreUserCodeData(char* pdata, int length);
	PMDresult	StoreUserCodeEnd();
	PMDresult	GetResetCause(WORD resetmask, WORD* resetcause);
};

#endif

