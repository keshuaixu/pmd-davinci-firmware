//********************************************************
// PMDResourceProtocol.cpp
// PMD Prodigy/CME Resource Protocol class
//********************************************************

#include "PMDRPtypes.h"
#include "PMDperiph.h"
#include "PMDecode.h"
#include "PMDdiag.h"
#include "PMDmutex.h"
#include "PMDRP.h"
#include "PMDsys.h"

PMDMutexDefine(xMutexPRP);

//********************************************************
PMDResourceProtocol::PMDResourceProtocol()
{
	m_rxbufindex = 0;
	m_txbufindex = 0;
	m_prxBuffer = &m_txBuffer[3]; // in order to word align response packets
	m_bIgnoreResetError = 0;
	PMDMutexCreate(xMutexPRP);
}

//********************************************************
PMDResourceProtocol::~PMDResourceProtocol()
{
}

//********************************************************
PMDresult PMDResourceProtocol::Command( int xCt, WORD *xDat, int rCt, WORD *rDat )
{
	char axis = (char)(xDat[0] >> 8);
	char cmd  = (char)xDat[0];

	xCt--;
	xDat++;
	return Command( axis, cmd, xCt, xDat, rCt, rDat );
}

//********************************************************
PMDresult PMDResourceProtocol::Command( char axis, char cmd, int xCt, WORD *xDat, int rCt, WORD *rDat )
{
	int  i;
	PMDresult result;

	FormPacketHeader(PMD_Resource_MotionProcessor, PMD_Action_Command);

	// place expected #return words in axis word for efficiency
	// it will be extracted by the receiving end
	axis |= (rCt << 6);
	// add axis number and command code
	AddByte(cmd);
	AddByte(axis);

	// add data
	for( i=0; i<xCt; i++ )
	{
		AddWord(xDat[i]);
	}

	
	result = ProcessPacket();

	if (result == PMD_ERR_OK)
	{
		// retrieve data
		for( i=0; i<rCt; i++ )
		{
			rDat[i] = ExtractWord();
		}
	}
	return result;
}

//********************************************************
PMDresult PMDResourceProtocol::WriteMemory(int address, DWORD* data, DWORD offset_in_dwords, DWORD dwords_to_write)
{
	PMDresult result;
	int nBytesRead = 0;
	DWORD dwords_to_write_packet;
	long dwords_to_write_remaining = dwords_to_write;

	dwords_to_write = min(dwords_to_write, MAX_DATA_DWORDS);

	while (dwords_to_write_remaining > 0)
	{
		dwords_to_write_packet = min(dwords_to_write_remaining, MAX_DATA_DWORDS);
		FormPacketHeader(PMD_Resource_Memory, PMD_Action_Write, address);
		AddWord(PMDDataSize_32Bit);
		AddDWord(offset_in_dwords);
		AddDWord(dwords_to_write_packet);

		for (DWORD i = 0; i<dwords_to_write_packet; i++)
		{
			AddDWord(data[i]);
		}

		
		result = ProcessPacket();

		if (result != PMD_ERR_OK)
			return result;
		data += dwords_to_write_packet;
		offset_in_dwords += dwords_to_write_packet;
		dwords_to_write_remaining -= dwords_to_write_packet;
	}
    return PMD_ERR_OK;
}

//********************************************************
PMDresult PMDResourceProtocol::ReadMemory(int address, DWORD* data, DWORD offset_in_dwords, DWORD dwords_to_read)
{
	PMDresult result;
	DWORD dwords_to_read_packet;
	long dwords_to_read_remaining = dwords_to_read;
	int packetno = 0;

	while (dwords_to_read_remaining > 0)
	{
		dwords_to_read_packet = min(dwords_to_read_remaining, MAX_DATA_DWORDS);
		FormPacketHeader(PMD_Resource_Memory, PMD_Action_Read, address);
		AddWord(PMDDataSize_32Bit);
		AddDWord(offset_in_dwords);
		AddDWord(dwords_to_read_packet);

		
		result = ProcessPacket();

		if (result != PMD_ERR_OK)
			return result;

		// retrieve data
		ExtractDWords(data, dwords_to_read_packet);
		data += dwords_to_read_packet;
		offset_in_dwords += dwords_to_read_packet;
		dwords_to_read_remaining -= dwords_to_read_packet;
	}
	
    return PMD_ERR_OK;
}

//********************************************************
PMDresult PMDResourceProtocol::WriteMemory(int address, WORD* data, DWORD offset_in_words, DWORD words_to_write)
{
	PMDresult result;
	int nBytesRead = 0;
	DWORD words_to_write_packet;
	DWORD words_to_write_remaining = words_to_write;

	words_to_write = min(words_to_write, MAX_DATA_DWORDS*2);

	while (words_to_write_remaining > 0)
	{
		words_to_write_packet = min(words_to_write_remaining, MAX_DATA_DWORDS*2);
		FormPacketHeader(PMD_Resource_Memory, PMD_Action_Write, address);
		AddWord(PMDDataSize_16Bit);
		AddDWord(offset_in_words);
		AddDWord(words_to_write_packet);

		for (DWORD i = 0; i<words_to_write_packet; i++)
		{
			AddWord(data[i]);
		}

		
		result = ProcessPacket();

		if (result != PMD_ERR_OK)
			return result;

		data += words_to_write_packet;
		offset_in_words += words_to_write_packet;
		words_to_write_remaining -= words_to_write_packet;
	}

    return PMD_ERR_OK;
}

//********************************************************
PMDresult PMDResourceProtocol::ReadMemory(int address, WORD* data, DWORD offset_in_words, DWORD words_to_read)
{
	PMDresult result;
	DWORD words_to_read_packet;
	DWORD words_to_read_remaining = words_to_read;
	int packetno = 0;

	while (words_to_read_remaining > 0)
	{
		words_to_read_packet = min(words_to_read_remaining, MAX_DATA_DWORDS*2);
		FormPacketHeader(PMD_Resource_Memory, PMD_Action_Read, address);
		AddWord(PMDDataSize_32Bit);
		AddDWord(offset_in_words);
		AddDWord(words_to_read_packet);

		
		result = ProcessPacket();

		if (result != PMD_ERR_OK)
			return result;

		ExtractWords(data, words_to_read_packet);
		data += words_to_read_packet;
		offset_in_words += words_to_read_packet;
		words_to_read_remaining -= words_to_read_packet;
	}
	
    return PMD_ERR_OK;
}

//********************************************************
PMDresult PMDResourceProtocol::Send()
{
	memcpy(m_txHeader, m_txBuffer, PACKET_HEADER_LENGTH);
	if (!PMDMutexLock(xMutexPRP))
		return PMD_ERR_MutexTimeout;
 	return SendPacket(&m_txBuffer[0], m_txbufindex);
}

//********************************************************
PMDresult PMDResourceProtocol::ProcessPacket(int timeout)
{
	PMDresult result;
	int nReceived;

	result = Send();
	if (result == PMD_ERR_OK)
	{
		result = ReceivePacket(&m_txBuffer[0], USER_PACKET_LENGTH, timeout, &nReceived);
		m_rxbufcount = nReceived;
		PMDMutexUnLock(xMutexPRP);
		if (result == PMD_ERR_OK)
		{
			if (m_rxbufcount > 3)
			{
				m_rxbufcount -= 3;
				m_rxbufindex = PACKET_HEADER_LENGTH_RX; // point to data to extract
				result = CheckErrorCode();
			}
			else
				result = PMD_ERR_InsufficientDataReceived;
		}
	}
	return result; // if exception handling is preferred, replace "return" with "throw" here to catch all communication errors.
}

//********************************************************
int PMDResourceProtocol::GetNumberBytesRead()
{
	return m_rxbufcount - 1; // exclude the header byte from the count
}

//********************************************************
PMDresult PMDResourceProtocol::CheckErrorCode()
{
	int bError = (m_prxBuffer[0] >> 4 & 3) == PMD_PRPStatus_Error;
	int status = (m_prxBuffer[0] >> 4 & 3);

	if (bError && m_rxbufcount < PACKET_ERROR_LENGTH)
	{
		DisplayReceivedPacket();
		return PMD_ERR_InsufficientDataReceived;
	}
	// check version bits
	if ((m_prxBuffer[0] >> 6 & 3) != PROTOCOL_VERSION)
		return PMD_ERR_Version;
	if (bError)
	{
		PMDErrorCode CMEerror = (PMDErrorCode)*(unsigned short*)&m_prxBuffer[ERROR_CODE_POS];
//		Printf("%s. Prodigy/CME error code: 0x%04X\r\n", PMDGetErrorMessage(CMEerror), CMEerror);
		if (CMEerror == PMD_ERR_RP_Reset && m_bIgnoreResetError)
		{	// resend the command because it was not processed
			return ProcessPacket();
		}
		else
			return CMEerror;
	}	
	else if (status != 0)
	{
		return PMD_ERR_UnexpectedDataReceived;
	}
	return PMD_ERR_OK;
}

//********************************************************
void PMDResourceProtocol::AddSubCommand(int command)
{
	assert(m_txbufindex == SUB_COMMAND_POS);

	m_txBuffer[SUB_COMMAND_POS] = command;
	m_txbufindex++;
}

//********************************************************
void PMDResourceProtocol::AddSubAction(int subaction)
{
	assert(m_txbufindex == SUB_ACTION_POS);

	m_txBuffer[SUB_ACTION_POS] = subaction;
	m_txbufindex++;
}

//-----------------------------------------------------------------
//                        PRP outgoing header
//|           byte 0              |           byte 1              |
//| 7   6   5   4   3   2   1   0 | 7   6   5   4   3   2   1   0 |
//|   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |   |
//|  ver  | status|    action     |  resource |      address      |
//-----------------------------------------------------------------
//********************************************************
char* PMDResourceProtocol::FormPacketHeader(char resource, char action, char address /* = 0 */)
{
	m_txbufindex = PACKET_HEADER_LENGTH;
	action &= 0x0F;
	m_txBuffer[0] = PROTOCOL_VERSION << 6 | PMD_PRPStatus_Outgoing << 4 | action;
	m_txBuffer[1] = resource << 5 | address;
	m_txBuffer[2] = 0;
	m_txBuffer[3] = 0;

	return &m_txBuffer[2]; // return a pointer to the data portion
}

//********************************************************
DWORD PMDResourceProtocol::ExtractDWord()
{
	assert((DWORD)(&m_prxBuffer[m_rxbufindex]) % 4 == 0);  // make sure we're on a dword aligned address
	assert(m_rxbufindex + 4 <= m_rxbufcount);

	DWORD data = *(DWORD*)&m_prxBuffer[m_rxbufindex];
	m_rxbufindex += 4;

	return data;
}

//********************************************************
WORD PMDResourceProtocol::ExtractWord()
{
	assert((DWORD)(&m_prxBuffer[m_rxbufindex]) % 2 == 0);  // make sure we're on a word aligned address
	assert(m_rxbufindex + 2 <= m_rxbufcount);

	WORD data = *(WORD*)&m_prxBuffer[m_rxbufindex];
	m_rxbufindex += 2;

	return data;
}

//********************************************************
BYTE PMDResourceProtocol::ExtractByte()
{
	assert(m_rxbufindex + 1 <= m_rxbufcount);
	return m_prxBuffer[m_rxbufindex++];
}

//********************************************************
void PMDResourceProtocol::ExtractDWords(DWORD* pData, int nDWords)
{
	for (int i=0; i<nDWords; i++)
		pData[i] = ExtractDWord();
}

//********************************************************
void PMDResourceProtocol::ExtractWords(WORD* pData, int nWords)
{
	for (int i=0; i<nWords; i++)
		pData[i] = ExtractWord();
}

//********************************************************
char* PMDResourceProtocol::ExtractString()
{
	int index = m_rxbufindex;
	int length = strlen(&m_prxBuffer[index]);
	m_rxbufindex += length;;
	assert(m_rxbufindex <= m_rxbufcount);
	return &m_prxBuffer[index];
}

//********************************************************
char* PMDResourceProtocol::GetBuffer()
{
	return m_prxBuffer;
}

//********************************************************
void PMDResourceProtocol::AddDWord(DWORD data)
{
	assert((DWORD)(&m_txBuffer[m_txbufindex]) % 4 == 0);

	*(DWORD*)&m_txBuffer[m_txbufindex] = data;
	m_txbufindex += 4;
}

//********************************************************
void PMDResourceProtocol::AddWord(WORD data)
{
	assert((DWORD)(&m_txBuffer[m_txbufindex]) % 2 == 0);

	*(WORD*)&m_txBuffer[m_txbufindex] = data;
	m_txbufindex += 2;
}

//********************************************************
void PMDResourceProtocol::AddByte(BYTE data)
{
	m_txBuffer[m_txbufindex++] = data;
}

//********************************************************
void PMDResourceProtocol::DisplayReceivedPacket()
{
/*
	Printf( "Packet received:" );
	for( int i=0; i<m_rxbufindex; i++ )
		Printf( " %02X", (BYTE)m_prxBuffer[i]);
	Printf("\n");
*/
}

//*********************************************************************************
// PMD RP Specific commands
// words and dwords are little endian
//*********************************************************************************
// tx byte 1 |	PMD_Action_NOP
// tx byte 2 |	PMD_Resource_Device
PMDresult PMDResourceProtocol::NoOperation()
{
	FormPacketHeader(PMD_Resource_Device, PMD_Action_NOP);
	
	return ProcessPacket();
}

//*********************************************************************************
// tx byte 1 |	PMD_Action_Reset
// tx byte 2 |	PMD_Resource_Device
PMDresult PMDResourceProtocol::GetResetCause(WORD resetmask, WORD* resetcause)
{
	PMDresult result;
	FormPacketHeader(PMD_Resource_Device, PMD_Action_Get);
	AddSubAction(PMD_Value_ResetCause);
	AddSubCommand(0);
	AddWord(resetmask);
	
	result = ProcessPacket();
	if (result == PMD_ERR_OK)
		*resetcause = ExtractWord();

	return result;
}

//*********************************************************************************
// tx byte 1 |	PMD_Action_Reset
// tx byte 2 |	PMD_Resource_Device
PMDresult PMDResourceProtocol::Reset()
{
	FormPacketHeader(PMD_Resource_Device, PMD_Action_Reset);
	Send();
	// no response packet is sent
	// allow enough time for board to come out of reset
	PMDTaskWait(1000);
	PMDMutexUnLock(xMutexPRP); // since this function does not call Receive() we need to unlock the mutex here
	return PMD_ERR_OK;
}

//*********************************************************************************
// tx byte 1 |	PMD_Action_Reset
// tx byte 2 |	PMD_Resource_MotionProcessor
PMDresult PMDResourceProtocol::ResetMagellan()
{
	FormPacketHeader(PMD_Resource_MotionProcessor, PMD_Action_Reset);
	
	return ProcessPacket();
}

//*********************************************************************************
// tx byte	|	PMD_Action_Close
// tx byte	|	PMD_Resource_Peripheral
PMDresult PMDResourceProtocol::ClosePeriph(int address)
{
	FormPacketHeader(PMD_Resource_Peripheral, PMD_Action_Close, address);
	
	return ProcessPacket();
}

//*********************************************************************************
// tx byte	|	PMD_Action_Close
// tx byte	|	PMD_Resource_Memory
PMDresult PMDResourceProtocol::CloseMemory(int address)
{
	FormPacketHeader(PMD_Resource_Memory, PMD_Action_Close, address);
	
	return ProcessPacket();
}

//*********************************************************************************
// tx byte	|	PMD_Action_Send
// tx byte	|	PMD_Resource_Peripheral
// tx word	|	timeout
PMDresult PMDResourceProtocol::SendPeriph(int address, char* pbuff, int nCount, int timeout)
{
	if (nCount > USER_PACKET_LENGTH)
		return PMD_ERR_ParameterOutOfRange;
	FormPacketHeader(PMD_Resource_Peripheral, PMD_Action_Send, address);
	AddWord(timeout);
	while(nCount-- > 0)
		AddByte(*pbuff++);
	
	return ProcessPacket(timeout);
}

//*********************************************************************************
// tx byte	|	PMD_Action_Receive
// tx byte	|	PMD_Resource_Peripheral
// tx word	|	timeout
PMDresult PMDResourceProtocol::ReceivePeriph(int address, char* pbuff, int nMaxExpected, int timeout, PMDparam* nReceived)
{
	PMDresult result;
	int i, nBytesRead;
	if (nMaxExpected > USER_PACKET_LENGTH)
		return PMD_ERR_ParameterOutOfRange;
	FormPacketHeader(PMD_Resource_Peripheral, PMD_Action_Receive, address);
	AddWord(timeout);
	AddWord(nMaxExpected);
	
	result = ProcessPacket(timeout);
	if (result != PMD_ERR_OK)
		return result;

	nBytesRead = GetNumberBytesRead();
	if (nBytesRead > nMaxExpected)
		nBytesRead = nMaxExpected;
	i = nBytesRead;
	while(i-- > 0)
		*pbuff++ = ExtractByte();
	*nReceived = nBytesRead;

	return result;
}

//********************************************************
PMDresult PMDResourceProtocol::WritePeriph(int address, WORD* pdata, DWORD offset_in_bytes, DWORD words_to_write)
{
	words_to_write = min(words_to_write, MAX_DATA_DWORDS*2);

	FormPacketHeader(PMD_Resource_Peripheral, PMD_Action_Write, address);
	AddByte(PMDDataSize_16Bit);
	AddByte(0);
	AddDWord(offset_in_bytes);
	AddDWord(words_to_write);

	for (DWORD i = 0; i<words_to_write; i++)
	{
		AddWord(pdata[i]);
	}
	
	return ProcessPacket();
}

//********************************************************
PMDresult PMDResourceProtocol::ReadPeriph(int address, WORD* pdata, DWORD offset_in_bytes, DWORD words_to_read)
{
	PMDresult result;
	DWORD nBytesExpected;
	DWORD nBytesRead;

	words_to_read = min(words_to_read, MAX_DATA_DWORDS*2);
	nBytesExpected = words_to_read * sizeof(WORD);

	FormPacketHeader(PMD_Resource_Peripheral, PMD_Action_Read, address);
	AddByte(PMDDataSize_16Bit);
	AddByte(0);
	AddDWord(offset_in_bytes);
	AddDWord(words_to_read);

	result = ProcessPacket();
	if (result != PMD_ERR_OK)
		return result;

	nBytesRead = GetNumberBytesRead();
	if (nBytesRead * 2 < words_to_read)
		return PMD_ERR_InsufficientDataReceived;

	for (DWORD i=0; i<words_to_read; i++)
		pdata[i] = ExtractWord();

	return result;
}

//*********************************************************************************
// tx byte	|	PMD_Action_Send
// tx byte	|	PMD_Resource_CMotionEngine
// tx word	|	timeout
PMDresult PMDResourceProtocol::SendCME(char* pbuff, int nCount, int timeout)
{
	if (nCount > USER_PACKET_LENGTH)
		return PMD_ERR_ParameterOutOfRange;
	FormPacketHeader(PMD_Resource_CMotionEngine, PMD_Action_Send);
	AddWord(timeout);
	while(nCount-- > 0)
		AddByte(*pbuff++);
	
	return ProcessPacket(timeout);
}

//*********************************************************************************
// tx byte	|	PMD_Action_Receive
// tx byte	|	PMD_Resource_CMotionEngine
// tx word	|	timeout
PMDresult PMDResourceProtocol::ReceiveCME(char* pbuff, int nMaxExpected, int timeout, PMDparam* nReceived)
{
	PMDresult result;
	int i, nBytesRead;

	if (nMaxExpected > USER_PACKET_LENGTH)
		return PMD_ERR_ParameterOutOfRange;
	FormPacketHeader(PMD_Resource_CMotionEngine, PMD_Action_Receive);
	AddWord(timeout);
	
	result = ProcessPacket();
	if (result != PMD_ERR_OK)
		return result;
	nBytesRead = GetNumberBytesRead();
	nBytesRead = GetNumberBytesRead();
	if (nBytesRead > nMaxExpected)
		nBytesRead = nMaxExpected;
	i = nBytesRead;
	while(i-- > 0)
		*pbuff++ = ExtractByte();
	*nReceived = nBytesRead;

	return result;
}

//*********************************************************************************
// tx byte	|	PMD_Action_Open
// tx byte	|	PMD_Resource_Device
// tx word	|	portid
// rx word	|	periphaddress - this is the address returned from and Open Peripheral action
PMDresult PMDResourceProtocol::OpenPeriphSerial(int* periphaddress, int portno, int baud, int parity, int stopbits)
{
	PMDresult result;
	WORD serialmode;

	FormPacketHeader(PMD_Resource_Device, PMD_Action_Open);
	AddSubAction(PMD_Open_PeriphCOM);
	AddSubCommand(portno);
	SET_SERIALPORTMODE(serialmode, baud, parity, stopbits, 0, 0);
	AddWord(serialmode);
	
	result = ProcessPacket();
	if (result == PMD_ERR_OK)
		*periphaddress = ExtractByte();

	return result;
}

//*********************************************************************************
// tx byte	|	PMD_Action_Open
// tx byte	|	PMD_Resource_Device
// tx word	|	portid
// rx word	|	periphaddress - this is the address returned from and Open Peripheral action
PMDresult PMDResourceProtocol::OpenPeriphCAN(int* periphaddress, DWORD transmitid, DWORD receiveid, DWORD eventid)
{
	PMDresult result;
	int portno = 0;

	FormPacketHeader(PMD_Resource_Device, PMD_Action_Open);
	AddSubAction(PMD_Open_PeriphCAN);
	AddSubCommand(portno);
	AddDWord(transmitid);
	AddDWord(receiveid);
	AddDWord(eventid);
	
	result = ProcessPacket();
	if (result == PMD_ERR_OK)
		*periphaddress = ExtractByte();

	return result;
}

//*********************************************************************************
// tx byte	|	PMD_Action_Open
// tx byte	|	PMD_Resource_Device
// tx word	|	portid
// rx word	|	periphaddress - this is the address returned from and Open Peripheral action
PMDresult PMDResourceProtocol::OpenPeriphUDP(int* periphaddress, DWORD ipaddress, DWORD port)
{
	PMDresult result;
	BYTE portno = 0;

	FormPacketHeader(PMD_Resource_Device, PMD_Action_Open);
	AddSubAction(PMD_Open_PeriphUDP);
	AddSubCommand(portno);
	AddDWord(ipaddress);
	AddDWord(port);
	
	result = ProcessPacket();
	if (result == PMD_ERR_OK)
		*periphaddress = ExtractByte();

	return result;
}

//*********************************************************************************
// tx byte	|	PMD_Action_Open
// tx byte	|	PMD_Resource_Device
// tx word	|	portid
// rx word	|	periphaddress - this is the address returned from and Open Peripheral action
PMDresult PMDResourceProtocol::OpenPeriphTCP(int* periphaddress, DWORD ipaddress, DWORD port)
{
	PMDresult result;
	BYTE portno = 0;

	FormPacketHeader(PMD_Resource_Device, PMD_Action_Open);
	AddSubAction(PMD_Open_PeriphTCP);
	AddSubCommand(portno);
	AddDWord(ipaddress);
	AddDWord(port);

	result = ProcessPacket();
	if (result == PMD_ERR_OK)
		*periphaddress = ExtractByte();

	return result;
}

//*********************************************************************************
// tx byte	|	PMD_Action_Open
// tx byte	|	PMD_Resource_Device
// tx word	|	16-bit start address
// rx word	|	periphaddress - this is the address returned from and Open Peripheral action
PMDresult PMDResourceProtocol::OpenPeriphISA(int* periphaddress, WORD address, BYTE irq, BYTE datasize)
{
	PMDresult result;
	FormPacketHeader(PMD_Resource_Device, PMD_Action_Open);
	AddSubAction(PMD_Open_PeriphISA);
	AddSubCommand(0);
	AddWord(address);
	AddByte(irq);
	AddByte(datasize); // 1,2,4
	
	result = ProcessPacket();
	if (result == PMD_ERR_OK)
		*periphaddress = ExtractByte();

	return result;
}

//*********************************************************************************
// tx byte	|	PMD_Action_Open
// tx byte	|	PMD_Resource_Device
// tx word	|	16-bit start address
// rx word	|	periphaddress - this is the address returned from and Open Peripheral action
PMDresult PMDResourceProtocol::OpenPeriphPAR(int* periphaddress, WORD address, BYTE irq, BYTE datasize)
{
	PMDresult result;
	FormPacketHeader(PMD_Resource_Device, PMD_Action_Open);
	AddSubAction(PMD_Open_PeriphPAR);
	AddSubCommand(0);
	AddWord(address);
	AddByte(irq);
	AddByte(datasize); // 1,2,4
	
	result = ProcessPacket();
	if (result == PMD_ERR_OK)
		*periphaddress = ExtractByte();

	return result;
}

//*********************************************************************************
// tx byte	|	PMD_Action_Open
// tx byte	|	PMD_Resource_Device
// tx byte	|	memorytype
// tx byte	|	datasize
// rx word	|	resourceaddress - this is the address returned from and Open Memory action
PMDresult PMDResourceProtocol::OpenMemory(int* resourceaddress, PMDDataSize datasize, PMDMemoryType memorytype)
{
	PMDresult result;
	FormPacketHeader(PMD_Resource_Device, PMD_Action_Open);
	AddSubAction(PMD_Open_Memory);
	AddSubCommand(memorytype);
	AddByte(datasize);
	
	result = ProcessPacket();
	if (result == PMD_ERR_OK)
		*resourceaddress = ExtractByte();

	return result;
}

//*********************************************************************************
// tx byte	|	PMD_Action_Set
// tx byte	|	PMD_Resource_CMotionEngine
// tx word	|	PMD_Value_Console
// tx word	|	periphaddress - this is the address returned from and Open Peripheral action
PMDresult PMDResourceProtocol::SetConsole(int periphaddress)
{
	FormPacketHeader(PMD_Resource_CMotionEngine, PMD_Action_Set);
	AddSubAction(PMD_Value_Console);
	AddSubCommand(periphaddress);
	
	return ProcessPacket();
}

//*********************************************************************************
// tx byte 1 |	PMD_Action_Get
// tx byte 2 |	PMD_Resource_Device
// tx byte 3 |	PMD_Value_Default
// tx byte 4 |	0
// tx word	 |	default id
// tx words  |	default value
PMDresult PMDResourceProtocol::GetDefault(DWORD dwParamNo, DWORD* dwParam)
{
	PMDresult result;
	FormPacketHeader(PMD_Resource_Device, PMD_Action_Get);
	AddSubAction(PMD_Value_Default);
	AddSubCommand(0);
	AddDWord(dwParamNo);
	
	result = ProcessPacket();
	if (result == PMD_ERR_OK)
		*dwParam = ExtractDWord();

	return result;
}

//*********************************************************************************
// tx byte 1 |	PMD_Action_Set
// tx byte 2 |	PMD_Resource_Device
// tx byte 3 |	PMD_Value_Default
// tx byte 4 |	0
// tx word	 |	default id
// tx words  |	default value
//
PMDresult PMDResourceProtocol::SetDefault(DWORD dwParamNo, DWORD dwParam)
{
	FormPacketHeader(PMD_Resource_Device, PMD_Action_Set);
	AddSubAction(PMD_Value_Default);
	AddSubCommand(0);
	AddDWord(dwParamNo);
	AddDWord(dwParam);
	
	return ProcessPacket(2000); // need a longer than normal timeout value to accomodate
	               // the off chance that the NVRAM needs to be formatted.
}

//*********************************************************************************
// tx byte 1 |	PMD_Action_Command
// tx byte 2 |	PMD_Resource_CMotionEngine
// tx byte 3 |	PMD_Command_TaskControl
// tx byte 4 |	PMD_TaskControl_Start
PMDresult PMDResourceProtocol::StartUserTasks()
{
	FormPacketHeader(PMD_Resource_CMotionEngine, PMD_Action_Command);
	AddSubAction(PMD_Command_TaskControl);
	AddSubCommand(PMD_TaskControl_StartAll);
	
	return ProcessPacket();
}
//*********************************************************************************
// tx byte 1 |	PMD_Action_Command
// tx byte 2 |	PMD_Resource_CMotionEngine
// tx byte 3 |	PMD_Command_TaskControl
// tx byte 4 |	PMD_TaskControl_Start
PMDresult PMDResourceProtocol::StopUserTasks()
{
	FormPacketHeader(PMD_Resource_CMotionEngine, PMD_Action_Command);
	AddSubAction(PMD_Command_TaskControl);
	AddSubCommand(PMD_TaskControl_StopAll);
	
	return ProcessPacket();
}

//*********************************************************************************
// tx byte  |	PMD_Action_Get
// tx byte  |	PMD_Resource_CMotionEngine
// tx byte  |	PMD_Value_FileName
// rx bytes |	filename
PMDresult PMDResourceProtocol::GetUserCodeFileName(char* filename)
{
	PMDresult result;
	FormPacketHeader(PMD_Resource_CMotionEngine, PMD_Action_Get);
	AddSubAction(PMD_Value_FileName);
	
	result = ProcessPacket();
	if (result == PMD_ERR_OK)
		strcpy(filename,  ExtractString());

	return result;
}

//*********************************************************************************
PMDresult PMDResourceProtocol::GetUserCodeFileDate(char* filedate)
{
	PMDresult result;
	FormPacketHeader(PMD_Resource_CMotionEngine, PMD_Action_Get);
	AddSubAction(PMD_Value_FileDate);
	
	result = ProcessPacket();
	if (result == PMD_ERR_OK)
		strcpy(filedate, ExtractString());

	return result;
}

//*********************************************************************************
PMDresult PMDResourceProtocol::GetUserCodeFileChecksum(DWORD* checksum)
{
	PMDresult result;
	FormPacketHeader(PMD_Resource_CMotionEngine, PMD_Action_Get);
	AddSubAction(PMD_Value_FileChecksum);
	
	result = ProcessPacket();
	if (result == PMD_ERR_OK)
		*checksum = ExtractDWord();

	return result;
}

//*********************************************************************************
PMDresult PMDResourceProtocol::GetUserCodeFileVersion(DWORD* version)
{
	PMDresult result;
	FormPacketHeader(PMD_Resource_CMotionEngine, PMD_Action_Get);
	AddSubAction(PMD_Value_FileVersion);
	
	result = ProcessPacket();
	if (result == PMD_ERR_OK)
		*version = ExtractDWord();

	return result;
}

//*********************************************************************************
// tx byte  |	PMD_Action_Get
// tx byte  |	PMD_Resource_CMotionEngine
// tx byte  |	PMD_Value_TaskState
// rx dword |	
PMDresult PMDResourceProtocol::GetUserCodeStatus(int taskno, int* taskstate)
{
	PMDresult result;
	FormPacketHeader(PMD_Resource_CMotionEngine, PMD_Action_Get);
	AddSubAction(PMD_Value_TaskState);
	AddSubCommand(taskno);
	
	result = ProcessPacket();
	if (result == PMD_ERR_OK)
		*taskstate = (int)ExtractDWord();

	return result;
}

//*********************************************************************************
// tx byte 1 |	PMD_Action_Get
// tx byte 2 |	PMD_Resource_Device
// tx byte 3 |	PMD_Value_Version
PMDresult PMDResourceProtocol::GetFirmwareVersion(DWORD *version)
{
	PMDresult result;
	FormPacketHeader(PMD_Resource_Device, PMD_Action_Get);
	AddSubAction(PMD_Value_Version);
	
	result = ProcessPacket();

	if (result != PMD_ERR_OK)
		return result;
	*version = ExtractDWord();

	return PMD_ERR_OK;
}

//*********************************************************************************
// tx byte 1 |	PMD_Action_Command
// tx byte 2 |	PMD_Resource_CMotionEngine
// tx byte 3 |	PMD_Flash_Erase
PMDresult PMDResourceProtocol::EraseUserCode()
{
	FormPacketHeader(PMD_Resource_CMotionEngine, PMD_Action_Command);
	AddSubAction(PMD_Command_Flash);
	AddSubCommand(PMD_Flash_Erase);
	
	return ProcessPacket(6000);
}

//*********************************************************************************
// StoreUserCodeBegin is called to prepare the CME to receive the user code binary file.
// This command may take several seconds so the communications timeout must be increased accordingly
PMDresult PMDResourceProtocol::StoreUserCodeBegin(int filesize)
{
	FormPacketHeader(PMD_Resource_CMotionEngine, PMD_Action_Command);
	AddSubAction(PMD_Command_Flash);
	AddSubCommand(PMD_Flash_Start);
	AddDWord(filesize);
	
	return ProcessPacket(6000);
}

//*********************************************************************************
// StoreUserCodeData is called for each contiguous block of the user code binary file.
// length must be even except for last buffer
// length must not esceed MAX_PACKET_DATA_LENGTH
PMDresult PMDResourceProtocol::StoreUserCodeData(char* pdata, int length)
{
	FormPacketHeader(PMD_Resource_CMotionEngine, PMD_Action_Command);
	AddSubAction(PMD_Command_Flash);
	AddSubCommand(PMD_Flash_Data);
	for (int i=0; i<length; i++)
		AddByte(pdata[i]);
	
	return ProcessPacket();
}

//*********************************************************************************
// StoreUserCodeEnd is called to complete the download process.
PMDresult PMDResourceProtocol::StoreUserCodeEnd()
{
	FormPacketHeader(PMD_Resource_CMotionEngine, PMD_Action_Command);
	AddSubAction(PMD_Command_Flash);
	AddSubCommand(PMD_Flash_End);
	
	return ProcessPacket();
}

