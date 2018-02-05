//********************************************************
// PMDRPperiph.cpp
// PMD Prodigy/CME Resource Protocol wrapper class implementation
//********************************************************

#include "PMDtypes.h"
#include "PMDsys.h"
#include "PMDRPperiph.h"


//********************************************************
// generic RP protocol wrapper class
PMDRPperiph::PMDRPperiph(PMDPeriphHandle* hPeriph)
{
	m_hPeriph = hPeriph;
}

void PMDRPperiph::WriteByte(char byte)
{
	PMDPeriphSend(m_hPeriph, &byte, 1, RP_TIMEOUT);
}

PMDresult PMDRPperiph::SendPacket(char* pbuff, int nCount)
{
	return PMDPeriphSend(m_hPeriph, pbuff, nCount, RP_TIMEOUT);
}

PMDresult PMDRPperiph::ReceivePacket(char* pbuff, int nMaxCount, int timeout, int* nReceived)
{
	return PMDPeriphReceive(m_hPeriph, pbuff, (PMDparam*)nReceived, nMaxCount, timeout);
}


//********************************************************
// COM port protocol is slightly different
PMDRPperiphCOM::PMDRPperiphCOM(PMDPeriphHandle* hPeriph) : PMDRPperiph(hPeriph)
{
	m_Address = 0;
}

void PMDRPperiphCOM::SetAddress(int address)
{
	m_Address = address;
}

PMDresult PMDRPperiphCOM::SendPacket(char* pbuff, int nCount)
{
	char RS485address = (char)m_hPeriph->param;
	char checksum = 0;
	char packetlength = nCount;
	char COMpacket[TOTAL_PACKET_LENGTH];
	int i;

	for (i=0; i<nCount; i++)
	{
		checksum += pbuff[i];
	}

	// form the packet to send.
	i = 0;
	if (RS485address > 0)
		COMpacket[i++] = RS485address;
	COMpacket[i++] = checksum;
	COMpacket[i++] = packetlength;
	memcpy(&COMpacket[i], pbuff, nCount);

	return PMDPeriphSend(m_hPeriph, COMpacket, nCount+i, RP_TIMEOUT);
}

PMDresult PMDRPperiphCOM::ReceivePacket(char* pbuff, int nMaxCount, int timeout, int* nReceived)
{
	PMDresult result = PMD_ERR_OK;
	char  RS485address = (char)m_hPeriph->param;
	char  checksum = 0;
	char packetlength = 0;
	DWORD nBytesReceived = 0;
	int nBytesExpected = 2;
	char header[4];
	int i = 0;

	if (RS485address > 0)
	{
		nBytesExpected = 3;
		i = 1;
	}
	result = PMDPeriphReceive(m_hPeriph, &header, &nBytesReceived, nBytesExpected, timeout);
	if (result == PMD_ERR_OK)
	{
		checksum = header[i++];
		packetlength = header[i++];
		nBytesExpected = min(packetlength, nMaxCount);
		result = PMDPeriphReceive(m_hPeriph, &pbuff[3], &nBytesReceived, nBytesExpected, timeout);
		if (result == PMD_ERR_OK)
		{
			nBytesReceived += 3; // calling function expects return header to start at 4th byte
			*nReceived = nBytesReceived;
		}
		// return invalid address AFTER absorbing the packet to keep things in sync.
		if ((RS485address != 0) && (header[0] != RS485address))
			return PMD_ERR_InvalidAddress;
	}

  return result;
}

//*********************************************************************************
// The CAN port protocol includes a sequence number as the first byte
PMDRPperiphCAN::PMDRPperiphCAN(PMDPeriphHandle* hPeriph) : PMDRPperiph(hPeriph)
{
	m_Address = 0;
}

void PMDRPperiphCAN::SetAddress(int address)
{
	m_Address = address;
}

PMDresult PMDRPperiphCAN::SendPacket(char* pbuff, int nCount)
{
	int nSequence;
	int nContinued;
	int nPacketSize;
	char CANpacket[8];
    const char* pDataTx;
	PMDErrorCode result;

	pDataTx = pbuff;
	nContinued = (nCount-1) / 7;
	nSequence = 1;
	CANpacket[0] = nContinued | 0x80;
	while (nContinued-- > 0)
	{
		memcpy(&CANpacket[1], pDataTx, 7);
		result = PMDPeriphSend(m_hPeriph, CANpacket, 8, RP_TIMEOUT);
		if (PMD_ERR_OK != result)
			return result;
		pDataTx += 7;
		CANpacket[0] = nSequence++;
	}
	{
		nPacketSize = (nCount-1) % 7 + 1;
		if (nPacketSize > 0)
		{
			memcpy(&CANpacket[1], pDataTx, nPacketSize);
			nPacketSize += 1;
			result = PMDPeriphSend(m_hPeriph, CANpacket, nPacketSize, RP_TIMEOUT);
		}
	}
	return result;
}

PMDresult PMDRPperiphCAN::ReceivePacket(char* pbuff, int nMaxCount, int timeout, int* nReceived)
{
	int nSequence;
	int nExpectedSequence;
	int nContinued;
	char CANpacket[8];
    char* pDataRx = &pbuff[3];
    int bInitialMessage;
	PMDparam nBytesReceived = 0;
	PMDparam nBytesExpected = 8;
	int nBytesReceivedTotal = 0;
	PMDErrorCode result;

	result = PMDPeriphReceive(m_hPeriph, CANpacket, &nBytesReceived, nBytesExpected, timeout);
	if (PMD_ERR_OK != result)
		return result;

	if (nBytesReceived == 0)
		return PMD_ERR_InsufficientDataReceived;

	bInitialMessage = CANpacket[0] >> 7;
	nContinued = CANpacket[0] & 0x7F;
	if (bInitialMessage)
	{
		nBytesReceived--; // skip header byte
		// copy CAN packet to command packet
		memcpy(pDataRx, &CANpacket[1], nBytesReceived);
		pDataRx += nBytesReceived;
		nBytesReceivedTotal += nBytesReceived;
		nExpectedSequence = 1;
		while (nContinued-- > 0)
		{
			result = PMDPeriphReceive(m_hPeriph, CANpacket, &nBytesReceived, nBytesExpected, timeout);
			if (PMD_ERR_OK == result)
			{
				if (nBytesReceived == 0)
					return PMD_ERR_InsufficientDataReceived;

				bInitialMessage = CANpacket[0] >> 7;
				if (bInitialMessage)
				{
					return PMD_ERR_CommunicationsError;
				}
				nSequence = CANpacket[0] & 0x7F;
				if (nSequence != nExpectedSequence)
				{
					return PMD_ERR_CommunicationsError;
				}
				nExpectedSequence++;
				nBytesReceived--; // skip header byte
				memcpy(pDataRx, &CANpacket[1], nBytesReceived);
				pDataRx += nBytesReceived;
				nBytesReceivedTotal += nBytesReceived;
			}
		}
	}

	nBytesReceivedTotal += 3; // calling function expects header length to be 4 bytes
	*nReceived = nBytesReceivedTotal;

	return result;
}


