#ifndef _DEF_INC_Pfunc
#define _DEF_INC_Pfunc

#if defined(__cplusplus)
extern "C" {
#endif

// forward declerations for local peripherals. Not to be called directly.
PMDresult PMDPCME_Open(PMDPeriphHandle* hPeriph, PMDDeviceHandle *hDevice);
PMDresult PMDPCOM_Open(PMDPeriphHandle* hPeriph, PMDparam portnum, PMDSerialBaud baud, PMDSerialParity parity, PMDSerialStopBits stopbits);
PMDresult PMDPTCP_Open(PMDPeriphHandle* hPeriph, PMDparam ipaddress, PMDparam portnum, PMDparam timeout);
PMDresult PMDPUDP_Open(PMDPeriphHandle* hPeriph, PMDparam ipaddress, PMDparam portnum);
PMDresult PMDPISA_Open(PMDPeriphHandle* hPeriph, PMDparam address, PMDparam eventIRQ, PMDDataSize datasize);
PMDresult PMDPPAR_Open(PMDPeriphHandle* hPeriph, PMDparam address, PMDparam eventIRQ, PMDDataSize datasize);
PMDresult PMDPPCI_Open(PMDPeriphHandle* hPeriph, int board_number);
PMDresult PMDPCAN_Open(PMDPeriphHandle* hPeriph, PMDparam addressTX, PMDparam addressRX, PMDparam addressEvent);
PMDresult PMDPCAN_OpenRange(PMDPeriphHandle* hPeriph, PMDparam addressRXlo, PMDparam addressRXhi);

// interface specific config functions
PMDresult PMDPCAN_AddNodeID(PMDPeriphHandle* hPeriph, PMDPeriphHandle* hPeriphParent, PMDuint8 nodeID);
PMDresult PMDPCOM_SetConfig(PMDPeriphHandle* hPeriph, PMDSerialBaud baud, PMDSerialParity parity, PMDSerialStopBits stopbits);

PMDresult PMDSerial_Sync(PMDPeriphHandle* hPeriph);
PMDresult PMDSerial_Send(void* transport_data, PMDuint8 xCt, PMDuint16* xDat, PMDuint8 rCt, PMDuint16* rDat);
PMDresult PMDSerial_SetConfig(void* transport_data, PMDuint16 serialmode);
PMDresult PMDSerial_Init();

PMDresult PMDCAN_Send(void* transport_data, PMDuint8 xCt, PMDuint16* xDat, PMDuint8 rCt, PMDuint16* rDat);
PMDresult PMDCAN_WaitForEvent(PMDDeviceHandle* hDevice, PMDEvent* event, PMDuint32 timeout);
PMDresult PMDCAN_Init();

PMDresult PMDISA_Send(void* transport_data, PMDuint8 xCt, PMDuint16* xDat, PMDuint8 rCt, PMDuint16* rDat);
PMDresult PMDISA_WaitForEvent(PMDDeviceHandle* hDevice, PMDEvent* event, PMDuint32 timeout);
PMDresult PMDISA_ReadMemory(PMDMemoryHandle *hMemory, void* data, PMDuint32 offset_in_dwords, PMDuint32 dwords_to_read);
PMDresult PMDISA_WriteMemory(PMDMemoryHandle *hMemory, void* data, PMDuint32 offset_in_dwords, PMDuint32 dwords_to_write);
PMDresult PMDISA_Init();

PMDresult PMDPCI_ReadMemory(PMDMemoryHandle *hMemory, void* data, PMDuint32 offset_in_dwords, PMDuint32 dwords_to_read);
PMDresult PMDPCI_WriteMemory(PMDMemoryHandle *hMemory, void* data, PMDuint32 offset_in_dwords, PMDuint32 dwords_to_write);
PMDresult PMDPCI_Init();

PMDresult PMDPAR_Send(void* transport_data, PMDuint8 xCt, PMDuint16* xDat, PMDuint8 rCt, PMDuint16* rDat);
PMDresult PMDPAR_Read(void* transport_data, PMDuint32* data, PMDuint32 offset, PMDuint32 length);
PMDresult PMDPAR_Write(void* transport_data, PMDuint32* data, PMDuint32 offset, PMDuint32 length);
PMDresult PMDPAR_ReadMemory(PMDMemoryHandle *hMemory, void* data, PMDuint32 offset_in_dwords, PMDuint32 dwords_to_read);
PMDresult PMDPAR_WriteMemory(PMDMemoryHandle *hMemory, void* data, PMDuint32 offset_in_dwords, PMDuint32 dwords_to_write);
PMDresult PMDPAR_WaitForEvent(PMDDeviceHandle* hDevice, PMDEvent* event, PMDuint32 timeout);
PMDresult PMDPAR_HardReset(void* transport_data);
PMDresult PMDPAR_WaitUntilReady(void* transport_data);
PMDresult PMDPAR_Init();

#if defined(__cplusplus)
}
#endif


#endif