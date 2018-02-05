
#if defined(__cplusplus)
extern "C" {
#endif

PMDresult PMDRPCOM_Open(PMDPeriphHandle* hPeriph, PMDDeviceHandle *hDevice, PMDparam portnum, PMDSerialBaud baud, PMDSerialParity parity, PMDSerialStopBits stopbits);
PMDresult PMDRPCAN_Open(PMDPeriphHandle* hPeriph, PMDDeviceHandle *hDevice, PMDparam transmitid, PMDparam receiveid, PMDparam eventid);
PMDresult PMDRPPAR_Open(PMDPeriphHandle* hPeriph, PMDDeviceHandle *hDevice, WORD address, BYTE eventIRQ, PMDDataSize datasize);
PMDresult PMDRPISA_Open(PMDPeriphHandle* hPeriph, PMDDeviceHandle *hDevice, WORD address, BYTE eventIRQ, PMDDataSize datasize);
PMDresult PMDRPUDP_Open(PMDPeriphHandle* hPeriph, PMDDeviceHandle *hDevice, PMDparam ipaddress, PMDparam portnum);
PMDresult PMDRPTCP_Open(PMDPeriphHandle* hPeriph, PMDDeviceHandle *hDevice, PMDparam ipaddress, PMDparam portnum);
PMDresult PMDRPMultiDrop_Open(PMDPeriphHandle* hPeriph, PMDDeviceHandle *hDevice, PMDPeriphHandle* hPeriphParent, PMDparam nodeID);

PMDresult PMDRPMemoryOpen(PMDMemoryHandle *hMemory, PMDDeviceHandle *hDevice, PMDDataSize datasize, PMDMemoryType memorytype);
PMDresult PMDRPMemoryWrite(PMDMemoryHandle *hMemory, void *data, PMDuint32 offset, PMDuint32 length);
PMDresult PMDRPMemoryRead(PMDMemoryHandle *hMemory, void *data, PMDuint32 offset, PMDuint32 length);


#if defined(__cplusplus)
}
#endif
