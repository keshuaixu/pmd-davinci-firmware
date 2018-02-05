#ifndef _DEF_INC_PMDperiph
#define _DEF_INC_PMDperiph

//
//  "Base Class" definitions for PeriphTransport
//
//  Performance Motion Devices, Inc.
//

#include "PMDtypes.h"
#include "PMDdevice.h"

#define PMD_INVALID_HANDLE      (void*)(0)
#define PMD_CONNECTED_HANDLE    (void*)(1)
#define PMD_WAITFOREVER         ((PMDparam)(-1))

#define PMD_VERIFYHANDLE(handle)        if( handle == NULL ) return PMD_ERR_InvalidHandle;
#define PMD_PERIPHCONNECTED(hPeriph)    if( hPeriph == NULL ) return PMD_ERR_InterfaceNotInitialized; \
                                        if( hPeriph->handle == PMD_INVALID_HANDLE ) return PMD_ERR_NotConnected;
// zero the handle structure so that a call to a function with an uninitialized handle won't cause problems.
#define PMD_INITIALIZEHANDLE(handle)    memset(&handle, 0, sizeof(handle));

#define PMD_IP4_ADDR(a,b,c,d)  (((PMDuint32)((a) & 0xff) << 24) | \
                                ((PMDuint32)((b) & 0xff) << 16) | \
                                ((PMDuint32)((c) & 0xff) << 8)  | \
                                 (PMDuint32)((d) & 0xff) << 0)

#if defined(__cplusplus)
extern "C" {
#endif

// open C-Motion Engine User Packet 'peripheral'
PMDCFunc PMDPeriphOpenCME          (PMDPeriphHandle* hPeriph, PMDDeviceHandle *hDevice);
PMDCFunc PMDPeriphOpenCOM          (PMDPeriphHandle* hPeriph, PMDDeviceHandle *hDevice, PMDparam portnum, PMDSerialBaud baud, PMDSerialParity parity, PMDSerialStopBits stopbits);
PMDCFunc PMDPeriphOpenCAN          (PMDPeriphHandle* hPeriph, PMDDeviceHandle *hDevice, PMDparam addressTX, PMDparam addressRX, PMDparam addressEvent);
PMDCFunc PMDPeriphOpenCANRange     (PMDPeriphHandle* hPeriph, PMDDeviceHandle *hDevice, PMDparam addressRXlo, PMDparam addressRXhi);
PMDCFunc PMDPeriphOpenTCP          (PMDPeriphHandle* hPeriph, PMDDeviceHandle *hDevice, PMDparam ipaddress, PMDparam portnum, PMDparam timeout);
PMDCFunc PMDPeriphOpenUDP          (PMDPeriphHandle* hPeriph, PMDDeviceHandle *hDevice, PMDparam ipaddress, PMDparam portnum);
PMDCFunc PMDPeriphOpenISA          (PMDPeriphHandle* hPeriph, PMDDeviceHandle *hDevice, WORD address, BYTE eventIRQ, PMDDataSize datasize);
PMDCFunc PMDPeriphOpenPIO          (PMDPeriphHandle* hPeriph, PMDDeviceHandle *hDevice, WORD address, BYTE eventIRQ, PMDDataSize datasize);
// open a PCI Prodigy card on a host PC only
PMDCFunc PMDPeriphOpenPCI          (PMDPeriphHandle* hPeriph, PMDparam boardnum);

// PMDPeriphOpenMultiDrop creates a handle to a multidrop (RS485) serial port peripheral for use with PMD serial IONs.
// use the hPeriph handle for all communications and only close the hPeriphParent when done.
// hPeriphParent is a handle to an open serial port peripheral from a call to PMDPeriphOpenCOM.
PMDCFunc PMDPeriphOpenMultiDrop   (PMDPeriphHandle* hPeriph, PMDPeriphHandle* hPeriphParent, PMDparam address);

// for peripherals that can receive asynchronous events, ReceiveEvent will wait until the event is received or timeout has elapsed
PMDCFunc PMDPeriphReceiveEvent    (PMDPeriphHandle* hPeriph, void *data, PMDparam *pnReceived, PMDparam nExpected, PMDparam timeoutms);
PMDCFunc PMDPeriphReceive         (PMDPeriphHandle* hPeriph, void *data, PMDparam *pnReceived, PMDparam nExpected, PMDparam timeoutms);
PMDCFunc PMDPeriphSend            (PMDPeriphHandle* hPeriph, void *data, PMDparam nCount, PMDparam timeoutms);

// for reading and writing 8 or 16 bit data via ISA only
PMDCFunc PMDPeriphRead            (PMDPeriphHandle* hPeriph, void *data, PMDparam offset, PMDparam length);
PMDCFunc PMDPeriphWrite           (PMDPeriphHandle* hPeriph, void *data, PMDparam offset, PMDparam length);

void      PMDPeriphOut            (PMDPeriphHandle* hPeriph, PMDparam offset, PMDparam data);
int       PMDPeriphIn             (PMDPeriphHandle* hPeriph, PMDparam offset);

PMDCFunc PMDPeriphClose           (PMDPeriphHandle* hPeriph);
PMDCFunc PMDPeriphFlush           (PMDPeriphHandle* hPeriph);


#if defined(__cplusplus)
}
#endif

#endif
