#ifndef _DEF_INC_PMDdevice
#define _DEF_INC_PMDdevice

//
//  PMDdevice.h -- Definitions for Device functions
//
//  Performance Motion Devices, Inc.
//

#include "PMDtypes.h"
#include "PMDRPtypes.h"
#include "PMDecode.h"
#include "PMDintf.h"

#include <string.h>
#define PMDZeroMemory(dest, count)  if (dest) memset(dest, 0, count);

typedef enum PMDDeviceTypeEnum {
    PMDDeviceTypeNone             = 0,
    PMDDeviceTypeResourceProtocol = 1,
    PMDDeviceTypeMotionProcessor  = 2
} PMDDeviceType;


#pragma pack(4)

// Event structures
typedef struct tagPMDAxisEvent {
    PMDAxis             axis;
    PMDuint16           event;
} PMDEvent;

// Peripheral structures
struct tagPMDPeriphHandle;
typedef struct tagPMDPeriphTransport {
    PMDresult (*Close)       (struct tagPMDPeriphHandle* hPeriph);
    PMDresult (*Send)        (struct tagPMDPeriphHandle* hPeriph, void *pData, PMDparam length, PMDparam timeout);
    PMDresult (*Receive)     (struct tagPMDPeriphHandle* hPeriph, void *pData, PMDparam length, PMDparam *pnreceived, PMDparam timeout);
    PMDresult (*Write)       (struct tagPMDPeriphHandle* hPeriph, void *pData, PMDparam offset, PMDparam length);
    PMDresult (*Read)        (struct tagPMDPeriphHandle* hPeriph, void *pData, PMDparam offset, PMDparam length);
    PMDresult (*ReceiveEvent)(struct tagPMDPeriphHandle* hPeriph, void *pData, PMDparam length, PMDparam *pnreceived, PMDparam timeout);
} PMDPeriphTransport;

typedef struct tagPMDPeriphHandle {
    void*               handle;         // device dependent handle
    void*               transport_data; // peripheral dependant data pointer
    PMDparam            address;        // peripheral dependant address or PRP address if communicating to a peripheral over PRP
    PMDInterfaceType    type;
    PMDPeriphTransport  transport;
    PMDparam            param;          // peripheral dependant parameter which may or may not be used
} PMDPeriphHandle;


// Device structures
struct tagPMDDeviceHandle;
struct tagPMDMemoryHandle;

typedef struct tagPMDDeviceTransport {
    PMDresult (*SendCommand) (void*, PMDuint8 xCt, PMDuint16* xDat, PMDuint8 rCt, PMDuint16* rDat);
    // the following Memory access functions are for non-CME Prodigy cards with DPRAM
    PMDresult (*WriteMemory) (struct tagPMDMemoryHandle* hMemory, void *pData, PMDuint32 offset, PMDuint32 length);
    PMDresult (*ReadMemory)  (struct tagPMDMemoryHandle* hMemory, void *pData, PMDuint32 offset, PMDuint32 length);
    PMDresult (*WaitForEvent)(struct tagPMDDeviceHandle* hDevice, PMDEvent* pEvent, PMDuint32 timeout);
    PMDresult (*Reset)       (void*);
    PMDresult (*Close)       (struct tagPMDDeviceHandle* hDevice);
} PMDDeviceTransport;

typedef struct tagPMDDeviceHandle {
    void*               transport_data;
    PMDPeriphHandle*    hPeriph;        // handle to the peripheral that the device is connected to
    PMDDeviceType       type;
    PMDDeviceTransport  transport;
} PMDDeviceHandle;


// Memory structures
typedef struct tagPMDMemoryTransport {
    PMDresult (*WriteMemory) (struct tagPMDMemoryHandle* hMemory, void *pData, PMDuint32 offset, PMDuint32 length);
    PMDresult (*ReadMemory)  (struct tagPMDMemoryHandle* hMemory, void *pData, PMDuint32 offset, PMDuint32 length);
    PMDresult (*Close)       (struct tagPMDMemoryHandle* hMemory);
} PMDMemoryTransport;

typedef struct tagPMDMemoryHandle {
    PMDDeviceHandle*    hDevice;        // handle of the device containing the memory
    PMDDataSize         datasize;
    PMDparam            address;        // base address of local memory or PRP address if communicating to memory over PRP
    PMDMemoryTransport  transport;
} PMDMemoryHandle;


// Axis structures
typedef struct tagPMDCPTransport {
    PMDresult (*SendCommand)(void*, PMDuint8 xCt, PMDuint16* xDat, PMDuint8 rCt, PMDuint16* rDat);
} PMDCPTransport;

typedef struct tagPMDAxisHandle {
    PMDAxis             axis;
    void*               transport_data;
    PMDCPTransport      transport;
} PMDAxisHandle;

typedef PMDAxisHandle* PMDAxisInterface;

#pragma pack()


#if defined(__cplusplus)
extern "C" {
#endif

PMDCFunc PMDMPDeviceOpen			(PMDDeviceHandle *hDevice, PMDPeriphHandle *hPeriph);
PMDCFunc PMDRPDeviceOpen			(PMDDeviceHandle *hDevice, PMDPeriphHandle *hPeriph);
PMDCFunc PMDDeviceClose				(PMDDeviceHandle *hDevice);
PMDCFunc PMDDeviceReset				(PMDDeviceHandle *hDevice);
PMDCFunc PMDDeviceGetVersion		(PMDDeviceHandle *hDevice, PMDuint32 *major, PMDuint32 *minor);
PMDCFunc PMDDeviceGetDefault		(PMDDeviceHandle *hDevice, PMDDefaults defaultcode, void *value, PMDDataSize valueSize);
PMDCFunc PMDDeviceSetDefault		(PMDDeviceHandle *hDevice, PMDDefaults defaultcode, void *value, PMDDataSize valueSize);
PMDCFunc PMDTaskGetState			(PMDDeviceHandle *hDevice, PMDTaskState *state);
PMDCFunc PMDTaskStart				(PMDDeviceHandle *hDevice);
PMDCFunc PMDTaskStop				(PMDDeviceHandle *hDevice);
PMDCFunc PMDDeviceStoreUserCode		(PMDDeviceHandle *hDevice, char* pdata, int length);
PMDCFunc PMDGetUserCodeFileVersion	(PMDDeviceHandle *hDevice, PMDuint32 *version);
PMDCFunc PMDGetUserCodeFileName		(PMDDeviceHandle *hDevice, char * filename);
PMDCFunc PMDGetUserCodeFileDate		(PMDDeviceHandle *hDevice, char * date);
PMDCFunc PMDDeviceNoOperation		(PMDDeviceHandle *hDevice);
PMDCFunc PMDDeviceGetResetCause (PMDDeviceHandle *hDevice, PMDuint16* resetcause, PMDuint16 resetmask);
PMDCFunc PMDDeviceSetConsole    (PMDDeviceHandle *hDevice, PMDPeriphHandle *hPeriph);

PMDCFunc PMDWaitForEvent        (PMDDeviceHandle *hDevice, PMDEvent* event, PMDuint32 timeout);
PMDCFunc PMDDeviceGetEvent      (PMDDeviceHandle* phDevice, PMDEvent* event_data);

PMDCFunc PMDAxisOpen            (PMDAxisHandle *hAxis, PMDDeviceHandle *hDevice, PMDAxis axis_number);
PMDCFunc PMDAtlasAxisOpen       (PMDAxisInterface hSourceAxis, PMDAxisInterface hAtlasAxis);
PMDCFunc PMDAxisClose           (PMDAxisHandle *hAxis);

PMDCFunc PMDMemoryOpen          (PMDMemoryHandle *hMemory, PMDDeviceHandle *hDevice, PMDDataSize datasize, PMDMemoryType memorytype);
                                // offset and length are in units of datasize that the handle is opened with.
PMDCFunc PMDMemoryRead          (PMDMemoryHandle *hMemory, void *data, PMDuint32 offset, PMDuint32 length);
PMDCFunc PMDMemoryWrite         (PMDMemoryHandle *hMemory, void *data, PMDuint32 offset, PMDuint32 length);
PMDCFunc PMDMemoryClose         (PMDMemoryHandle *hMemory);

#if defined(__cplusplus)
}
#endif

#endif
