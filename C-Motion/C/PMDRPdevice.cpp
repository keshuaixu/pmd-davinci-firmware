//
//  PMDRPdevice.cpp -- PMD Resource Protocol device functions
//
//  Performance Motion Devices, Inc.
//

#include "PMDtypes.h"
#include "PMDecode.h"
#include "PMDperiph.h"
#include "PMDtrans.h"
#include "PMDdevice.h"
#include "PMDrpperiph.h"
#include "PMDrpdevice.h"


#define PMDRESULT(_call)  result = _call;
#define PMD_VERIFYDEVICEHANDLE(handle)  \
    if( handle == NULL ) return PMD_ERR_InvalidHandle; \
    if (handle->transport_data == NULL) return PMD_ERR_NotSupported; \
    PMDRPperiph* rp = (PMDRPperiph*)hDevice->transport_data; \
    PMDresult result = PMD_ERR_OK;


//********************************************************
// private functions
//********************************************************
static PMDresult PMDRPReset(void* transport_data)
{
    PMDDeviceHandle* hDevice = (PMDDeviceHandle*)transport_data;
    PMD_VERIFYDEVICEHANDLE(hDevice)

    PMDRESULT(rp->Reset());

    return result;
}

//********************************************************
static PMDresult PMDRPSendCommand(void* transport_data, PMDuint8 xCt, PMDuint16* xDat, PMDuint8 rCt, PMDuint16* rDat)
{
    PMDDeviceHandle* hDevice = (PMDDeviceHandle*)transport_data;
    PMD_VERIFYDEVICEHANDLE(hDevice)

    PMDRESULT(rp->Command( xCt, xDat, rCt, rDat ));

    return result;
}

//********************************************************
static PMDresult PMDRPPeriphClose(PMDPeriphHandle* hPeriph)
{
    PMDDeviceHandle* hDevice = (PMDDeviceHandle*)hPeriph->transport_data;
    PMD_VERIFYDEVICEHANDLE(hDevice)

    PMDRESULT(rp->ClosePeriph( hPeriph->address ));
    hPeriph->handle = PMD_INVALID_HANDLE;

    return result;
}

//********************************************************
static PMDresult PMDRPPeriphSend(PMDPeriphHandle* hPeriph, void* data, PMDparam nCount, PMDparam timeoutms)
{
    PMDDeviceHandle* hDevice = (PMDDeviceHandle*)hPeriph->transport_data;
    PMD_VERIFYDEVICEHANDLE(hDevice)

    PMDRESULT(rp->SendPeriph( hPeriph->address, (char*)data, nCount, timeoutms ));

    return result;
}

//********************************************************
static PMDresult PMDRPPeriphReceive(PMDPeriphHandle* hPeriph, void* data, PMDparam nMaxExpected, PMDparam* pnreceived, PMDparam timeoutms)
{
    PMDDeviceHandle* hDevice = (PMDDeviceHandle*)hPeriph->transport_data;
    PMD_VERIFYDEVICEHANDLE(hDevice)

    PMDRESULT(rp->ReceivePeriph( hPeriph->address, (char*)data, nMaxExpected, timeoutms, pnreceived));

    return result;
}

//********************************************************
static PMDresult PMDRPPeriphWrite(PMDPeriphHandle* hPeriph, void* data, DWORD offset_in_bytes, DWORD words_to_write)
{
    PMDDeviceHandle* hDevice = (PMDDeviceHandle*)hPeriph->transport_data;
    PMD_VERIFYDEVICEHANDLE(hDevice)

    PMDRESULT(rp->WritePeriph( hPeriph->address, (WORD*)data, offset_in_bytes, words_to_write));

    return result;
}

//********************************************************
static PMDresult PMDRPPeriphRead(PMDPeriphHandle* hPeriph, void* data, DWORD offset_in_bytes, DWORD words_to_read)
{
    PMDDeviceHandle* hDevice = (PMDDeviceHandle*)hPeriph->transport_data;
    PMD_VERIFYDEVICEHANDLE(hDevice)

    PMDRESULT(rp->ReadPeriph( hPeriph->address, (WORD*)data, offset_in_bytes, words_to_read));

    return result;
}

//********************************************************
static void PMDRPPeriph_Init(PMDPeriphHandle* hPeriph)
{
    // set the interface type
    hPeriph->type = InterfaceNone;

    hPeriph->transport.Close            = PMDRPPeriphClose;
    hPeriph->transport.Send             = PMDRPPeriphSend;
    hPeriph->transport.Receive          = PMDRPPeriphReceive;
    hPeriph->transport.Read             = NULL;
    hPeriph->transport.Write            = NULL;
    hPeriph->transport.ReceiveEvent     = NULL;
}


//********************************************************
PMDresult PMDRPCOM_Open(PMDPeriphHandle* hPeriph, PMDDeviceHandle* hDevice, PMDparam portnum, PMDSerialBaud baud, PMDSerialParity parity, PMDSerialStopBits stopbits)
{
    int address;
    PMD_VERIFYDEVICEHANDLE(hDevice)

    PMDRESULT(rp->OpenPeriphSerial(&address, portnum, baud, parity, stopbits));

    if (result == PMD_ERR_OK)
    {
        PMDRPPeriph_Init(hPeriph);
        hPeriph->transport_data = hDevice;
        hPeriph->type = InterfaceSerial;
        hPeriph->address = address;
        hPeriph->handle = PMD_CONNECTED_HANDLE; // set the periph handle as connected
    }

    return result;
}

//********************************************************
PMDresult PMDRPCAN_Open(PMDPeriphHandle* hPeriph, PMDDeviceHandle* hDevice, PMDparam transmitid, PMDparam receiveid, PMDparam eventid)
{
    int address;
    PMD_VERIFYDEVICEHANDLE(hDevice)

    PMDRESULT(rp->OpenPeriphCAN(&address, transmitid, receiveid, eventid));

    if (result == PMD_ERR_OK)
    {
        PMDRPPeriph_Init(hPeriph);
        hPeriph->transport_data = hDevice;
        hPeriph->type = InterfaceCAN;
        hPeriph->address = address;
        hPeriph->handle = PMD_CONNECTED_HANDLE; // set the periph handle as connected
    }

    return result;
}

//********************************************************
PMDresult PMDRPUDP_Open(PMDPeriphHandle* hPeriph, PMDDeviceHandle* hDevice, PMDparam ipaddress, PMDparam portnum)
{
    int address;
    PMD_VERIFYDEVICEHANDLE(hDevice)

    PMDRESULT(rp->OpenPeriphUDP(&address, ipaddress, portnum));

    if (result == PMD_ERR_OK)
    {
        PMDRPPeriph_Init(hPeriph);
        hPeriph->transport_data = hDevice;
        hPeriph->type = InterfaceUDP;
        hPeriph->address = address;
        hPeriph->handle = PMD_CONNECTED_HANDLE; // set the periph handle as connected
    }

    return result;
}

//********************************************************
PMDresult PMDRPTCP_Open(PMDPeriphHandle* hPeriph, PMDDeviceHandle *hDevice, PMDparam ipaddress, PMDparam portnum)
{
    int address;
    PMD_VERIFYDEVICEHANDLE(hDevice)

    PMDRESULT(rp->OpenPeriphTCP(&address, ipaddress, portnum));

    if (result == PMD_ERR_OK)
    {
        PMDRPPeriph_Init(hPeriph);
        hPeriph->transport_data = hDevice;
        hPeriph->type = InterfaceTCP;
        hPeriph->address = address;
        hPeriph->handle = PMD_CONNECTED_HANDLE; // set the periph handle as connected
    }

    return result;
}

//********************************************************
PMDresult PMDRPPAR_Open(PMDPeriphHandle* hPeriph, PMDDeviceHandle* hDevice, WORD address, BYTE eventIRQ, PMDDataSize datasize)
{
    int periphaddress;
    PMD_VERIFYDEVICEHANDLE(hDevice)

    PMDRESULT(rp->OpenPeriphPAR(&periphaddress, address, eventIRQ, datasize));

    if (result == PMD_ERR_OK)
    {
        hPeriph->transport.Close            = PMDRPPeriphClose;
        hPeriph->transport.Send             = NULL;
        hPeriph->transport.Receive          = NULL;
        hPeriph->transport.Read             = PMDRPPeriphRead;
        hPeriph->transport.Write            = PMDRPPeriphWrite;
        hPeriph->transport.ReceiveEvent     = NULL;
        hPeriph->transport_data = hDevice;
        hPeriph->type = InterfaceParallel;
        hPeriph->address = periphaddress;
        hPeriph->handle = PMD_CONNECTED_HANDLE; // set the periph handle as connected
    }

    return result;
}

//********************************************************
PMDresult PMDRPISA_Open(PMDPeriphHandle* hPeriph, PMDDeviceHandle* hDevice, WORD address, BYTE eventIRQ, PMDDataSize datasize)
{
    int periphaddress;
    PMD_VERIFYDEVICEHANDLE(hDevice)

    PMDRESULT(rp->OpenPeriphISA(&periphaddress, address, eventIRQ, datasize));

    if (result == PMD_ERR_OK)
    {
        hPeriph->transport.Close            = PMDRPPeriphClose;
        hPeriph->transport.Send             = NULL;
        hPeriph->transport.Receive          = NULL;
        hPeriph->transport.Read             = PMDRPPeriphRead;
        hPeriph->transport.Write            = PMDRPPeriphWrite;
        hPeriph->transport.ReceiveEvent     = NULL;
        hPeriph->transport_data = hDevice;
        hPeriph->type = InterfaceISA;
        hPeriph->address = periphaddress;
        hPeriph->handle = PMD_CONNECTED_HANDLE; // set the periph handle as connected
    }

    return result;
}

//********************************************************
// public functions
//********************************************************
static PMDresult PMDRPMemoryClose(PMDMemoryHandle* hMemory)
{
    PMDDeviceHandle* hDevice = hMemory->hDevice;
    PMD_VERIFYDEVICEHANDLE(hDevice)

    PMDRESULT(rp->CloseMemory( hMemory->address ));

    return result;
}

//********************************************************
PMDresult PMDRPMemoryOpen(PMDMemoryHandle* hMemory, PMDDeviceHandle* hDevice, PMDDataSize datasize, PMDMemoryType memorytype)
{
    int address;
    PMD_VERIFYDEVICEHANDLE(hDevice)

    PMDRESULT(rp->OpenMemory(&address, datasize, memorytype));

    if (result == PMD_ERR_OK)
    {
        hMemory->hDevice  = hDevice;
        hMemory->datasize = datasize;
        hMemory->address  = address;
        hMemory->transport.ReadMemory  = PMDRPMemoryRead;
        hMemory->transport.WriteMemory = PMDRPMemoryWrite;
        hMemory->transport.Close       = PMDRPMemoryClose;
    }
    return result;
}

//********************************************************
PMDresult PMDRPMemoryWrite(PMDMemoryHandle* hMemory, void* data, PMDuint32 offset, PMDuint32 length)
{
    PMDDeviceHandle* hDevice = hMemory->hDevice;
    PMD_VERIFYDEVICEHANDLE(hDevice)

    if (hMemory->datasize == PMDDataSize_32Bit)
        PMDRESULT(rp->WriteMemory(hMemory->address, (PMDuint32*)data, offset, length))
    else if (hMemory->datasize == PMDDataSize_16Bit)
        PMDRESULT(rp->WriteMemory(hMemory->address, (PMDuint16*)data, offset, length))
    else
        result = PMD_ERR_NotSupported;

    return result;
}

//********************************************************
PMDresult PMDRPMemoryRead(PMDMemoryHandle* hMemory, void* data, PMDuint32 offset, PMDuint32 length)
{
    PMDDeviceHandle* hDevice = hMemory->hDevice;
    PMD_VERIFYDEVICEHANDLE(hDevice)

    if (hMemory->datasize == PMDDataSize_32Bit)
        PMDRESULT(rp->ReadMemory(hMemory->address, (PMDuint32*)data, offset, length))
    else if (hMemory->datasize == PMDDataSize_16Bit)
        PMDRESULT(rp->ReadMemory(hMemory->address, (PMDuint16*)data, offset, length))
    else
        result = PMD_ERR_NotSupported;

    return result;
}

//********************************************************
static PMDresult PMDRPClose(PMDDeviceHandle* hDevice)
{
    PMD_VERIFYDEVICEHANDLE(hDevice)
    if (hDevice->transport_data != NULL)
        delete (hDevice->transport_data);

    return PMD_ERR_OK;
}

//********************************************************
PMDCFunc PMDDeviceGetVersion(PMDDeviceHandle* hDevice, PMDuint32* major, PMDuint32* minor)
{
    DWORD version = 0;
    PMD_VERIFYDEVICEHANDLE(hDevice)

    PMDRESULT(rp->GetFirmwareVersion(&version));
    *major = version >> 16;
    *minor = version & 0xFFFF;

    return result;
}

//********************************************************
PMDCFunc PMDDeviceNoOperation(PMDDeviceHandle* hDevice)
{
    PMD_VERIFYDEVICEHANDLE(hDevice)
    PMDRESULT(rp->NoOperation());

    return result;
}

//********************************************************
PMDCFunc PMDDeviceReset(PMDDeviceHandle* hDevice)
{
    PMD_VERIFYDEVICEHANDLE(hDevice)
    PMDRESULT(rp->Reset());

    return result;
}

//********************************************************
PMDCFunc PMDTaskGetState(PMDDeviceHandle* hDevice, PMDTaskState* state)
{
    int status;
    PMD_VERIFYDEVICEHANDLE(hDevice)

    PMDRESULT(rp->GetUserCodeStatus(0, &status));
    *state = (PMDTaskState)status;

    return result;
}

//********************************************************
PMDCFunc PMDTaskStart(PMDDeviceHandle* hDevice)
{
    PMD_VERIFYDEVICEHANDLE(hDevice)

    PMDRESULT(rp->StartUserTasks());

    return result;
}

//********************************************************
PMDCFunc PMDTaskStop(PMDDeviceHandle* hDevice)
{
    PMD_VERIFYDEVICEHANDLE(hDevice)

    PMDRESULT(rp->StopUserTasks());

    return result;
}

//********************************************************
// Send a user code (.bin) file to the CME.
// pdata is a pointer to the data of the .bin file in memory.
// length is the size of the buffer pdata is poitning to.
PMDCFunc PMDDeviceStoreUserCode(PMDDeviceHandle* hDevice, char* pdata, int length)
{
    PMD_VERIFYDEVICEHANDLE(hDevice)
    const int blocksize = MAX_PACKET_DATA_LENGTH;

    PMDRESULT(rp->StoreUserCodeBegin(length));
    if (result == PMD_ERR_OK)
    {
        while (length > blocksize)
        {
            PMDRESULT(rp->StoreUserCodeData(pdata, blocksize));
            if (result != PMD_ERR_OK)
                return result;
            pdata += blocksize;
            length -= blocksize;
        }
        PMDRESULT(rp->StoreUserCodeData(pdata, length));
        if (result != PMD_ERR_OK)
            return result;
        PMDRESULT(rp->StoreUserCodeEnd());
    }

    return result;
}

//********************************************************
PMDCFunc PMDGetUserCodeFileVersion(PMDDeviceHandle* hDevice, PMDuint32* version)
{
    PMD_VERIFYDEVICEHANDLE(hDevice)
    PMDRESULT(rp->GetUserCodeFileVersion(version));

    return result;
}

//********************************************************
PMDCFunc PMDGetUserCodeFileName(PMDDeviceHandle* hDevice, char* filename)
{
    PMD_VERIFYDEVICEHANDLE(hDevice)
    PMDRESULT(rp->GetUserCodeFileName(filename))

    return result;
}

//********************************************************
PMDCFunc PMDGetUserCodeFileDate(PMDDeviceHandle* hDevice, char* date)
{
    PMD_VERIFYDEVICEHANDLE(hDevice)
    PMDRESULT(rp->GetUserCodeFileDate(date));

    return result;
}


//********************************************************
PMDCFunc PMDDeviceGetDefault(PMDDeviceHandle* hDevice, PMDDefaults defaultcode, void* value, PMDDataSize valueSize)
{
    DWORD dwValue = 0;
    PMD_VERIFYDEVICEHANDLE(hDevice)
    PMDRESULT(rp->GetDefault(defaultcode, &dwValue));

    if (valueSize == PMDDataSize_32Bit)
        *(PMDuint32*) value = dwValue;
    else if (valueSize == PMDDataSize_16Bit)
        *(PMDuint16*) value = (PMDuint16)dwValue;
    else
        return PMD_ERR_InvalidParameter;
    return result;
}

//********************************************************
PMDCFunc PMDDeviceSetDefault(PMDDeviceHandle* hDevice, PMDDefaults defaultcode, void* value, PMDDataSize valueSize)
{
    DWORD dwValue = 0;
    PMD_VERIFYDEVICEHANDLE(hDevice)

    if (valueSize == PMDDataSize_32Bit)
        dwValue =* (PMDuint32*) value;
    else if (valueSize == PMDDataSize_16Bit)
        dwValue =* (PMDuint16*) value;
    else
        return PMD_ERR_InvalidParameter;

    PMDRESULT(rp->SetDefault(defaultcode, dwValue));
    return result;
}

//********************************************************
PMDCFunc PMDDeviceSetConsole(PMDDeviceHandle* hDevice, PMDPeriphHandle* hPeriph)
{
    int address;
    PMD_VERIFYDEVICEHANDLE(hDevice)

    if( hPeriph == NULL )
      address = 0; // 0 disables console output
    else
    {
      if( hPeriph->handle == PMD_INVALID_HANDLE )
        return PMD_ERR_NotConnected;
      address = hPeriph->address;
    }
    PMDRESULT(rp->SetConsole(address));

    return result;
}

//********************************************************
PMDCFunc PMDDeviceGetResetCause (PMDDeviceHandle* hDevice, PMDuint16* resetcause, PMDuint16 resetmask)
{
    PMD_VERIFYDEVICEHANDLE(hDevice)
    PMDRESULT(rp->GetResetCause(resetmask, resetcause));

    return result;
}

//********************************************************
// Open a Resource Protocol Device (Prodigy/CME)
PMDCFunc PMDRPDeviceOpen(PMDDeviceHandle* hDevice, PMDPeriphHandle* hPeriph)
{
    PMDZeroMemory(hDevice, sizeof(PMDDeviceHandle));
    PMD_PERIPHCONNECTED(hPeriph)
    if (hPeriph->type == InterfaceCAN)
        hDevice->transport_data = new PMDRPperiphCAN(hPeriph);
    else if (hPeriph->type == InterfaceSerial)
        hDevice->transport_data = new PMDRPperiphCOM(hPeriph);
    else
        hDevice->transport_data = new PMDRPperiph(hPeriph);
    hDevice->type = PMDDeviceTypeResourceProtocol;
    hDevice->hPeriph = hPeriph;
    hDevice->transport.SendCommand  = PMDRPSendCommand;
    hDevice->transport.Reset        = PMDRPReset;
    hDevice->transport.WaitForEvent = NULL;
    hDevice->transport.Close        = PMDRPClose;
    hDevice->transport.ReadMemory   = PMDRPMemoryRead;
    hDevice->transport.WriteMemory  = PMDRPMemoryWrite;

    return PMD_ERR_OK;
}



//********************************************************
// CME user packet 'peripheral'
//********************************************************
static PMDresult PMDPCME_Send(PMDPeriphHandle* hPeriph, void* pData, PMDuint32 nCount, PMDuint32 timeout)
{
    PMDDeviceHandle* hDevice = (PMDDeviceHandle*)hPeriph->transport_data;
    PMD_VERIFYDEVICEHANDLE(hDevice)

    PMDRESULT(rp->SendCME((char*)pData, nCount, timeout));

    return result;
}

//********************************************************
static PMDresult PMDPCME_Receive(PMDPeriphHandle* hPeriph, void* pData, PMDuint32 nCount, PMDparam* pnreceived, PMDuint32 timeout)
{
    PMDDeviceHandle* hDevice = (PMDDeviceHandle*)hPeriph->transport_data;
    PMD_VERIFYDEVICEHANDLE(hDevice)

    PMDRESULT(rp->ReceiveCME((char*)pData, nCount, timeout, pnreceived));

    return result;
}

//********************************************************
static void PMDPCME_Init(PMDPeriphHandle* hPeriph)
{
    // set the interface type
    hPeriph->type = InterfaceNone;

    hPeriph->transport.Close            = NULL;
    hPeriph->transport.Send             = PMDPCME_Send;
    hPeriph->transport.Receive          = PMDPCME_Receive;
    hPeriph->transport.Read             = NULL;
    hPeriph->transport.Write            = NULL;
    hPeriph->transport.ReceiveEvent     = NULL;
}

//********************************************************
PMDCFunc PMDPeriphOpenCME(PMDPeriphHandle* hPeriph, PMDDeviceHandle* hDevice)
{
    if (hDevice == NULL)
        return PMD_ERR_InvalidOperation;

    PMDPCME_Init(hPeriph);

    hPeriph->transport_data = hDevice;
    hPeriph->handle = PMD_CONNECTED_HANDLE;

    return PMD_ERR_OK;
}


