///////////////////////////////////////////////////////////////////////////////
//
//  PMDPpci.c -- parallel interface command/data transfer functions for
//               using the PCI bus interface via the PLX MS Windows driver.
//
///////////////////////////////////////////////////////////////////////////////

#include <stdio.h>
#include <stdlib.h>

#include "PMDtypes.h"
#include "PMDecode.h"
#include "PMDperiph.h"
#include "PMDsys.h"
#include "PLX\plxapi.h"
#include "PMDPfunc.h"

// only required if we are running in diagnostics mode
#include "PMDdiag.h"

// PCI SubDevice IDs
const int CARD_DK_NAVIGATOR  = 0x2811;
const int CARD_DK_MAGELLAN   = 0x2996;
const int CARD_MB_NAVIGATOR  = 0x2746;
const int CARD_MB_MAGELLAN   = 0x2960; // Prodigy card 
const int CARD_MB_PRODIGYCME = 0x3249; // Prodigy/CME card 

// Prodigy CME base address offsets
#define PCI_COMMAND_REG         0x10    // write only register that signals there's a command 
#define PCI_STATUS_REG          0x12    // read only register that reports ready status
#define PCI_IRQ_ENA_REG         0x14    // IRQ out enable register
#define PCI_COMMAND_WRITE       1       // command from HOST
#define PCI_COMMAND_READ_DONE   2       // Tells ARM we're done reading
#define PCI_COMMAND_RESET       4
#define PCI_COMMAND_CONNECT     8
#define PCI_STATUS_READY        1
#define PACKET_DATA_OFFSET      4

#define MAP_DPRAM

#define PLX_ERROR(_call) \
{   PLX_STATUS  rc = _call; \
    if (ApiSuccess != rc) \
    { \
            PMDprintf("PLX error %d calling %s.\n", rc, #_call); \
            result = PMD_ERR_Driver; \
    } \
}


//static PLX_NOTIFY_OBJECT PlxNotifyObject; // can only have one process waiting on an interrupt

typedef struct tagPMDPCIIOTransportData {

    PMDuint16 bVerifyChecksum;
    PMDuint16 bDiagnostics;
    PMDuint16 nBoardNo;
    int nDeviceID;
    int nDPRAMBarIndex;
    U32 nBasePortAddress;
    U32 *pDPRAMAddress;
    int bInterruptRegistered;

    PLX_DEVICE_OBJECT PlxDeviceObject;
    PLX_NOTIFY_OBJECT PlxNotifyObject;// can only have one process waiting on an interrupt

} PMDPCIIOTransportData;

//*****************************************************************************
static PMDresult PMDPPCI_Write16(PMDPeriphHandle* hPeriph, void *pData, PMDparam offset, PMDparam length)
{
    PMDPCIIOTransportData* PIOtransport_data = (PMDPCIIOTransportData*)(hPeriph->transport_data);
    PMDresult result = PMD_ERR_OK;
    PLX_DEVICE_OBJECT *pDevice = &PIOtransport_data->PlxDeviceObject;
    U64                port = PIOtransport_data->nBasePortAddress + offset;
    U32                ByteCount = 2 * length;
    PLX_ACCESS_TYPE    AccessType = BitSize16;

    PLX_ERROR(PlxPci_IoPortWrite(
        pDevice,
        port,
        pData,
        ByteCount,
        AccessType
    ));

    return result;
}

//*****************************************************************************
static PMDresult PMDPPCI_Read16(PMDPeriphHandle* hPeriph, void *pData, PMDparam offset, PMDparam length)
{
    PMDPCIIOTransportData* PIOtransport_data = (PMDPCIIOTransportData*)hPeriph->transport_data;
    PMDresult result = PMD_ERR_OK;
    PLX_DEVICE_OBJECT *pDevice = &PIOtransport_data->PlxDeviceObject;
    U64                port = PIOtransport_data->nBasePortAddress + offset;
    U32                ByteCount = 2 * length;
    PLX_ACCESS_TYPE    AccessType = BitSize16;

    PLX_ERROR( PlxPci_IoPortRead(
        pDevice,
        port,
        pData,
        ByteCount,
        AccessType
    ));

    return result;
}

//*****************************************************************************
static void SetRegister(PMDPeriphHandle* hPeriph, PMDuint32 reg, PMDuint16 regvalue)
{
	PMDPPCI_Write16(hPeriph, &regvalue, reg, 1);
}

//*****************************************************************************
static PMDresult WriteMemory(PMDPeriphHandle* hPeriph, PMDuint32* data, PMDuint32 offset_in_dwords, PMDuint32 dwords_to_write)
{
	PMDuint32 i;
    PMDPCIIOTransportData* PIOtransport_data = (PMDPCIIOTransportData*)hPeriph->transport_data;
	PMDuint32* pDPRAMAddress = PIOtransport_data->pDPRAMAddress;

	if (!pDPRAMAddress)
		return PMD_ERR_NotSupported;

	for (i=0;i<dwords_to_write;i++)
	{
		pDPRAMAddress[i+offset_in_dwords] = data[i];
	}
    return PMD_ERR_OK;
}

//*****************************************************************************
static PMDresult ReadMemory(PMDPeriphHandle* hPeriph, PMDuint32* data, PMDuint32 offset_in_dwords, PMDuint32 dwords_to_read)
{
	PMDuint32 i;
    PMDPCIIOTransportData* PIOtransport_data = (PMDPCIIOTransportData*)hPeriph->transport_data;
	PMDuint32* pDPRAMAddress = PIOtransport_data->pDPRAMAddress;

	if (!pDPRAMAddress)
		return PMD_ERR_NotSupported;

	for (i=0;i<dwords_to_read;i++)
	{
		data[i] = pDPRAMAddress[i+offset_in_dwords];
	}
    return PMD_ERR_OK;
}

//*****************************************************************************
PMDresult PMDPCI_ReadMemory(PMDMemoryHandle *hMemory, void* data, PMDuint32 offset_in_dwords, PMDuint32 dwords_to_read)
{
    PMDPeriphHandle* hPeriph = hMemory->hDevice->hPeriph;
	if (hMemory->datasize != PMDDataSize_32Bit)
		return PMD_ERR_NotSupported;

    return ReadMemory(hPeriph, (PMDuint32*)data, offset_in_dwords, dwords_to_read);
}

//*****************************************************************************
PMDresult PMDPCI_WriteMemory(PMDMemoryHandle *hMemory, void* data, PMDuint32 offset_in_dwords, PMDuint32 dwords_to_write)
{
    PMDPeriphHandle* hPeriph = hMemory->hDevice->hPeriph;
	if (hMemory->datasize != PMDDataSize_32Bit)
		return PMD_ERR_NotSupported;

    return WriteMemory(hPeriph, (PMDuint32*)data, offset_in_dwords, dwords_to_write);
}


//*****************************************************************************
static PMDresult WaitUntilReady( PMDPeriphHandle* hPeriph, long msec )
{
    PMDuint32 stopTime, currentTime;
	PMDuint32 iterations = 0;
	PMDuint16 status;

    // GetTickCount returns time in ms
    stopTime = GetTickCount() + msec;

    do
    {
        // poll ready port, if not ready, loop
		//ReadWord(PCI_STATUS_REG, &status);
		PMDPPCI_Read16(hPeriph, &status, PCI_STATUS_REG, 1);
		iterations++;
		if (status & PCI_STATUS_READY)
		{	// clear the bit
			SetRegister(hPeriph, PCI_STATUS_REG, PCI_STATUS_READY);
			return PMD_ERR_OK;
		}

        currentTime = GetTickCount();
	}
    while( currentTime < stopTime );

    return PMD_ERR_Timeout;
}

//*****************************************************************************
// this function is equal to SendPacket rather than a general purpose bytewise send function
static PMDresult PMDPPCI_Send(PMDPeriphHandle* hPeriph, void *data, PMDparam nCount, PMDparam timeoutms)
{
    int nCountSent = 0;
	PMDuint32 offset_in_dwords;
	PMDuint32 dwords_to_write;
	PMDuint32* pData32;

    PMD_PERIPHCONNECTED(hPeriph)

    if (nCount > 0)
    {
		SetRegister(hPeriph, PCI_STATUS_REG, PCI_STATUS_READY);

		// first write the length of the packet at location 0
		offset_in_dwords = 0;
		dwords_to_write = 1;
		pData32 = (PMDuint32*)&nCount;
		WriteMemory(hPeriph, pData32, offset_in_dwords, dwords_to_write);
		
		// then write the packet at PACKET_DATA_OFFSET
		offset_in_dwords = PACKET_DATA_OFFSET / 4;
		dwords_to_write = nCount / 4;
		if (nCount % 4)
			dwords_to_write++; // round up to word boundary
		pData32 = (PMDuint32*)data;
		WriteMemory(hPeriph, pData32, offset_in_dwords, dwords_to_write);

		// tell the ARM a command packet is ready
		SetRegister(hPeriph, PCI_COMMAND_REG, PCI_COMMAND_WRITE);
    }
    return PMD_ERR_OK;
}

//*****************************************************************************
static PMDresult PMDPPCI_Receive(PMDPeriphHandle* hPeriph, void *data, PMDparam nCount, PMDparam *pnReceived, PMDparam timeoutms)
{
	PMDuint32 offset_in_dwords;
	PMDuint32 dwords_to_read;
	PMDuint32* pData32;
	PMDuint32 nCountReceived;

    PMD_PERIPHCONNECTED(hPeriph)

    *pnReceived = 0;
    if (nCount > 0)
    {
		if (PMD_ERR_Timeout == WaitUntilReady(hPeriph, timeoutms))
			return PMD_ERR_Timeout;

		// first get the length of the packet at location 0
		offset_in_dwords = 0;
		dwords_to_read = 1;
		pData32 = (PMDuint32*)&nCountReceived;
		ReadMemory(hPeriph, pData32, offset_in_dwords, dwords_to_read);

		// then get the packet at PACKET_DATA_OFFSET
		offset_in_dwords = PACKET_DATA_OFFSET / 4;
		dwords_to_read = nCountReceived / 4;
		if (nCountReceived % 4)
			dwords_to_read++;
		pData32 = (PMDuint32*)data;
		ReadMemory(hPeriph, pData32, offset_in_dwords, dwords_to_read);

		*pnReceived = nCountReceived;
    }
    return PMD_ERR_OK;
}


//*****************************************************************************
PMDresult PMDPPCI_RegisterInterrupt(PMDPCIIOTransportData* PIOtransport_data)
{
    PMDresult result = PMD_ERR_OK;
    PLX_DEVICE_OBJECT *pDevice = &PIOtransport_data->PlxDeviceObject;
    PLX_NOTIFY_OBJECT *pPlxNotifyObject = &PIOtransport_data->PlxNotifyObject;
    PLX_INTERRUPT     PlxIntr;

    memset(&PlxIntr, 0, sizeof(PLX_INTERRUPT));
    PlxIntr.LocalToPci = 1;
    PLX_ERROR( PlxPci_NotificationRegisterFor(pDevice, &PlxIntr, pPlxNotifyObject) );
    PIOtransport_data->bInterruptRegistered = TRUE;

    return result;
}

//*****************************************************************************
PMDresult PMDPPCI_InitPort(PMDPCIIOTransportData* transport_data)
{
    PMDresult result = PMD_ERR_OK;
    PLX_DEVICE_KEY device;
    PLX_PCI_BAR_PROP BarMagellan;
    U8 DeviceNumber;
    U8 PLXVersionMajor;
    U8 PLXVersionMinor;
    U8 PLXVersionRevision;
    U32 nBasePortAddress;
    int nDeviceID;
    int nMagellanBar = 3;
    int nDPRAMBar = 2;

    memset(&device, PCI_FIELD_IGNORE, sizeof(PLX_DEVICE_KEY));

    device.DeviceId   = 0x9030; 
    device.VendorId   = 0x10b5;  //PLX vendor id

    transport_data->bInterruptRegistered = FALSE;

    PLX_ERROR( PlxPci_ApiVersion(
        &PLXVersionMajor,
        &PLXVersionMinor,
        &PLXVersionRevision) );

    PMDprintf("PLX API version %d.%d%d\n", PLXVersionMajor, PLXVersionMinor, PLXVersionRevision );

    DeviceNumber = (U8)transport_data->nBoardNo;
    
    PLX_ERROR( PlxPci_DeviceFind(&device, DeviceNumber) );
  

    /* Open the device */
    PLX_ERROR( PlxPci_DeviceOpen (&device, &transport_data->PlxDeviceObject) );

    PLX_ERROR(PlxPci_DriverVersion(
        &transport_data->PlxDeviceObject,
        &PLXVersionMajor,
        &PLXVersionMinor,
        &PLXVersionRevision));

    PMDprintf("PLX driver version %d.%d%d\n", PLXVersionMajor, PLXVersionMinor, PLXVersionRevision );

    // get PCI SubsystemID
    nDeviceID = transport_data->PlxDeviceObject.Key.SubDeviceId;
    nBasePortAddress = 0;

    // get io port addresses for PMD developer's kit boards
    if (nDeviceID == CARD_DK_NAVIGATOR
    ||  nDeviceID == CARD_DK_MAGELLAN)
    {
        nMagellanBar = 3;
        nDPRAMBar = 2;
    }
    // get io port addresses for PMD Prodigy boards
    else if (nDeviceID == CARD_MB_NAVIGATOR
         ||  nDeviceID == CARD_MB_MAGELLAN)
    {
        nMagellanBar = 2;
        nDPRAMBar = 4;
    }
    else if (nDeviceID == CARD_MB_PRODIGYCME)
    {
        nMagellanBar = 2;
        nDPRAMBar = 5;
    }
    else
    {
        PMDprintf( "No suppported PCI card found. SubVendorID: %d\n", nDeviceID );
        return PMD_ERR_Driver;
    }

    PLX_ERROR( PlxPci_PciBarProperties( &transport_data->PlxDeviceObject, (U8)nMagellanBar, &BarMagellan) );

    nBasePortAddress = (U32)BarMagellan.Physical;

    if (nBasePortAddress == 0)
    {
        PMDprintf( "Invalid PCI port address: %d\n", nBasePortAddress );
        return PMD_ERR_Driver;
    }

    transport_data->nDeviceID = nDeviceID;
    transport_data->nBasePortAddress = nBasePortAddress;
    transport_data->nDPRAMBarIndex = nDPRAMBar;
    transport_data->pDPRAMAddress = 0;
#ifdef MAP_DPRAM
    PLX_ERROR( PlxPci_PciBarMap(&transport_data->PlxDeviceObject, (U8)nDPRAMBar, (void**)&transport_data->pDPRAMAddress));
#endif

    if (result == PMD_ERR_OK)
        result = PMDPPCI_RegisterInterrupt(transport_data);

    return result; 
}

//*****************************************************************************
PMDresult PMDPPCI_Close(PMDPeriphHandle* hPeriph)
{
    PMDresult result = PMD_ERR_OK;
    PMDPCIIOTransportData* PIOtransport_data = (PMDPCIIOTransportData*)hPeriph->transport_data;
    PLX_DEVICE_OBJECT *pDevice = &PIOtransport_data->PlxDeviceObject;
    PLX_NOTIFY_OBJECT *pPlxNotifyObject = &PIOtransport_data->PlxNotifyObject;

    if (PIOtransport_data->bInterruptRegistered)
        PLX_ERROR( PlxPci_NotificationCancel(pDevice, pPlxNotifyObject) );
    PIOtransport_data->bInterruptRegistered = FALSE;
#ifdef MAP_DPRAM
    PLX_ERROR( PlxPci_PciBarUnmap(&PIOtransport_data->PlxDeviceObject, (void**)&PIOtransport_data->pDPRAMAddress) );
#endif
    PLX_ERROR( PlxPci_DeviceClose( pDevice ) );
    free(PIOtransport_data);
    hPeriph->transport_data = NULL;
    hPeriph->handle = PMD_INVALID_HANDLE; // set the periph handle as not connected

    return PMD_ERR_OK;
}

//********************************************************
static void PMDPPCI_Init(PMDPeriphHandle* hPeriph)
{
    // set the interface type 
    hPeriph->type = InterfacePCI;

    hPeriph->transport.Close    = PMDPPCI_Close;
    hPeriph->transport.Send     = PMDPPCI_Send;
    hPeriph->transport.Receive  = PMDPPCI_Receive;
    hPeriph->transport.Read     = PMDPPCI_Read16;
    hPeriph->transport.Write    = PMDPPCI_Write16;
}

//*****************************************************************************
PMDresult PMDPPCI_Open(PMDPeriphHandle* hPeriph, int board_number)
{
    PMDresult result = PMD_ERR_OK;
    PMDPCIIOTransportData* pPCItransport_data;

    pPCItransport_data = (PMDPCIIOTransportData*) malloc( sizeof(PMDPCIIOTransportData) );
    hPeriph->transport_data = pPCItransport_data;
    pPCItransport_data->nBoardNo = board_number;

    PMDPPCI_Init(hPeriph);
    result = PMDPPCI_InitPort(pPCItransport_data);
    if (result != PMD_ERR_OK)
        PMDPPCI_Close(hPeriph);
    hPeriph->handle = PMD_CONNECTED_HANDLE; // set the periph handle as connected

    return result; 
}
