//
//  PMDpar.c -- parallel interface command/data transfer functions
//
//  Performance Motion Devices, Inc.
//

#include "PMDperiph.h"
#include "PMDsys.h"
#include "c-motion.h"
#include "PMDecode.h"
#include "PMDocode.h"
#include "PMDdiag.h"
#include "PMDmutex.h"

#define offsetData      (0)
#define offsetCommand   (2)
#define offsetStatus    (2)
#define offsetReset     (6)

#define readyMask           0x8000
#define readyValue          0x8000
#define hostInterruptMask   0x4000
#define hostInterruptValue  0x4000
#define commandStatusMask   0x2000
#define commandStatusValue  0x2000

#define HandleError(errcode) errcode
#define LF_F "\r\n"

static  PMDuint8 bVerifyChecksum = 1;
static  PMDuint8 bDiagnostics = 1;


// forward declarations
static PMDresult PMDISA_GetInstructionError(void* transport_data);

PMDMutexDefine(xMutexISA);


//********************************************************
PMDresult PMDISA_HardReset(void* transport_data)
{
    PMDDeviceHandle* hDevice = (PMDDeviceHandle*)transport_data;
    PMDPeriphHandle* hPeriph = hDevice->hPeriph;
    int data = 0;

    return PMDPeriphWrite(hPeriph, &data, offsetReset, 1);

}

//********************************************************
// wait for the MP to be ready for the next command
PMDresult PMDISA_WaitForReady(PMDPeriphHandle* hPeriph)
{
    PMDuint16 status;
    PMDresult result;
    int i;

    // wait indefinitely (todo: use interrupts to improve performance)
    for(i=0;i<1000;i++)
    {
        // poll ready port, if not ready, loop
        result = PMDPeriphRead(hPeriph, &status, offsetStatus, 1);
        if (result != PMD_ERR_OK)
          return result;
        if ((status != 0xFFFF) && ((status & readyMask) == readyValue))
                return PMD_ERR_OK;
    }
    return HandleError(PMD_ERR_Timeout);
}

//********************************************************
// wait for the device to be ready for the next command
static PMDresult PMDISA_GetCommandStatus(PMDPeriphHandle* hPeriph)
{
    PMDresult result;
    PMDuint16 status;

    if((result = PMDISA_WaitForReady(hPeriph)) != PMD_ERR_OK)
            return result;

    result = PMDPeriphRead(hPeriph, &status, offsetStatus, 1);
    if ((status & commandStatusMask) == commandStatusValue)
    {
        // command error bit set so call GetInstructionError
        return HandleError(PMDISA_GetInstructionError(hPeriph));
    }

    return PMD_ERR_OK;
}

//********************************************************
// send the command to and get data from the device
static PMDresult PMDISA_ReceiveResponse(PMDPeriphHandle* hPeriph, PMDuint16 length, PMDuint16 buffer[])
{
    PMDresult result;
    PMDuint16 index;

    result = PMD_ERR_InterfaceNotInitialized;

    // get the data from the device
    for(index=0; index<length; index++)
    {
        if((result = PMDISA_WaitForReady(hPeriph)) != PMD_ERR_OK)
                break;
        buffer[index] = PMDPeriphIn(hPeriph, offsetData);
    }

    return result;
}

//********************************************************
// send the command and data to the device
PMDresult PMDISA_Send(void* transport_data, PMDuint8 xCt, PMDuint16* xDat, PMDuint8 rCt, PMDuint16* rDat)
{
    PMDDeviceHandle* hDevice = (PMDDeviceHandle*)transport_data;
    PMDPeriphHandle* hPeriph = hDevice->hPeriph;
    PMDresult result;
    PMDuint16 commandstatus = PMD_ERR_OK;
    PMDuint16 status;
    PMDuint16 index;
    long messageChecksum=0;
    PMDuint16 chipsetChecksum=0;

    if (!PMDMutexLock(xMutexISA))
      return PMD_ERR_MutexTimeout;

    result = PMD_ERR_InterfaceNotInitialized;
    if((result = PMDISA_WaitForReady(hPeriph)) == PMD_ERR_OK)
    {
        // write the command to the device
        PMDPeriphOut(hPeriph, offsetCommand, xDat[0]);

        if((result = PMDISA_WaitForReady(hPeriph)) != PMD_ERR_OK)
                return result;

        // check for a command error before proceeding
        result = PMDPeriphRead(hPeriph, &status, offsetStatus, 1);
        if ((status & commandStatusMask) == commandStatusValue)
        {
            // command error bit set so call GetInstructionError
            commandstatus = HandleError(PMDISA_GetInstructionError(hPeriph));
        }
        commandstatus = PMDISA_GetCommandStatus(hPeriph);

        if (commandstatus == PMD_ERR_OK)
        {
            // write the data to the device one word at a time
            for(index=1; index<xCt; index++)
            {
                if((result = PMDISA_WaitForReady(hPeriph)) != PMD_ERR_OK)
                        break;
                PMDPeriphOut(hPeriph, offsetData, xDat[index]);
            }
            // get the status
            commandstatus = PMDISA_GetCommandStatus(hPeriph);
        }
    }

    // get the data from the device
    for(index=0; index<rCt; index++)
    {
            if((result = PMDISA_WaitForReady(hPeriph)) != PMD_ERR_OK)
                    break;
            rDat[index] = PMDPeriphIn(hPeriph, offsetData);
    }

    if (commandstatus == PMD_ERR_OK)
    {
        if (bVerifyChecksum)
        {
            for(index=0; index<xCt; index++)
                            messageChecksum += xDat[index];
            for(index=0; index<rCt; index++)
                            messageChecksum += rDat[index];
            messageChecksum = messageChecksum & 0xFFFF;

            result = PMDISA_ReceiveResponse(hPeriph, 1, &chipsetChecksum);

            if ( result == PMD_ERR_OK && messageChecksum != chipsetChecksum )
            {
                    if (bDiagnostics)
                            PMDprintf("Checksum failure.  expected: %04x,  got: %04x"LF_F,messageChecksum,chipsetChecksum);
                    commandstatus = PMD_ERR_Checksum;
            }
        }
    }
    PMDMutexUnLock(xMutexISA);

    if (commandstatus != PMD_ERR_OK)
    {
        if (bDiagnostics)
        {
            PMDprintf("Local CP sent:"); //%s ",PMDGetOpcodeText(xDat[0]));
            for(index=0; index<xCt; index++)
                PMDprintf(" %X",xDat[index]);
            PMDprintf(LF_F);
        }
          result = (PMDresult)commandstatus;
    }

    return result;
}

//********************************************************
// for use by parallel interfaces only for retrieving the instruction error
static PMDresult PMDISA_GetInstructionError(void* transport_data)
{
    PMDresult result;
    PMDuint8 xCt;
    PMDuint16 xDat[2];
    PMDuint8 rCt;
    PMDuint16 rDat[2];
    static int GettingError = 0;

    if (GettingError)
        return PMD_ERR_CommunicationsError;

    GettingError = 1;

    xCt = 1;
    xDat[0] = PMDOPGetInstructionError;
    rCt = 1;
    rDat[0] = PMD_NOERROR;

    result = PMDISA_Send(transport_data, xCt, xDat, rCt, rDat);

    if (result == PMD_NOERROR)
    {
        result = (PMDresult)rDat[0];
    }
    GettingError = 0;

    return result;
}

/******************************************************************************

    Function Name: ReadDPRAM()

    Reads from the dual port RAM instead of through the CP chip.
    Returns 0 if sucessful ERR_DPRAM otherwise.
    We must use PMDGEN2_IOCTL_PORT_WRITE and PMDGEN2_IOCTL_PORT_READ
    to access the new DPRAM ports.

    DPRAM base address offsets are:

    0x8 - Config Register
    0xA - Address Register
    0xC - Data Register

 ******************************************************************************/
#define DPRAM_REG_CONFIG    0x08
#define DPRAM_REG_ADDRESS   0x0A
#define DPRAM_REG_DATA      0x0C

/******************************************************************************

    Function Name: ReadMemory()

    Reads from the dual port RAM instead of through the CP chip.

 ******************************************************************************/
PMDresult PMDISA_ReadMemory(PMDMemoryHandle *hMemory, PMDuint32* data, PMDuint32 offset_in_dwords, PMDuint32 dwords_to_read)
{
    PMDPeriphHandle* hPeriph = hMemory->hDevice->hPeriph;
    PMDuint32 i;
    PMDuint16 offset_in_words;
    PMDuint32 word_read_lo;
    PMDuint32 word_read_hi;

    if (offset_in_dwords + dwords_to_read > 0x4000)
        return HandleError(PMD_ERR_ParameterOutOfRange);

    offset_in_words = (PMDuint16)offset_in_dwords * 2;

    // write the command to the device
    PMDPeriphOut(hPeriph, DPRAM_REG_ADDRESS, offset_in_words);

    // read the data
    for (i=0; i<dwords_to_read; i++)
    {
        word_read_hi = PMDPeriphIn(hPeriph, DPRAM_REG_DATA);
        word_read_lo = PMDPeriphIn(hPeriph, DPRAM_REG_DATA);
        data[i] = word_read_hi << 16 | word_read_lo;
    }

    return PMD_ERR_OK;
}

/******************************************************************************

    Function Name: WriteMemory()

    Writes to the dual port RAM instead of through the CP chip.

 ******************************************************************************/
PMDresult PMDISA_WriteMemory(PMDMemoryHandle *hMemory, PMDuint32* data, PMDuint32 offset_in_dwords, PMDuint32 dwords_to_write)
{
    PMDPeriphHandle* hPeriph = hMemory->hDevice->hPeriph;
    PMDuint32 i;
    PMDuint16 offset_in_words;
    PMDuint16 word_write_lo;
    PMDuint16 word_write_hi;

    if (offset_in_dwords + dwords_to_write > 0x4000)
        return HandleError(PMD_ERR_ParameterOutOfRange); // DPRAM boundary exceeded

    offset_in_words = (PMDuint16)offset_in_dwords * 2;

    // write the command to the device
    PMDPeriphOut(hPeriph, DPRAM_REG_ADDRESS, offset_in_words);

    // write the data
    for (i=0; i<dwords_to_write; i++)
    {
        word_write_hi = (PMDuint16)((data[i] >> 16) & 0x0000FFFF);
        word_write_lo = (PMDuint16)(data[i] & 0x0000FFFF);

        PMDPeriphOut(hPeriph, DPRAM_REG_DATA, word_write_hi);
        PMDPeriphOut(hPeriph, DPRAM_REG_DATA, word_write_lo);
    }

    return PMD_ERR_OK;
}

//********************************************************
PMDresult PMDISA_WaitForEvent(PMDDeviceHandle* phDevice, PMDEvent* event_data, PMDuint32 timeout)
{
PMDresult result;
PMDPeriphHandle* hPeriph = phDevice->hPeriph;

    if (hPeriph->transport.ReceiveEvent == NULL)
        return PMD_ERR_NotSupported;
    if (event_data == NULL)
        return PMD_ERR_InvalidParameter;
    result = hPeriph->transport.ReceiveEvent( hPeriph, NULL, 0, NULL, timeout);
    if (result == PMD_ERR_OK)
    {
        return PMDDeviceGetEvent(phDevice, event_data);
    }
    return result;
}

//********************************************************
PMDresult PMDISA_Init()
{
    PMDMutexCreate(xMutexISA);

    return PMD_NOERROR;
}

