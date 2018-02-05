///////////////////////////////////////////////////////////////////////////////
//
//  PMDpar.c -- parallel interface command/data transfer functions for using
//              a parallel bus interface such as PCI or ISA or 16-bit bus.
//
///////////////////////////////////////////////////////////////////////////////

#include "PMDtypes.h"
#include "PMDecode.h"
#include "PMDperiph.h"
#include "PMDmutex.h"
#include "PMDPfunc.h"
#include "PMDsys.h"

// only required if we are running in diagnostics mode
#include "PMDdiag.h"

/* local constants */
const int RDY_MASK = 0x8000;
const int INT_MASK = 0x4000;
const int ERR_MASK = 0x2000;
const PMDparam CommandPortOffset = 2;
const PMDparam DataPortOffset = 0;
const int CommandTimeoutms = 100;
const int bVerifyChecksum = 1;
const int bDiagnostics = 1;

PMDMutexDefine(xMutexPAR)

PMDresult PMDPAR_HandleError(void* transport_data);
PMDresult PMDPAR_GetCommandStatus(void* transport_data);

//*****************************************************************************
PMDresult PMDPAR_Send(void* transport_data, PMDuint8 xCt, PMDuint16* xDat, PMDuint8 rCt, PMDuint16* rDat)
{
    PMDDeviceHandle* hDevice = (PMDDeviceHandle*)transport_data;
    PMDPeriphHandle* hPeriph = hDevice->hPeriph;
    PMDresult result = PMD_NOERROR;

    int i;
    PMDuint16 cmd = xDat[0];
    PMDuint16 cksum;
    PMDuint16 calcSum;
    PMDuint16 datawordcount = xCt-1;

    PMD_PERIPHCONNECTED(hPeriph);

    if (!PMDMutexLock(xMutexPAR))
      return PMD_ERR_MutexTimeout;

    result = PMDPAR_WaitUntilReady(transport_data);

    if (result == PMD_ERR_OK)
    {
        PMDPeriphWrite(hPeriph, &cmd, CommandPortOffset, 1);

        // check for success/failure of this command
        result = PMDPAR_GetCommandStatus(transport_data);

        if (result == PMD_ERR_OK)
        {
            // send the data one word at a time
            if (datawordcount > 0)
                PMDPeriphWrite(hPeriph, &xDat[1], DataPortOffset, datawordcount);

            result = PMDPAR_GetCommandStatus(transport_data);

            if (result == PMD_ERR_OK)
            {
                // retrieve any expected data
                if (rCt > 0)
                    PMDPeriphRead(hPeriph, rDat, DataPortOffset, rCt);

                if (bVerifyChecksum)
                {
                    result = PMDPeriphRead(hPeriph, &cksum, DataPortOffset, 1);

                    calcSum = 0;

                    for( i=0; i<xCt; i++ ) calcSum += xDat[i];
                    for( i=0; i<rCt; i++ ) calcSum += rDat[i];

                    if( calcSum != cksum )
                    {
                        PMDprintf( "Checksum error, sent: ");
                        for( i=0; i<xCt; i++ ) PMDprintf( " 0x%04hx", xDat[i] );
                        PMDprintf( "\n" );

                        if( rCt ) PMDprintf( "            received:" );
                        for( i=0; i<rCt; i++ ) PMDprintf( " 0x%04hx", rDat[i] );

                        PMDprintf( "\nExpected: 0x%04hx, got: 0x%04hx\n", calcSum, cksum );

                        result = PMD_ERR_Checksum;
                    }
                }
            }
        }
    }

    PMDMutexUnLock(xMutexPAR);

    return result;
}

//********************************************************
// wait for the device to be ready for the next command
static PMDresult PMDPAR_GetCommandStatus(void* transport_data)
{
    PMDDeviceHandle* hDevice = (PMDDeviceHandle*)transport_data;
    PMDPeriphHandle* hPeriph = hDevice->hPeriph;
    PMDuint16 status;
    PMDresult result = PMD_ERR_OK;

    if ( PMDPAR_WaitUntilReady(transport_data) )
        result = PMD_ERR_Timeout;
    PMDPeriphRead(hPeriph, &status, CommandPortOffset, 1);

    if (status & ERR_MASK)
    {
        if (bDiagnostics)
        {
            result = PMDPAR_HandleError(transport_data);
            //PMDprintf("Command Error: %s\n", PMDGetErrorMessage(result));
        }
    }

    return result;
}


//*****************************************************************************
PMDresult PMDPAR_WaitUntilReady(void* transport_data)
{
    PMDDeviceHandle* hDevice = (PMDDeviceHandle*)transport_data;
    PMDPeriphHandle* hPeriph = hDevice->hPeriph;
    PMDresult result = PMD_NOERROR;
    DWORD stopTime, currentTime;
    PMDuint16 status=0;

    stopTime = PMDDeviceGetTickCount() + CommandTimeoutms;

    for(;;)
    {
        // poll ready port, if not ready, loop
        result = PMDPeriphRead(hPeriph, &status, CommandPortOffset, 1);

        if (status & RDY_MASK)
            return PMD_NOERROR;

        currentTime = PMDDeviceGetTickCount();

        if (currentTime > stopTime)
            return PMD_ERR_Timeout;
    }

} /* end WaitUntilReady */

//*****************************************************************************
PMDresult PMDPAR_HandleError(void* transport_data)
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
    xDat[0] = 0xA5; //PMDOPGetInstructionError;
    rCt = 1;
    rDat[0] = PMD_NOERROR;

    result = PMDPAR_Send(transport_data, xCt, xDat, rCt, rDat);

    if (result == PMD_NOERROR)
    {
        result = (PMDresult)rDat[0];
    }
    GettingError = 0;

    return result;
}

//********************************************************
PMDresult PMDPAR_Init()
{
    PMDMutexCreate(xMutexPAR);

    return PMD_NOERROR;
}

