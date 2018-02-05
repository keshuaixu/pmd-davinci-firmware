//
//  PMDPSer.c -- Windows serial interface peripheral functions
//
//  Performance Motion Devices, Inc.
//

#include <stdio.h>
#include <stdlib.h>
#include <windows.h>

#include "PMDtypes.h"
#include "PMDecode.h"
#include "PMDperiph.h"
#include "PMDPfunc.h"

// only need to include this if diagnostics mode is used
#include "PMDdiag.h"

static long baudList[] = { 1200, 2400, 9600, 19200, 57600, 115200, 230400, 460800 };
static BYTE  parList[] = { NOPARITY, ODDPARITY, EVENPARITY, MARKPARITY, SPACEPARITY };
static BYTE stopList[] = { ONESTOPBIT, TWOSTOPBITS };


//********************************************************
static PMDresult PMDPCOM_Close(PMDPeriphHandle* hPeriph)
{
    if ( hPeriph->handle != INVALID_HANDLE_VALUE )
    {
        CloseHandle( hPeriph->handle );
        hPeriph->handle = INVALID_HANDLE_VALUE;
    }
    return PMD_ERR_OK;
}

//********************************************************
PMDresult PMDPCOM_SetConfig(PMDPeriphHandle* hPeriph, PMDSerialBaud baud, PMDSerialParity parity, PMDSerialStopBits stopbits)
{
    DCB dcb;

    PMD_PERIPHCONNECTED(hPeriph)

    dcb.DCBlength         = sizeof(DCB);
    dcb.BaudRate          = baudList[baud];
    dcb.fBinary           = 1;
    dcb.fParity           = TRUE;
    dcb.fOutxCtsFlow      = 0;
    dcb.fOutxDsrFlow      = 0;
    dcb.fDtrControl       = DTR_CONTROL_DISABLE;
    dcb.fDsrSensitivity   = 0;
    dcb.fTXContinueOnXoff = 0;
    dcb.fOutX             = 0;
    dcb.fInX              = 0;
    dcb.fErrorChar        = 0;
    dcb.fNull             = 0;
    dcb.fRtsControl       = RTS_CONTROL_DISABLE;
    dcb.fAbortOnError     = 0;
    dcb.XonLim            = 0;
    dcb.XoffLim           = 0;
    dcb.ByteSize          = 8;
    dcb.Parity            = parList[parity];
    dcb.StopBits          = stopList[stopbits];
    dcb.XonChar           = 0;
    dcb.XoffChar          = 0;
    dcb.ErrorChar         = 0;
    dcb.EofChar           = 0;
    dcb.EvtChar           = 0;

    if( !SetCommState( hPeriph->handle, &dcb ) )
        return PMD_ERR_ConfiguringPort;
    return PMD_ERR_OK;
}

//********************************************************
static BOOL PMDPCOM_SetTimeout(PMDPeriphHandle* hPeriph, long msec)
{
    COMMTIMEOUTS timeouts;

    PMD_PERIPHCONNECTED(hPeriph)

    timeouts.ReadIntervalTimeout         = 0;
    timeouts.ReadTotalTimeoutMultiplier  = 10;
    timeouts.ReadTotalTimeoutConstant    = msec;
    timeouts.WriteTotalTimeoutMultiplier = 0;
    timeouts.WriteTotalTimeoutConstant   = 0;

    return SetCommTimeouts( hPeriph->handle, &timeouts );
}


//********************************************************
static PMDresult PMDPCOM_Send(PMDPeriphHandle* hPeriph, void *data, PMDparam nCount, PMDparam timeoutms)
{
    PMDparam nCountSent = 0;
    PMD_PERIPHCONNECTED(hPeriph)

    if (nCount > 0)
    {
        PMDPCOM_SetTimeout(hPeriph, timeoutms);
        if( !WriteFile( hPeriph->handle, data, nCount, &nCountSent, NULL ) )
            return PMD_ERR_PortWrite;
        if (nCountSent < nCount)
            return PMD_ERR_PortWrite;
    }
    return PMD_ERR_OK;
}

//********************************************************
static PMDresult PMDPCOM_Receive(PMDPeriphHandle* hPeriph, void *data, PMDparam nCount, PMDparam *pnreceived, PMDparam timeoutms)
{
    *pnreceived = 0;
    PMD_PERIPHCONNECTED(hPeriph)

    if (nCount > 0)
    {
        PMDPCOM_SetTimeout(hPeriph, timeoutms);
        if( !ReadFile( hPeriph->handle, data, nCount, pnreceived, NULL ) )
            return PMD_ERR_PortRead;
        if (*pnreceived < nCount)
            return PMD_ERR_Timeout;
    }
    return PMD_ERR_OK;
}

//********************************************************
static PMDuint16 PMDPCOM_FlushRecv(PMDPeriphHandle* hPeriph)
{
    PMD_PERIPHCONNECTED(hPeriph)

    PurgeComm(hPeriph->handle, PURGE_RXCLEAR);

    return PMD_ERR_OK;
}

//********************************************************
static void PMDPCOM_Init(PMDPeriphHandle* hPeriph)
{
    // set the interface type 
    hPeriph->type = InterfaceSerial;

    hPeriph->transport.Close    = PMDPCOM_Close;
    hPeriph->transport.Send     = PMDPCOM_Send;
    hPeriph->transport.Receive  = PMDPCOM_Receive;
    hPeriph->transport.Read     = NULL;
    hPeriph->transport.Write    = NULL;
}

//********************************************************
//Set portnum as PMDSerialPort  (0 = COM1, 1 = COM2, etc.)
//********************************************************
PMDresult PMDPCOM_Open(PMDPeriphHandle* hPeriph, PMDparam portnum, PMDSerialBaud baud, PMDSerialParity parity, PMDSerialStopBits stopbits)
{
    char szPort[12];

    PMDPCOM_Init(hPeriph);

    portnum++;
    sprintf( szPort, "\\\\.\\COM%d", portnum );

    hPeriph->handle = CreateFile( szPort, GENERIC_READ|GENERIC_WRITE, 0, 
        NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL );

    if( hPeriph->handle == INVALID_HANDLE_VALUE )
    {
        return PMD_ERR_OpeningPort;
    }
    if (PMD_ERR_OK != PMDPCOM_SetConfig(hPeriph, baud, parity, stopbits))
    {
        PMDPCOM_Close(hPeriph);
        return PMD_ERR_ConfiguringPort;
    }
    if (!PMDPCOM_SetTimeout(hPeriph, 100))
    {
        PMDPCOM_Close(hPeriph);
        return PMD_ERR_ConfiguringPort;
    }
	return PMD_ERR_OK;
}