//
//  "Base Class" implementation for PMD Peripherals
//
//  Performance Motion Devices, Inc.
//

#include "PMDperiph.h"
#include "PMDsys.h"
#include "c-motion.h"

//********************************************************
PMDCFunc PMDPeriphReceiveEvent(PMDPeriphHandle* hPeriph, void *pData, PMDparam *pnReceived, PMDparam nExpected, PMDparam timeout)
{
    PMD_PERIPHCONNECTED(hPeriph)
    *pnReceived = 0;
    if (hPeriph->transport.ReceiveEvent == NULL)
        return PMD_ERR_NotSupported;

    return hPeriph->transport.ReceiveEvent(hPeriph, pData, nExpected, pnReceived, timeout);
}

//********************************************************
PMDCFunc PMDPeriphReceive(PMDPeriphHandle* hPeriph, void *pData, PMDparam *pnReceived, PMDparam nExpected, PMDparam timeout)
{
    PMD_PERIPHCONNECTED(hPeriph)
    *pnReceived = 0;
    if (hPeriph->transport.Receive == NULL)
        return PMD_ERR_NotSupported;
    if (nExpected == 0)
        return PMD_ERR_OK;

    return hPeriph->transport.Receive(hPeriph, pData, nExpected, pnReceived, timeout);
}

//********************************************************
PMDCFunc PMDPeriphSend(PMDPeriphHandle* hPeriph, void *pData, PMDparam nCount, PMDparam timeout)
{
    PMD_PERIPHCONNECTED(hPeriph)
    if (hPeriph->transport.Send == NULL)
        return PMD_ERR_NotSupported;
    if (nCount == 0)
        return PMD_ERR_OK;

    return hPeriph->transport.Send(hPeriph, pData, nCount, timeout);
}

//********************************************************
PMDCFunc PMDPeriphWrite(PMDPeriphHandle* hPeriph, void *pData, PMDparam offset, PMDparam length)
{
    PMD_PERIPHCONNECTED(hPeriph)
    if (hPeriph->transport.Write == NULL)
        return PMD_ERR_NotSupported;

    return hPeriph->transport.Write(hPeriph, pData, offset, length);
}

//********************************************************
PMDCFunc PMDPeriphRead(PMDPeriphHandle* hPeriph, void *pData, PMDparam offset, PMDparam length)
{
    PMD_PERIPHCONNECTED(hPeriph)
    if (hPeriph->transport.Read == NULL)
        return PMD_ERR_NotSupported;

    return hPeriph->transport.Read(hPeriph, pData, offset, length);
}

//********************************************************
void PMDPeriphOut(PMDPeriphHandle* hPeriph, PMDparam offset, PMDparam data)
{
    PMDPeriphWrite(hPeriph, &data, offset, 1);
}

//********************************************************
int PMDPeriphIn(PMDPeriphHandle* hPeriph, PMDparam offset)
{
    PMDparam data = 0;
    PMDPeriphRead(hPeriph, &data, offset, 1);

    return (int)data;
}

//********************************************************
PMDCFunc PMDPeriphClose(PMDPeriphHandle* hPeriph)
{
    PMDresult result = PMD_ERR_OK;

    PMD_PERIPHCONNECTED(hPeriph)
    if (hPeriph->transport.Close != NULL)
        result = hPeriph->transport.Close(hPeriph);
    PMDZeroMemory( hPeriph, sizeof(PMDPeriphHandle));

    return result;
}

//********************************************************
PMDCFunc PMDPeriphFlush(PMDPeriphHandle* hPeriph)
{
    PMDresult result = PMD_ERR_OK;
    char ch;
    int nCount = 0;
    PMDparam nReceived;

    while( PMD_ERR_OK == PMDPeriphReceive( hPeriph, &ch, &nReceived, 1, 0 ) && nReceived==1)
    {
        nCount++;
        if (nCount > 2)
          return PMD_ERR_UnexpectedDataReceived;
    }

    return result;
}

//********************************************************
// Create an axis handle for communicating directly with an Atlas rather than via the Magellan
// This is required for certain functions such as Atlas trace.
PMDCFunc PMDAtlasAxisOpen(PMDAxisInterface hSourceAxis, PMDAxisInterface hAtlasAxis)
{
    // copy the source axis handle info
    *hAtlasAxis = *hSourceAxis;
    // set the axis number to indicate it is an Atlas axis
    hAtlasAxis->axis = hAtlasAxis->axis | PMDAxisAtlasMask;

    return PMD_ERR_OK;
}

//********************************************************
// PMDAxisClose defined for consistency, but is not required
PMDCFunc PMDAxisClose(PMDAxisHandle *hAxis)
{
    PMDZeroMemory(hAxis, sizeof(PMDAxisHandle));
    return PMD_ERR_OK;
}

//********************************************************
PMDCFunc PMDMemoryClose(PMDMemoryHandle *hMemory)
{
    PMDresult result = PMD_ERR_OK;
    PMD_VERIFYHANDLE(hMemory)
    if (hMemory->transport.Close)
        result = hMemory->transport.Close(hMemory);
    PMDZeroMemory(hMemory, sizeof(PMDMemoryHandle));

    return result;
}

//********************************************************
PMDCFunc PMDDeviceClose(PMDDeviceHandle *hDevice)
{
    PMDresult result = PMD_ERR_OK;
    PMD_VERIFYHANDLE(hDevice)
    if (hDevice->transport.Close)
        result = hDevice->transport.Close(hDevice);
    PMDZeroMemory(hDevice, sizeof(PMDDeviceHandle));

    return result;
}

//********************************************************
PMDCFunc PMDDeviceGetEvent(PMDDeviceHandle* phDevice, PMDEvent* event_data)
{
PMDuint16 event;
PMDAxis axis = 0;
PMDuint16 axismask = 0;
PMDuint16 interruptmask = 0;
PMDAxisHandle hAxis;
PMDresult result;

    event_data->axis = 0;
    event_data->event = 0;
    PMDAxisOpen(&hAxis, phDevice, PMDAxis1);
    // returns mask of all axes that caused the interrupt
    result = PMDGetInterruptAxis(&hAxis, &axismask);
    if (result == PMD_ERR_OK)
    {
        if (axismask == 0)
        {
          PMDClearInterrupt(&hAxis);
          return PMD_ERR_Cancelled; // interrupt was reset in Magellan before WaitForEvent was called
        }
        while ((axismask & 1) != 1 && axis < 4)
        {
          axismask >>= 1;
          axis++;
        }
        hAxis.axis = axis;
        PMDGetInterruptMask(&hAxis, &interruptmask);
        PMDGetEventStatus(&hAxis, &event);
        event_data->axis = axis;
        event_data->event = event;
    }
    return result;
}

//********************************************************
// The default WaitForEvent function for interfaces that do not support interrupts
PMDCFunc PMDWaitForEventPoll(PMDDeviceHandle* phDevice, PMDEvent* event_data, PMDuint32 timeoutms)
{
    PMDuint16 event;
    PMDAxis axis = 0;
    PMDuint16 axismask = 0;
    PMDAxisHandle hAxis;
    PMDresult result;
    PMDuint32 delta, lastTime, currentTime;

    event_data->axis = 0;
    event_data->event = 0;
    PMDAxisOpen(&hAxis, phDevice, PMDAxis1);

    // GetTickCount returns time in ms
    lastTime = PMDDeviceGetTickCount();

    // PMDGetInterruptAxis returns mask of all axes that caused the interrupt
    // but just return the first axis that caused the interrupt this time around.
    do
    {
        result = PMDGetInterruptAxis(&hAxis, &axismask);
        currentTime = PMDDeviceGetTickCount();
        // delta computation should work even if the counter rolls over.
        delta = currentTime - lastTime;
        lastTime = currentTime;
        if (timeoutms <= delta)
            break;
        if (timeoutms < PMD_WAITFOREVER)
            timeoutms -= delta;
    }
    while (result == PMD_ERR_OK && axismask == 0);

    if (result == PMD_ERR_OK)
    {
        if (axismask == 0)
            return PMD_ERR_Timeout;
        // convert axismask to axis number
        while ((axismask & 1) != 1 && axis < 4)
        {
            axismask >>= 1;
            axis++;
        }
        hAxis.axis = axis;
        PMDGetEventStatus(&hAxis, &event);
        result = PMDResetEventStatus(&hAxis, (PMDuint16)~event);
        event_data->axis = axis;
        event_data->event = event;
    }
    return result;
}

//********************************************************
PMDCFunc PMDWaitForEvent(PMDDeviceHandle *hDevice, PMDEvent* event, PMDuint32 timeout)
{
    PMD_VERIFYHANDLE(hDevice)
    if (hDevice->transport.WaitForEvent)
        return hDevice->transport.WaitForEvent(hDevice, event, timeout);
    else
        return PMDWaitForEventPoll(hDevice, event, timeout);
}

//********************************************************
PMDCFunc PMDMemoryRead(PMDMemoryHandle *hMemory, void *data, PMDuint32 offset, PMDuint32 length)
{
    int datasize;
    PMD_VERIFYHANDLE(hMemory)
    datasize = hMemory->datasize - 1;
    if (hMemory->transport.ReadMemory)
    {   // check that the supplied data pointer is correctly aligned
        if (datasize & (PMDuint32)data)
            return PMD_ERR_ParameterAlignment;
        else
            return hMemory->transport.ReadMemory(hMemory, data, offset, length);
    }
    else
        return PMD_ERR_InterfaceNotInitialized;
}

//********************************************************
PMDCFunc PMDMemoryWrite(PMDMemoryHandle *hMemory, void *data, PMDuint32 offset, PMDuint32 length)
{
    int datasize;
    PMD_VERIFYHANDLE(hMemory)
    datasize = hMemory->datasize - 1;
    if (hMemory->transport.WriteMemory)
    {   // check that the supplied data pointer is correctly aligned
        if (datasize & (PMDuint32)data)
            return PMD_ERR_ParameterAlignment;
        else
            return hMemory->transport.WriteMemory(hMemory, data, offset, length);
    }
    else
        return PMD_ERR_InterfaceNotInitialized;
}
