// PMD Prodigy/CME example


#include "c-motion.h"
#include "PMDperiph.h"
#include "PMDsys.h"
#include "PMDdiag.h"
#include "PMDMB.h"
#include "examples.h"

#define MAJOR_VERSION 1
#define MINOR_VERSION 0

USER_CODE_VERSION(MAJOR_VERSION,MINOR_VERSION)

PMDresult result; // global result value used by PMD_RESULT macro

// forward references
void IOExample(PMDAxisHandle* phAxis);
void SetupAxis1(PMDAxisHandle* phAxis1); // in Pro-MotionExport.c

USER_CODE_TASK(IONCME)
{
	PMDDeviceHandle *phDevice = NULL;
	PMDAxisHandle hAxis;
	PMDuint32 vmajor; 
	PMDuint32 vminor;
	
#ifdef CME
	phDevice = NULL;
#else
	// if this is a host computer open the local peripheral that is connected to the Prodigy/CME board
	PMDPeriphHandle hPeriph;
	PMDDeviceHandle hDevice;
	PMDprintf("Attempting to open the communications port\n");
	// TODO: select one of the following interfaces
	PMD_ABORTONERROR(PMDPeriphOpenTCP( &hPeriph, NULL, PMD_IP4_ADDR(192,168,2,2), DEFAULT_ETHERNET_PORT, 0 ))
//	PMD_ABORTONERROR(PMDPeriphOpenCOM( &hPeriph, NULL, PMDSerialPort1, PMDSerialBaud57600, PMDSerialParityNone, PMDSerialStopBits1);
//	PMD_ABORTONERROR(PMDPeriphOpenCAN( &hPeriph, NULL, 0x580, 0x600, 0 )) 
	PMD_ABORTONERROR(PMDRPDeviceOpen(&hDevice, &hPeriph))	// Open a handle to a Magellan MP device
	phDevice = &hDevice;
#endif

	PMDprintf("Prodigy/CME example\n");

	// this may return PMD_ERR_RP_Reset so do not abort.
	PMD_RESULT(PMDDeviceGetVersion(phDevice, &vmajor, &vminor))
	// should not return PMD_ERR_RP_Reset a second time.
	PMD_ABORTONERROR(PMDDeviceGetVersion(phDevice, &vmajor, &vminor))
	PMDprintf("CME device version v%d.%d\n", vmajor, vminor);

	PMD_ABORTONERROR(PMDAxisOpen(&hAxis, phDevice, PMDAxis1 ))

	PMDprintf("Calling AllMagellanCommands() from Examples.c\r\n");
	AllMagellanCommands(&hAxis);

//	PMDprintf("Calling SetupAxis1() from Pro-MotionExport.c\r\n");
//	SetupAxis1(&hAxis);

	PMDprintf("Calling ProfileMove() from Examples.c\r\n");
	ProfileMove(&hAxis);

	PMDprintf("Calling MemoryExample() from Examples.c\r\n");
	MemoryExample(phDevice, PMDMemoryType_DPRAM);

	PMDprintf("Calling IOExample() from ProdigyCME.c\r\n");
	IOExample(&hAxis);

	PMDprintf("Done\r\n");

#ifndef CME
	PMD_ABORTONERROR(PMDDeviceClose(&hDevice));
	PMD_ABORTONERROR(PMDPeriphClose(&hPeriph));
#endif

	PMDTaskWait(1000);
	PMDTaskAbort(0);
	
}

void IOExample(PMDAxisHandle* phAxis)
{
PMDuint16 DIOstate;

	PMDprintf("Prodigy specific functions\n");
	PMD_RESULT(PMDMBGetDACOutputEnable(phAxis, &DIOstate));
	PMD_RESULT(PMDMBSetDACOutputEnable(phAxis, DIOstate));
	PMD_RESULT(PMDMBGetAmplifierEnable(phAxis, &DIOstate));
	PMD_RESULT(PMDMBSetAmplifierEnable(phAxis, DIOstate, DIOstate));
	PMD_RESULT(PMDMBReadDigitalInput(phAxis, &DIOstate));
	PMD_RESULT(PMDMBReadDigitalOutput(phAxis, &DIOstate));
	PMD_RESULT(PMDMBWriteDigitalOutput(phAxis, DIOstate));
}

