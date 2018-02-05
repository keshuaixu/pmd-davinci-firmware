// IONCME example


#include "c-motion.h"
#include "PMDperiph.h"
#include "PMDsys.h"
#include "PMDdiag.h"
#include "examples.h"

#define MAJOR_VERSION 1
#define MINOR_VERSION 0

// for compatibility with the manual
#define PMD_ADDRESS_ION_DI	PMDIndexerIO_DI
#define PMD_ADDRESS_ION_DO	PMDIndexerIO_DO
#define PMD_ADDRESS_ION_AI	PMDIndexerIO_AI

USER_CODE_VERSION(MAJOR_VERSION,MINOR_VERSION)

PMDresult result; // global result value used by PMD_RESULT macro

// forward references
void IOExample(PMDDeviceHandle* phDevice);
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
	// if this is a host computer open the local peripheral that is connected to the ION
	PMDPeriphHandle hPeriph;
	PMDDeviceHandle hDevice;

	// TODO: select one of the following interfaces to connect to the device
	PMD_ABORTONERROR(PMDPeriphOpenTCP( &hPeriph, NULL, PMD_IP4_ADDR(192,168,2,2), DEFAULT_ETHERNET_PORT, 0 ))
//	PMD_ABORTONERROR(PMDPeriphOpenCOM( &hPeriph, NULL, PMDSerialPort1, PMDSerialBaud57600, PMDSerialParityNone, PMDSerialStopBits1);

	PMD_ABORTONERROR(PMDRPDeviceOpen(&hDevice, &hPeriph))	// Open a handle to a Magellan MP device
	phDevice = &hDevice;
#endif

	PMDprintf("ION/CME example\n");

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

	PMDprintf("Calling IOExample() from IONCME.c\r\n");
	IOExample(phDevice);

	PMDprintf("Done\r\n");

#ifndef CME
	PMD_ABORTONERROR(PMDDeviceClose(&hDevice));
	PMD_ABORTONERROR(PMDPeriphClose(&hPeriph));
#endif

	PMDTaskWait(1000);
	PMDTaskAbort(0);
	
}


void IOExample(PMDDeviceHandle* phDevice)
{
	PMDPeriphHandle hPeriph;
	BYTE eventIRQ = 0;
	PMDDataSize datasize = PMDDataSize_16Bit;
	PMDparam offset = 0;
	PMDparam length = 1;
	PMDuint16 data;
	
	PMDprintf("ION/CME analog and digital IO example\n");

	// Read digital inputs
	PMD_ABORTONERROR(PMDPeriphOpenPIO(&hPeriph, phDevice, PMD_ADDRESS_ION_DI, eventIRQ, datasize));
	PMD_ABORTONERROR(PMDPeriphRead(&hPeriph, &data, offset, length));
	
	PMDprintf("DI read %04X\r\n", data);

	PMD_ABORTONERROR(PMDPeriphClose(&hPeriph));

	// Read digital outputs
	PMD_ABORTONERROR(PMDPeriphOpenPIO(&hPeriph, phDevice, PMD_ADDRESS_ION_DO, eventIRQ, datasize));
	PMD_ABORTONERROR(PMDPeriphRead(&hPeriph, &data, offset, length));

	PMDprintf("DO read %04X\r\n", data);

	PMD_ABORTONERROR(PMDPeriphClose(&hPeriph));

	// Read analog input
	PMD_ABORTONERROR(PMDPeriphOpenPIO(&hPeriph, phDevice, PMD_ADDRESS_ION_AI, eventIRQ, datasize));
	PMD_ABORTONERROR(PMDPeriphRead(&hPeriph, &data, offset, length));

	// Read AI returns 0x0000 = -10V 0xFFFF,= +10V 
	PMDprintf("AI read %04X\r\n", data);

	PMD_ABORTONERROR(PMDPeriphClose(&hPeriph));

}


