// PMD Machine Controller example

#include "c-motion.h"
#include "PMDdevice.h"
#include "PMDperiph.h"
#include "PMDsys.h"
#include "PMDdiag.h"
#include "PMDutil.h"
#include "examples.h"

#define MAJOR_VERSION 1
#define MINOR_VERSION 1


USER_CODE_VERSION(MAJOR_VERSION,MINOR_VERSION)

PMDresult result; // global result value used by PMD_RESULT macro

// forward references
void ConnectToDevice(PMDDeviceHandle* phDevice, int portoffset);
void IOExample(PMDDeviceHandle* phDevice);
void RPDeviceExamples(PMDDeviceHandle* phDevice);
void MagellanExamples(PMDDeviceHandle* phDevice);
  // in Pro-MotionExport.c
void SetupAxis1(PMDAxisHandle* phAxis1);
void SetupAxis2(PMDAxisHandle* phAxis1);
void SetupAxis3(PMDAxisHandle* phAxis1);
void SetupAxis4(PMDAxisHandle* phAxis1);

USER_CODE_TASK(MachineController)
{
	PMDDeviceHandle *phDevice = NULL;

	PMDprintf("Machine Controller example\n");

#ifdef CME
    // If this code is running on the CME we can just use a NULL device handle to address the local device
	RPDeviceExamples(phDevice);
	MagellanExamples(phDevice);
#else
    // If this code is running on a host then we first need to get a device handle to address the device.
	ConnectToDevice(phDevice, 0);
#endif
	// connect to another CME device if available
	ConnectToDevice(phDevice, 1);

	PMDprintf("Done\r\n");
	PMDTaskAbort(0);
}

// portoffset is the offset from the default port number (TCP:192.168.2.2 CAN:0x600, 0x580 Serial:None)
void ConnectToDevice(PMDDeviceHandle* phDevice, int portoffset)
{
	PMDPeriphHandle hPeriph;
	PMDDeviceHandle hDevice;
	PMDInterfaceType Interface = InterfaceTCP; //InterfaceSerial, InterfaceCAN, InterfaceTCP

	PMD_INITIALIZEHANDLE(hPeriph);
	PMD_INITIALIZEHANDLE(hDevice);
	PMDprintf("\nOpening interface from MC: ");

	switch (Interface) {
	case InterfaceTCP:
		PMDprintf("Opening 192.168.2.%d\n", 2 + portoffset);
		PMD_RESULT(PMDPeriphOpenTCP( &hPeriph, phDevice, PMD_IP4_ADDR(192,168,2,2 + portoffset), DEFAULT_ETHERNET_PORT, 0 ))
		break;
	case InterfaceCAN:
		PMDprintf("Opening CAN port\n");
		PMD_RESULT(PMDPeriphOpenCAN( &hPeriph, phDevice, CAN_ADDRESS_BASE_TX + portoffset, CAN_ADDRESS_BASE_RX + portoffset, 0 )) 
		break;
	case InterfaceSerial:
		PMDprintf("Opening COM port 1\n");
		PMD_RESULT(PMDPeriphOpenCOM( &hPeriph, phDevice, PMDSerialPort1, PMDSerialBaud57600, PMDSerialParityNone, PMDSerialStopBits1))
		break;
	default:
		PMDprintf("Invalid interface specified\n");
		return;
	};
	if (result == PMD_ERR_OK)
	{
		PMDprintf("RPDeviceOpen to remote MC\n");
		PMD_RESULT(PMDRPDeviceOpen(&hDevice, &hPeriph))	// Open a handle to a PRP device
		phDevice = &hDevice;

		// This may return PMD_ERR_RP_Reset so call it twice.
		PMD_RESULT(PMDDeviceNoOperation(phDevice))
		PMD_RESULT(PMDDeviceNoOperation(phDevice))

		if (result == PMD_ERR_OK)
		{
			// We have our handles now call some functions
			RPDeviceExamples(phDevice);
			MagellanExamples(phDevice);

		}
		PMD_RESULT(PMDDeviceClose(&hDevice));
	}
	PMD_RESULT(PMDPeriphClose(&hPeriph));
}

void RPDeviceExamples(PMDDeviceHandle* phDevice)
{
	PMDuint32 vmajor, vminor;

	// This may return PMD_ERR_RP_Reset.
	PMD_RESULT(PMDDeviceNoOperation(phDevice))
	PMDTaskWait(1);
	// Subsequent commands should not return PMD_ERR_RP_Reset.
	PMD_RESULT(PMDDeviceGetVersion(phDevice, &vmajor, &vminor))
	PMDprintf("CME device version v%d.%d\n", vmajor, vminor);

	PMDprintf("Calling MemoryExample() from Examples.c\r\n");
	MemoryExample(phDevice, PMDMemoryType_DPRAM);

	PMDprintf("Calling IOExample() from MachineController.c\r\n");
	IOExample(phDevice);

	PMDprintf("Calling DeviceFunctions() from Examples.c\r\n");
	DeviceFunctions(phDevice);
}


void IOExample(PMDDeviceHandle* phDevice)
{
	PMDPeriphHandle hPeriphIO;
	BYTE eventIRQ = 0;
	PMDDataSize datasize = PMDDataSize_16Bit;
	PMDuint16 data;


	// Open a Peripheral IO handle to addresses 0 and above
	PMD_RESULT(PMDPeriphOpenPIO(&hPeriphIO, phDevice, 0, eventIRQ, datasize));
	if (result == PMD_ERR_OK)
	{
		// Read digital inputs
		PMD_RESULT(PMDPeriphRead(&hPeriphIO, &data, PMDMachineIO_DI, 1));
		PMDprintf("DI read %04X\r\n", data);

		// Read digital IO
		PMD_RESULT(PMDPeriphRead(&hPeriphIO, &data, PMDMachineIO_DORead, 1));
		PMDprintf("DIO read %04X\r\n", data);

		// Read digital IO direction register
		PMD_RESULT(PMDPeriphRead(&hPeriphIO, &data, PMDMachineIO_DODirRead, 1));
		PMDprintf("DIO Direction read %04X\r\n", data);

		// Read amp enable output 
		PMD_RESULT(PMDPeriphRead(&hPeriphIO, &data, PMDMachineIO_AmpEnaRead, 1));
		PMDprintf("AmpEnable read %04X\r\n", data);

		// Enable analog outputs
		data = 1;
		PMD_RESULT(PMDPeriphWrite(&hPeriphIO, &data, PMDMachineIO_AOEna, 1));
		PMDprintf("DAC enable write %04X\r\n", data);

		// Set channel 1 analog output source to CME (default is Magellan)
		data = 1;
		PMD_RESULT(PMDPeriphWrite(&hPeriphIO, &data, PMDMachineIO_AOCh1Ena, 1));
		PMDprintf("AO source write %04X\r\n", data);

		// Read channel 1 analog input at PIO offset of PMDMachineIO_AI
		// analog input data is a 16-bit signed integer 0x8000 = -10V 0x7FFF = +10V 
		PMD_RESULT(PMDPeriphRead(&hPeriphIO, &data, PMDMachineIO_AICh1, 1));
		PMDprintf("AI read %04X\r\n", data);
		
		// Write channel 1 analog output value
		data = 0x800;
		PMD_RESULT(PMDPeriphWrite(&hPeriphIO, &data, PMDMachineIO_AOCh1, 1));
		PMDprintf("AO write %04X\r\n", data);
		
		PMD_RESULT(PMDPeriphClose(&hPeriphIO));
	}
}


void MagellanExamples(PMDDeviceHandle* phDevice)
{
	PMDAxisHandle hAxis1;
	PMDAxisHandle hAxis2;
	PMDAxisHandle hAxis3;
	PMDAxisHandle hAxis4;
	PMDuint32 version;
	PMDint32 position;

	PMD_RESULT(PMDAxisOpen(&hAxis1, phDevice, PMDAxis1 ))
	PMD_RESULT(PMDAxisOpen(&hAxis2, phDevice, PMDAxis2 ))
	PMD_RESULT(PMDAxisOpen(&hAxis3, phDevice, PMDAxis3 ))
	PMD_RESULT(PMDAxisOpen(&hAxis4, phDevice, PMDAxis4 ))

	PMD_RESULT(PMDGetVersion32(&hAxis1, &version))
	PMDprintf("Magellan version %8lX\n", version);

	PMD_RESULT(PMDGetPosition(&hAxis1, &position ))
	PMDprintf("GetPosition %ld\n", position);

	// Generate Pro-MotionExport.c from the Pro-Motion GUI before calling SetupAxis().
	// Select "File/Export as C-Motion" in Pro-Motion.
	PMDprintf("Calling SetupAxis1() from Pro-MotionExport.c\r\n");
	SetupAxis1(&hAxis1);
	SetupAxis2(&hAxis2);
	SetupAxis3(&hAxis3);
	SetupAxis4(&hAxis4);

//	PMDprintf("Calling AtlasCommands() from Examples.c\r\n");
//	AtlasCommands(&hAxis1);

	PMDprintf("Calling ProfileMove() from Examples.c\r\n");
	ProfileMove(&hAxis1);

	PMD_RESULT(PMDAxisClose(&hAxis1));
	PMD_RESULT(PMDAxisClose(&hAxis2));
	PMD_RESULT(PMDAxisClose(&hAxis3));
	PMD_RESULT(PMDAxisClose(&hAxis4));
}
