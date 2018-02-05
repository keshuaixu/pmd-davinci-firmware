//
// GenericUserPacket.c :  

// This example code runs on a host connected to a CME device. There are two methods for sending/receiving
// user packets.  The CME user packet method transmits the user packet via a PMD Resource Protocol 
// connection.  The other method sends a raw packet over it's own interface connection, .
// The advantage to this method is that it does not include the protocol overhead of the CME method.
// This example will open a port and send 10 bytes of data and wait to receive any data 
// from the corresponding example running on the CME device.

// TO DO: Select the interface type you are using by uncommenting the appropriate PMDPeriphOpen??? function.

#include "c-motion.h"
#include "PMDdiag.h"
#include "PMDperiph.h"
#include "PMDsys.h"

#define BUFSIZE   20 
#define MAJOR_VERSION 1
#define MINOR_VERSION 0


USER_CODE_VERSION(MAJOR_VERSION,MINOR_VERSION)

USER_CODE_TASK(GenericUserPacket)
{
	PMDPeriphHandle hPeriph;
	PMDuint32 vmajor, vminor; 
	PMDuint8 data[BUFSIZE]; 	  
	PMDuint32 bytesReceived,i;
	PMDresult result;
	PMDDeviceHandle hDeviceCME;
	PMDPeriphHandle hPeriphCME;
	
	PMDprintf("\n\n*** Begin GenericUserPacket.c ***\n");
	PMDGetCMotionVersion( &vmajor, &vminor );
	PMDprintf("C-Motion Version %d.%d \n", vmajor, vminor);

	PMDprintf("Attempting to open the communications port\n");

	// define CMEUSERPACKET to send user packets via the PMD Resource Protocol
#define CMEUSERPACKET
#ifdef CMEUSERPACKET
	// TODO: select the interface to send user packets via the PMD Resource Protocol
	PMD_ABORTONERROR(PMDPeriphOpenTCP( &hPeriphCME, NULL, PMD_IP4_ADDR(192,168,2,2), DEFAULT_ETHERNET_PORT, 0 ))
//	PMD_ABORTONERROR(PMDPeriphOpenCAN( &hPeriphCME, NULL, 0x580, 0x600, 0 )) 
//	PMD_ABORTONERROR(PMDPeriphOpenCOM( &hPeriphCME, NULL, PMDSerialPort1, PMDSerialBaud57600, PMDSerialParityNone, PMDSerialStopBits1 ))
//	PMD_ABORTONERROR(PMDPeriphOpenPCI( &hPeriphCME, 0 ))
	PMD_RESULT(PMDRPDeviceOpen(&hDeviceCME, &hPeriphCME))  // Open a handle to a Prodigy/CME device 
	PMD_RESULT(PMDDeviceGetVersion(&hDeviceCME,&vmajor,&vminor))
	PMD_RESULT(PMDDeviceGetVersion(&hDeviceCME,&vmajor,&vminor)) // call it twice in case first call returned PMD_ERR_RP_Reset
	PMDprintf("CMEDevice version %d.%d\n",vmajor,vminor);
	PMD_RESULT(PMDPeriphOpenCME(&hPeriph, &hDeviceCME)) // open the user packet periph handle
#else
	// TODO: select the interface to send raw user packets over 
	PMD_ABORTONERROR(PMDPeriphOpenTCP( &hPeriph, NULL, PMD_IP4_ADDR(192,168,2,2), 1234, 0 ))
//	PMD_ABORTONERROR(PMDPeriphOpenCAN( &hPeriph, NULL, 0x581, 0x601, 0 )) 
//	PMD_ABORTONERROR(PMDPeriphOpenCOM( &hPeriph, NULL, PMDSerialPort1, PMDSerialBaud57600, PMDSerialParityNone, PMDSerialStopBits1 ))
#endif

 
	data[0]=0x0A;
	data[1]=0x0B;
	data[2]=0x0C;
	data[3]=0x0D;
	data[4]=0x0E;
	data[5]=0x0F;
	data[6]=0x1A;
	data[7]=0x2A;
	data[8]=0x3A;
	data[9]=0x4A;

	PMDprintf("Sending data\n");
	PMD_RESULT(PMDPeriphSend(&hPeriph,&data,10,5000))
	PMDprintf("Receiving data\n");
	PMD_RESULT(PMDPeriphReceive(&hPeriph, &data, &bytesReceived, 10, 5000))

	if (result == PMD_ERR_OK) 
	{
		PMDprintf("New data received, number of bytes=%d\n",bytesReceived);
		for(i=0;i<bytesReceived;i++)
		{
			PMDprintf("Data=%x\n",data[i]);
		}
	}

	PMDprintf("Done\n");

	PMDPeriphClose(&hPeriph);

#ifdef CMEUSERPACKET
	PMDDeviceClose(&hDeviceCME);
	PMDPeriphClose(&hPeriphCME);
#endif

	PMDTaskWait(1000);
	PMDTaskAbort(0);
}

