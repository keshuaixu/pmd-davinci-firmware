//
// GenericUserPacket.c :  

// This example code runs on a CME Device connected to a host. There are two methods for sending/receiving
// user packets.  The CME user packet method transmits the user packet via a PMD Resource Protocol 
// connection.  The other method sends a raw packet over it's own interface connection, .
// The advantage to this method is that it does not include the protocol overhead of the CME method.
// This example will open a port and wait to receive any data from the host and send it back.

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
	
	PMDprintf("\n\n*** Begin GenericUserPacket.c ***\n");
	PMDGetCMotionVersion( &vmajor, &vminor );
	PMDprintf("C-Motion Version %d.%d \n", vmajor, vminor);

	// TODO: select the desired interface 
	PMDprintf("Attempting to open the communications port\n");

	// TODO: select the interface to receive user packets
	PMD_ABORTONERROR(PMDPeriphOpenCME( &hPeriph, NULL)) // select this one to receive user packets via the PMD Resource Protocol
//	PMD_ABORTONERROR(PMDPeriphOpenTCP( &hPeriph, NULL, PMD_IP4_ADDR(0,0,0,0), 1234, 0 )) // listen for a TCP connection on port 1234
//	PMD_ABORTONERROR(PMDPeriphOpenCAN( &hPeriph, NULL, 0x601, 0x581, 0 )) 
//	PMD_ABORTONERROR(PMDPeriphOpenCOM( &hPeriph, NULL, PMDSerialPort1, PMDSerialBaud57600, PMDSerialParityNone, PMDSerialStopBits1 ))

	memset(data, 0, BUFSIZE); 

	PMD_RESULT(PMDPeriphReceive(&hPeriph, &data, &bytesReceived, 10, 5000))

	if (result == PMD_ERR_OK) 
	{
		PMDprintf("New data received, number of bytes=%d\n", bytesReceived);
		for(i=0; i<bytesReceived; i++)
		{
			PMDprintf("Data=%x\n", data[i]);
		}
		PMDprintf("Sending the received data\n");
		PMD_RESULT(PMDPeriphSend(&hPeriph, &data, bytesReceived, 5000))
	}

	PMDTaskWait(3000);
	PMDprintf("Done\n");
	PMDPeriphClose(&hPeriph);

	PMDTaskAbort(0);
}

