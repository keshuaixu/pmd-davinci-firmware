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

typedef struct {
	PMDuint32 mode[4];
	PMDint32 current[4];
	PMDint32 velocity[4];
	PMDint32 position[4];
	PMDuint32 temperature[4];
	PMDuint32 analog[4];
	PMDuint32 fault[4];
} DVRKMessageToPC;

typedef struct {
	PMDuint32 command;
	PMDint32 command_payload[4];
	PMDuint32 mode[4];
	PMDint32 motor_command[4];
} DVRKMessageFromPC;


USER_CODE_TASK(GenericUserPacket)
{
	PMDDeviceHandle *phDevice = NULL;
	PMDPeriphHandle hPeriphIO;
	PMDAxisHandle hAxis[4];
	PMDPeriphHandle hPeriph;
	PMDPeriphHandle hPeriph_rx;
	PMDuint32 vmajor, vminor; 
	PMDuint32 bytesReceived;
	PMDresult result;
	
	PMDprintf("\n\n*** Begin GenericUserPacket.c ***\n");
	PMDGetCMotionVersion( &vmajor, &vminor );
	PMDprintf("C-Motion Version %d.%d \n", vmajor, vminor);

	PMD_ABORTONERROR(PMDAxisOpen(&hAxis[0], phDevice, PMDAxis1));
	PMD_ABORTONERROR(PMDAxisOpen(&hAxis[1], phDevice, PMDAxis2));
	PMD_ABORTONERROR(PMDAxisOpen(&hAxis[2], phDevice, PMDAxis3));
	PMD_ABORTONERROR(PMDAxisOpen(&hAxis[3], phDevice, PMDAxis4));
	PMD_ABORTONERROR(PMDPeriphOpenPIO(&hPeriphIO, phDevice, 0, 0, PMDDataSize_16Bit));
	//PMDPeriphOpenTCP(&hPeriph, NULL, PMD_IP4_ADDR(0, 0, 0, 0), 18021, 0);

	DVRKMessageToPC to_pc = { 0 };
	DVRKMessageFromPC from_pc = { 0 };
	PMDuint16 adc_reading[8] = { 0 };
	
	PMDuint32 ip_addr = 0;
	PMDPeriphOpenUDP(&hPeriph, NULL, PMD_IP4_ADDR(192, 168, 1, 255), 18021);
	PMDPeriphOpenUDP(&hPeriph_rx, NULL, PMD_IP4_ADDR(0, 0, 0, 0), 18022);


	while (1) {
		

		PMD_RESULT(PMDPeriphRead(&hPeriphIO, &adc_reading, PMDMachineIO_AICh1, 8))
		to_pc.analog[0] = adc_reading[7];
		to_pc.analog[1] = adc_reading[1];
		to_pc.analog[2] = adc_reading[6];
		to_pc.analog[3] = adc_reading[3];
		
		for (int axis = 0; axis < 4; axis++) {
			PMDuint16 mode;
			PMD_RESULT(PMDGetOperatingMode(&hAxis[axis], &mode))
			to_pc.mode[axis] = mode;

			PMD_RESULT(PMDGetCurrentLoopValue(&hAxis[axis], 0x00, 0x01, &to_pc.current[axis]))

			PMD_RESULT(PMDGetActualVelocity(&hAxis[axis], &to_pc.velocity[axis]))

			PMD_RESULT(PMDGetActualPosition(&hAxis[axis], &to_pc.position[axis]))

			PMDuint16 temperature;
			PMD_RESULT(PMDGetTemperature(&hAxis[axis], &temperature))
			to_pc.temperature[axis] = temperature;

			PMDuint16 fault;
			PMD_RESULT(PMDGetDriveFaultStatus(&hAxis[axis], &fault))
			to_pc.fault[axis] = fault;
		}
		
		PMDresult send_result = PMDPeriphSend(&hPeriph, &to_pc, sizeof(to_pc), 0);
		/*
		if (send_result == PMD_ERR_NotConnected) {
			for (int axis = 0; axis < 4; axis++) {
				// disable motor power when connection lost.
				PMDSetOperatingMode(&hAxis[axis], 0x01);
				PMDUpdate(&hAxis[axis]);
			}	
			PMDPeriphClose(&hPeriph);
			PMDPeriphOpenTCP(&hPeriph, NULL, PMD_IP4_ADDR(0, 0, 0, 0), 18021, 0);
		}
		*/

		
		PMDresult receive_result = PMDPeriphReceive(&hPeriph_rx, &from_pc, &bytesReceived, sizeof(from_pc), 0);
		/*

		if (receive_result == PMD_ERR_NotConnected) {
			PMDPeriphClose(&hPeriph);
			PMDPeriphOpenTCP(&hPeriph, NULL, PMD_IP4_ADDR(0, 0, 0, 0), 18021, 0);
		}
		*/

		if (receive_result == PMD_ERR_OK)
		{	
			PMDprintf("received %d\n", bytesReceived);
			for (int axis = 0; axis < 4; axis++) {
				PMDuint16 mode = (PMDuint16)from_pc.mode[axis];
				if (mode <= 0x07) {
					PMD_RESULT(PMDSetOperatingMode(&hAxis[axis], mode))
					PMD_RESULT(PMDSetMotorCommand(&hAxis[axis], (PMDint16)from_pc.motor_command[axis]))
				}
				else {
					//set the velocity/position profile
				}
				PMD_RESULT(PMDUpdate(&hAxis[axis]))
			}
		}

	}
}

