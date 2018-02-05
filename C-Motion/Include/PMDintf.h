#ifndef PMD_Interface
#define	PMD_Interface


typedef enum {
	InterfaceNone,
	InterfaceParallel,
	InterfacePCI,
	InterfaceISA,
	InterfaceSerial,
	InterfaceCAN,
	InterfaceTCP,
	InterfaceUDP,
} PMDInterfaceType;

#define CAN_ADDRESS_BASE_RX         0x580
#define CAN_ADDRESS_BASE_TX         0x600
#define CAN_ADDRESS_BASE_INTR       0x180


#endif //PMD_Interface

