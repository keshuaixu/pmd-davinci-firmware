// ****************************************************************************
//  PMDRPtypes.h -- PMD resource protocol definitions
//
//  Performance Motion Devices, Inc.
//

#ifndef _DEF_INC_PMDRPtypes
#define _DEF_INC_PMDRPtypes

#define TOTAL_PACKET_LENGTH         252
#define MAX_PARAM_DWORDS            4     // number of 32-bit params that can be added to a packet
#define PACKET_HEADER_LENGTH        2
#define PACKET_HEADER_LENGTH_RX     1
#define PACKET_ERROR_LENGTH         3
// max # of words in a packet minus header
#define MAX_PACKET_DATA_LENGTH      (TOTAL_PACKET_LENGTH - 4)
#define MAX_PACKET_DWORDS           (MAX_PACKET_DATA_LENGTH >> 2)
// max # of words in a packet minus header and params
#define MAX_DATA_DWORDS             (MAX_PACKET_DWORDS - MAX_PARAM_DWORDS)
#define USER_PACKET_LENGTH          250

#define DEFAULT_ETHERNET_PORT       40100
#define RP_TIMEOUT                  1000 //ms

#define SUB_ACTION_POS              2       // position of sub command byte in packet header
#define SUB_COMMAND_POS             3       // position of sub command byte in packet header
#define ERROR_CODE_POS              1       // position of error code in response packet
#define PROTOCOL_VERSION            1

#define DEFAULT_ID( bytecount, id )  (((bytecount-1) << 8) | id)

typedef enum PMD_DefaultsEnum {
    PMD_Default_CP_Serial               = DEFAULT_ID( 2,  1 ),
    PMD_Default_CP_MotorType            = DEFAULT_ID( 2,  2 ),
    PMD_Default_IPAddress               = DEFAULT_ID( 4,  3 ),
    PMD_Default_NetMask                 = DEFAULT_ID( 4,  4 ),
    PMD_Default_Gateway                 = DEFAULT_ID( 4,  5 ),
    PMD_Default_IPPort                  = DEFAULT_ID( 2,  6 ),
    PMD_Default_MACH                    = DEFAULT_ID( 4,  7 ),
    PMD_Default_MACL                    = DEFAULT_ID( 4,  8 ),
    PMD_Default_PHYcontrol              = DEFAULT_ID( 2,  9 ), // Ethernet PHY control register (applied if non-zero) 
    PMD_Default_TCPKeepAliveIdleTime    = DEFAULT_ID( 2, 11 ), // TCP connection keepalive idle time in seconds
    PMD_Default_COM1                    = DEFAULT_ID( 2, 14 ),
    PMD_Default_COM2                    = DEFAULT_ID( 2, 15 ),
    PMD_Default_RS485Duplex             = DEFAULT_ID( 2, 16 ),
    PMD_Default_CAN                     = DEFAULT_ID( 2, 17 ),
    PMD_Default_AutoStartMode           = DEFAULT_ID( 2, 20 ), // auto start the user task(s) after a reset
    PMD_Default_DebugSource             = DEFAULT_ID( 2, 23 ),
    PMD_Default_ConsoleIntfType         = DEFAULT_ID( 2, 24 ),
    PMD_Default_ConsoleIntfAddr         = DEFAULT_ID( 4, 25 ),
    PMD_Default_ConsoleIntfPort         = DEFAULT_ID( 2, 26 ),
    PMD_Default_Factory                 = 0xFFFF
} PMDDefaults;


enum PMD_PRPStatusEnum {
    PMD_PRPStatus_OK                    = 0,
    PMD_PRPStatus_Error                 = 1,
    PMD_PRPStatus_Outgoing              = 2
};

enum PMD_ResourceEnum {
    PMD_Resource_Device                 = 0,
    PMD_Resource_CMotionEngine          = 1,
    PMD_Resource_MotionProcessor        = 2,
    PMD_Resource_Memory                 = 3,
    PMD_Resource_Peripheral             = 4
};

enum PMD_ActionCodesEnum {
    PMD_Action_NOP                      = 0,
    PMD_Action_Reset                    = 1,
    PMD_Action_Command                  = 2,
    PMD_Action_Open                     = 3,
    PMD_Action_Close                    = 4,
    PMD_Action_Send                     = 5,
    PMD_Action_Receive                  = 6,
    PMD_Action_Write                    = 7,
    PMD_Action_Read                     = 8,
    PMD_Action_Set                      = 9,
    PMD_Action_Get                      = 10
};

// used with PMD_Action_Open

enum PMD_OpenCodesEnum {
    PMD_Open_Device                     = 0,
    PMD_Open_CMotionEngine              = 1,
    PMD_Open_MotionProcessor            = 2,
    PMD_Open_Memory                     = 3,
    PMD_Open_PeriphNone                 = 16,
    PMD_Open_PeriphPAR                  = 18,
    PMD_Open_PeriphISA                  = 19,
    PMD_Open_PeriphCOM                  = 20,
    PMD_Open_PeriphCAN                  = 21,
    PMD_Open_PeriphTCP                  = 22,
    PMD_Open_PeriphUDP                  = 23,
    PMD_Open_PeriphMultiDrop            = 25
};

// used with PMD_Action_Get and PMD_Action_Set
enum PMD_ValueCodesEnum {
    PMD_Value_Version                   = 1,
    PMD_Value_Default                   = 2,
    PMD_Value_ResetCause                = 3,
    PMD_Value_Console                   = 4,
    PMD_Value_TaskState                 = 5,
    PMD_Value_FileName                  = 6,
    PMD_Value_FileDate                  = 7,
    PMD_Value_FileChecksum              = 8,
    PMD_Value_FileVersion               = 9,
    PMD_Value_LogicVersion              = 10,
    PMD_Value_DPRAMSize                 = 11
};

enum PMD_CommandCodesEnum {
    PMD_Command_TaskControl             = 1,
    PMD_Command_Flash                   = 2
};

enum PMD_CommandFlashCodesEnum {
    PMD_Flash_Start                     = 1,
    PMD_Flash_Data                      = 2,
    PMD_Flash_End                       = 3,
    PMD_Flash_Erase                     = 4
};

enum PMD_CommandTaskControlEnum {
    PMD_TaskControl_StartAll            = 1,
    PMD_TaskControl_StopAll             = 2
};

typedef enum PMD_TaskStateEnum {
    PMD_TaskState_NoCode                = 1,
    PMD_TaskState_NotStarted            = 2,
    PMD_TaskState_Running               = 3,
    PMD_TaskState_Aborted               = 4
} PMDTaskState;

typedef enum PMDDataSizeEnum {
    PMDDataSize_8Bit                    = 1,
    PMDDataSize_16Bit                   = 2,
    PMDDataSize_32Bit                   = 4
} PMDDataSize;

typedef enum PMDMemoryTypeEnum {
    PMDMemoryType_DPRAM                 = 0,
    PMDMemoryType_NVRAM                 = 1
} PMDMemoryType;

typedef enum PMDSerialPortEnum {
    PMDSerialPort1                      = 0,
    PMDSerialPort2                      = 1,
    PMDSerialPort3                      = 2
} PMDSerialPort;

typedef enum PMDIndexerIOEnum {
    PMDIndexerIO_AI                     = 0x40,
    PMDIndexerIO_DI                     = 0x50,
    PMDIndexerIO_DO                     = 0x60,
    PMDIndexerIO_Direction              = 0x70
} PMDIndexerIO;

typedef enum PMDMachineIOEnum {
    PMDMachineIO_DI                     = 0x200,
    PMDMachineIO_DOMask                 = 0x210,
    PMDMachineIO_DO                     = 0x212,
    PMDMachineIO_DORead                 = 0x214,
    PMDMachineIO_DODirMask              = 0x220,
    PMDMachineIO_DODir                  = 0x222,
    PMDMachineIO_DODirRead              = 0x224,
    PMDMachineIO_AmpEnaMask             = 0x230,
    PMDMachineIO_AmpEna                 = 0x232,
    PMDMachineIO_AmpEnaRead             = 0x234,
    PMDMachineIO_IntPending             = 0x240,
    PMDMachineIO_IntMask                = 0x250,
    PMDMachineIO_IntPosEdge             = 0x260,
    PMDMachineIO_IntNegEdge             = 0x270,
    PMDMachineIO_AOCh1                  = 0x300,
    PMDMachineIO_AOCh2                  = 0x302,
    PMDMachineIO_AOCh3                  = 0x304,
    PMDMachineIO_AOCh4                  = 0x306,
    PMDMachineIO_AOCh5                  = 0x308,
    PMDMachineIO_AOCh6                  = 0x30A,
    PMDMachineIO_AOCh7                  = 0x30C,
    PMDMachineIO_AOCh8                  = 0x30E,
    PMDMachineIO_AOCh1Ena               = 0x310,
    PMDMachineIO_AOCh2Ena               = 0x312,
    PMDMachineIO_AOCh3Ena               = 0x314,
    PMDMachineIO_AOCh4Ena               = 0x316,
    PMDMachineIO_AOCh5Ena               = 0x318,
    PMDMachineIO_AOCh6Ena               = 0x31A,
    PMDMachineIO_AOCh7Ena               = 0x31C,
    PMDMachineIO_AOCh8Ena               = 0x31E,
    PMDMachineIO_AOEna                  = 0x320,
    PMDMachineIO_AICh1                  = 0x340,
    PMDMachineIO_AICh2                  = 0x342,
    PMDMachineIO_AICh3                  = 0x344,
    PMDMachineIO_AICh4                  = 0x346,
    PMDMachineIO_AICh5                  = 0x348,
    PMDMachineIO_AICh6                  = 0x34A,
    PMDMachineIO_AICh7                  = 0x34C,
    PMDMachineIO_AICh8                  = 0x34E,
} PMDMachineIO;

// extract CAN port params from single word
#define GET_CANMODE(CANmode, baud, address) \
    baud            = (CANmode & 0xE000) >> 13; \
    address         = (CANmode & 0x007F);

// compact CAN port params into single word
#define SET_CANMODE(CANmode, baud, address) \
    CANmode = address | (baud << 13);

// extract serial port params from single word
#define GET_SERIALPORTMODE(serialmode, baud, parity, stopbits, protocol, address) \
    baud            = (serialmode & 0x000F); \
    parity          = (serialmode & 0x0030) >> 4; \
    stopbits        = (serialmode & 0x0040) >> 6; \
    protocol        = (serialmode & 0x0180) >> 7; \
    address         = (serialmode & 0xF800) >> 11;

// compact serial port params into single word
#define SET_SERIALPORTMODE(serialmode, baud, parity, stopbits, protocol, address) \
    serialmode = (PMDuint16)(baud | (parity << 4) | (stopbits << 6) | (protocol << 7) | (address << 11));


#endif
