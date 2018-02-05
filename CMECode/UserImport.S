#include "locations.h"

.globl  PMDAxisOpen
.globl  PMDMPDeviceOpen 
.globl  PMDRPDeviceOpen 
.globl  PMDWaitForEvent
.globl  PMDMemoryOpen
.globl  PMDPeriphOpenCME
.globl  PMDPeriphOpenTCP
.globl  PMDPeriphOpenUDP
.globl  PMDPeriphOpenCAN
.globl  PMDPeriphOpenCANRange
.globl  PMDPeriphOpenISA
.globl  PMDPeriphOpenPIO
.globl  PMDPeriphOpenCOM
.globl  PMDPeriphOpenMultiDrop
.globl  PMDprintf
.globl  PMDsprintf
.globl  PMDputs
.globl  PMDputch
.globl  PMDTaskStart
.globl  PMDTaskWait
.globl  PMDTaskWaitUntil
.globl  PMDTaskAbort
.globl  PMDTaskGetState
.globl  PMDTaskGetAbortCode
.globl  PMDTaskEnterCritical
.globl  PMDTaskExitCritical
.globl  PMDTaskSetPriority
.globl  PMDDeviceGetTickCount
.globl  PMDDeviceGetVersion
.globl  PMDDeviceGetDefault
.globl  PMDDeviceSetDefault
.globl  PMDDeviceReset
.globl  PMDDeviceGetResetCause
.globl  PMDDeviceSetConsole
.globl  PMDDeviceNoOperation
.globl  PMDDeviceStoreUserCode
.globl	PMDGetUserCodeFileVersion
.globl	PMDGetUserCodeFileDate
.globl	PMDGetUserCodeFileName
.globl  PMDTimerInit
.globl  PMDTimerWait
.globl  ReadPIObus
.globl  WritePIObus
.globl  PMDInterruptWait


#define location                table    ; .set table, table + 4
.set table,                          OS_ADDRESS_FUNC_TABLE

.equ  Reserved                      ,location
.equ  PMDAxisOpen                   ,location
.equ  Reserved                      ,location
.equ  PMDMPDeviceOpen               ,location
.equ  PMDRPDeviceOpen               ,location
.equ  PMDMemoryOpen                 ,location
.equ  Reserved                      ,location
.equ  Reserved                      ,location
.equ  Reserved                      ,location
.equ  Reserved                      ,location
.equ  Reserved                      ,location
.equ  PMDPeriphOpenCME              ,location
.equ  Reserved                      ,location
.equ  PMDPeriphOpenPIO              ,location
.equ  PMDPeriphOpenISA              ,location
.equ  PMDPeriphOpenCOM              ,location
.equ  PMDPeriphOpenCAN              ,location
.equ  PMDPeriphOpenTCP              ,location
.equ  PMDPeriphOpenUDP              ,location
.equ  PMDPeriphOpenCANRange         ,location
.equ  PMDPeriphOpenMultiDrop        ,location
.equ  Reserved                      ,location
.equ  Reserved                      ,location
.equ  PMDprintf                     ,location
.equ  PMDsprintf                    ,location
.equ  PMDputs                       ,location
.equ  PMDputch                      ,location
.equ  Reserved                      ,location
.equ  Reserved                      ,location
.equ  Reserved                      ,location
.equ  PMDTaskStart                  ,location
.equ  PMDTaskWait                   ,location
.equ  PMDTaskWaitUntil              ,location
.equ  PMDTaskGetState               ,location
.equ  PMDTaskAbort                  ,location
.equ  PMDTaskGetAbortCode           ,location
.equ  PMDTaskSetPriority            ,location
.equ  PMDTaskEnterCritical          ,location
.equ  PMDTaskExitCritical           ,location
.equ  Reserved                      ,location
.equ  PMDDeviceGetVersion           ,location
.equ  PMDDeviceGetDefault           ,location
.equ  PMDDeviceSetDefault           ,location
.equ  PMDDeviceGetTickCount         ,location
.equ  PMDDeviceReset                ,location
.equ  PMDDeviceGetResetCause        ,location
.equ  PMDDeviceSetConsole           ,location
.equ  PMDDeviceNoOperation          ,location
.equ  PMDDeviceStoreUserCode        ,location
.equ  PMDGetUserCodeFileVersion     ,location
.equ  PMDGetUserCodeFileName        ,location
.equ  PMDGetUserCodeFileDate        ,location
.equ  Reserved                      ,location
.equ  Reserved                      ,location
.equ  Reserved                      ,location
.equ  Reserved                      ,location
.equ  Reserved                      ,location
.equ  Reserved                      ,location
.equ  Reserved                      ,location
.equ  Reserved                      ,location
