#ifndef _PMDSYS_H
#define _PMDSYS_H


void PMDputch( int ch );
void PMDputs( const char *str );
void PMDprintf( const char *fmt, ... );
int  PMDsprintf( char *str, const char *fmt, ... );
void PMDTaskWait( PMDuint32 milliseconds );
void PMDTaskWaitUntil(PMDuint32 *pPreviousWakeTime, PMDuint32 TimeIncrementms);
void PMDTaskAbort(int UserAbortCode);
int  PMDTaskGetAbortCode();
void PMDTaskEnterCritical();
void PMDTaskExitCritical();
PMDresult PMDTaskSetPriority(PMDparam priority);
PMDuint32 PMDDeviceGetTickCount();

#ifdef CME

#include "locations.h"

#define PMDTaskInfo void
typedef void (*FuncPointer)( PMDTaskInfo* pvTaskInfo );
#define EXPORTFUNC static const FuncPointer

void CopyDataSectionToRAM();

#define USER_CODE_VERSION( UCmajorversion, UCminorversion )  \
__attribute__((used, section ("header"))) \
static const char CodeSignature[12] = UC_DATA_SIGNATURE; \
__attribute__((used, section ("header"))) \
static const long Version = UC_DATA_CMEVERSION; \
__attribute__((used, section ("date"))) \
static const char DateTime[] = __DATE__"  " __TIME__; \
static const char UCFileName[] = __FILE__;  \
__attribute__((used, section ("filename"))) \
static const char* pUCFileName = UCFileName;  \
__attribute__((used, section ("fileversion")))   \
static const unsigned long UCFileVersion = (UCmajorversion << 16) | UCminorversion; \
__attribute__((used, section ("funcstart"))) \
EXPORTFUNC CopyDataSectionToRAM_address = CopyDataSectionToRAM;

#define USER_CODE_LOOP_BEGIN while(1) {
#define USER_CODE_LOOP_END   }

// User code task function definition macro
// note: the ## is used to concatenate strings in macros
#define USER_CODE_TASK( sTaskName ) \
	void sTaskName( PMDTaskInfo* pvTaskInfo ); \
	__attribute__((used, section ("functable"))) \
	EXPORTFUNC sTaskName ## _address = sTaskName ;  \
	__attribute__((used, section ("funcnames"))) \
	static const char sTaskName ## _name[] = #sTaskName ;  \
	__attribute__((used, section ("funcstack"))) \
	static const long sTaskName ## _stack = 0;  \
	void sTaskName( PMDTaskInfo* pvTaskInfo ) 

#define USER_CODE_TASK_STACK( sTaskName, stacksize ) \
	void sTaskName( PMDTaskInfo* pvTaskInfo ); \
	__attribute__((used, section ("functable"))) \
	EXPORTFUNC sTaskName ## _address = sTaskName ;  \
	__attribute__((used, section ("funcnames"))) \
	static const char sTaskName ## _name[] = #sTaskName ;  \
	__attribute__((used, section ("funcstack"))) \
	static const long sTaskName ## _stack = stacksize;  \
	void sTaskName( PMDTaskInfo* pvTaskInfo ) 

#else // not CME

#include <stdio.h>
#include <windows.h>
#include <assert.h>
#define PMDputch                  putch
#define PMDputs                   puts
#define PMDprintf                 printf
#define PMDsprintf                sprintf
#define PMDTaskWait(ms)           Sleep(ms)
#define PMDTaskWaitUntil(a, ms)   Sleep(ms)
#define PMDTaskAbort(code)        exit(code)
#define PMDTaskEnterCritical()    
#define PMDTaskExitCritical()     
#define PMDDeviceGetTickCount()   GetTickCount()
#define USER_CODE_TASK(a)         int main(int argc, char* argv[])
#define USER_CODE_VERSION(major, minor)

#endif


#endif
