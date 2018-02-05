#ifndef _DEF_INC_PMDmutex
#define _DEF_INC_PMDmutex

#include <windows.h>

#define PMDMutexDefine(mutex)   static HANDLE mutex = NULL;
#define PMDMutexCreate(mutex)   if (mutex == NULL) mutex = CreateMutex( NULL, FALSE, #mutex );
#define PMDMutexLock(mutex)     (WAIT_OBJECT_0 == WaitForSingleObject( mutex, 2000 ))
#define PMDMutexUnLock(mutex)   ReleaseMutex( mutex );


#endif
