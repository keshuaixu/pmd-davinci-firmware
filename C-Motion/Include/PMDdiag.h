#ifndef PMD_Diagnostics
#define	PMD_Diagnostics

//  PMDdiag.h -- diagnostic functions
//
//  Performance Motion Devices, Inc.
//

#include "PMDtypes.h"


#define PMD_RESULT(_call)  {result = _call; if (result) PMDprintf("Error line: %d, %s, %30s\n", __LINE__, PMDGetErrorMessage(result), #_call);}
#define PMD_ABORTONERROR(_call)  PMD_RESULT(_call) if (result) PMDTaskAbort(result);

#if defined(__cplusplus)
extern "C" {
#endif

const char *PMDGetOpcodeText(PMDuint16 opCode);
const char *PMDGetErrorMessage(PMDresult errorCode);

#if defined(__cplusplus)
}
#endif

#endif

