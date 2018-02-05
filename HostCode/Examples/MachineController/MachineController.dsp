# Microsoft Developer Studio Project File - Name="MachineController" - Package Owner=<4>
# Microsoft Developer Studio Generated Build File, Format Version 6.00
# ** DO NOT EDIT **

# TARGTYPE "Win32 (x86) Console Application" 0x0103

CFG=MachineController - Win32 Debug
!MESSAGE This is not a valid makefile. To build this project using NMAKE,
!MESSAGE use the Export Makefile command and run
!MESSAGE 
!MESSAGE NMAKE /f "MachineController.mak".
!MESSAGE 
!MESSAGE You can specify a configuration when running NMAKE
!MESSAGE by defining the macro CFG on the command line. For example:
!MESSAGE 
!MESSAGE NMAKE /f "MachineController.mak" CFG="MachineController - Win32 Debug"
!MESSAGE 
!MESSAGE Possible choices for configuration are:
!MESSAGE 
!MESSAGE "MachineController - Win32 Release" (based on "Win32 (x86) Console Application")
!MESSAGE "MachineController - Win32 Debug" (based on "Win32 (x86) Console Application")
!MESSAGE 

# Begin Project
# PROP AllowPerConfigDependencies 0
# PROP Scc_ProjName ""
# PROP Scc_LocalPath ""
CPP=cl.exe
RSC=rc.exe

!IF  "$(CFG)" == "MachineController - Win32 Release"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 0
# PROP BASE Output_Dir "Release"
# PROP BASE Intermediate_Dir "Release"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 0
# PROP Output_Dir "Release"
# PROP Intermediate_Dir "Release"
# PROP Ignore_Export_Lib 0
# PROP Target_Dir ""
# ADD BASE CPP /nologo /W3 /GX /O2 /D "WIN32" /D "NDEBUG" /D "_CONSOLE" /D "_MBCS" /YX /FD /c
# ADD CPP /nologo /MT /W3 /GX /O2 /I "..\..\..\c-motion\include" /I "..\..\..\c-motion\include\ixxat" /D "WIN32" /D "NDEBUG" /D "_CONSOLE" /D "_MBCS" /YX /FD /c
# ADD BASE RSC /l 0x409 /d "NDEBUG"
# ADD RSC /l 0x409 /d "NDEBUG"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LINK32=link.exe
# ADD BASE LINK32 kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib /nologo /subsystem:console /machine:I386
# ADD LINK32 Ws2_32.lib delayimp.lib /nologo /subsystem:console /machine:I386 /libpath:"../../Release" /DELAYLOAD:vcinpl.dll
# SUBTRACT LINK32 /pdb:none

!ELSEIF  "$(CFG)" == "MachineController - Win32 Debug"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 1
# PROP BASE Output_Dir "Debug"
# PROP BASE Intermediate_Dir "Debug"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 1
# PROP Output_Dir "Debug"
# PROP Intermediate_Dir "Debug"
# PROP Ignore_Export_Lib 0
# PROP Target_Dir ""
# ADD BASE CPP /nologo /W3 /Gm /GX /ZI /Od /D "WIN32" /D "_DEBUG" /D "_CONSOLE" /D "_MBCS" /YX /FD /GZ /c
# ADD CPP /nologo /MTd /W3 /Gm /GX /ZI /Od /I "..\..\..\c-motion\include" /I "..\..\..\c-motion\include\ixxat" /D "WIN32" /D "_DEBUG" /D "_CONSOLE" /D "_MBCS" /FR /YX /FD /GZ /c
# ADD BASE RSC /l 0x409 /d "_DEBUG"
# ADD RSC /l 0x409 /d "_DEBUG"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LINK32=link.exe
# ADD BASE LINK32 kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib /nologo /subsystem:console /debug /machine:I386 /pdbtype:sept
# ADD LINK32 Ws2_32.lib delayimp.lib /nologo /subsystem:console /debug /machine:I386 /pdbtype:sept /libpath:"../../Debug" /opt:ref /DELAYLOAD:vcinpl.dll
# SUBTRACT LINK32 /pdb:none

!ENDIF 

# Begin Target

# Name "MachineController - Win32 Release"
# Name "MachineController - Win32 Debug"
# Begin Group "Source Files"

# PROP Default_Filter "cpp;c;cxx;rc;def;r;odl;idl;hpj;bat"
# Begin Source File

SOURCE="..\..\..\C-Motion\C\c-motion.c"
# End Source File
# Begin Source File

SOURCE="..\..\..\C-Motion\C\Examples.c"
# End Source File
# Begin Source File

SOURCE=..\..\..\CMECode\Examples\MachineController\MachineController.c
# End Source File
# Begin Source File

SOURCE="..\..\..\C-Motion\C\PMDcan.c"
# End Source File
# Begin Source File

SOURCE="..\..\..\C-Motion\C\PMDcommon.c"
# End Source File
# Begin Source File

SOURCE="..\..\..\C-Motion\C\PMDdiag.c"
# End Source File
# Begin Source File

SOURCE="..\..\..\C-Motion\C\PMDIXXATCAN3.c"
# End Source File
# Begin Source File

SOURCE="..\..\..\C-Motion\C\PMDopen.c"
# End Source File
# Begin Source File

SOURCE="..\..\..\C-Motion\C\PMDpar.c"
# End Source File
# Begin Source File

SOURCE="..\..\..\C-Motion\C\PMDPcan.c"
# End Source File
# Begin Source File

SOURCE="..\..\..\C-Motion\C\PMDPser.c"
# End Source File
# Begin Source File

SOURCE="..\..\..\C-Motion\C\PMDPtcp.c"
# End Source File
# Begin Source File

SOURCE="..\..\..\C-Motion\C\PMDRP.cpp"
# End Source File
# Begin Source File

SOURCE="..\..\..\C-Motion\C\PMDRPdevice.cpp"
# End Source File
# Begin Source File

SOURCE="..\..\..\C-Motion\C\PMDRPperiph.cpp"
# End Source File
# Begin Source File

SOURCE="..\..\..\C-Motion\C\PMDser.c"
# End Source File
# Begin Source File

SOURCE="..\..\..\C-Motion\C\PMDtrans.c"
# End Source File
# Begin Source File

SOURCE="..\..\..\C-Motion\C\PMDutil.c"
# End Source File
# Begin Source File

SOURCE="..\..\..\CMECode\Examples\MachineController\Pro-MotionExport.c"
# End Source File
# End Group
# Begin Group "Header Files"

# PROP Default_Filter "h;hpp;hxx;hm;inl"
# Begin Source File

SOURCE="..\..\..\C-Motion\Include\c-motion.h"
# End Source File
# Begin Source File

SOURCE="..\..\..\C-Motion\Include\Examples.h"
# End Source File
# Begin Source File

SOURCE="..\..\..\C-Motion\Include\PMDdevice.h"
# End Source File
# Begin Source File

SOURCE="..\..\..\C-Motion\Include\PMDdiag.h"
# End Source File
# Begin Source File

SOURCE="..\..\..\C-Motion\Include\PMDecode.h"
# End Source File
# Begin Source File

SOURCE="..\..\..\C-Motion\Include\PMDintf.h"
# End Source File
# Begin Source File

SOURCE="..\..\..\C-Motion\Include\PMDIXXATCAN.h"
# End Source File
# Begin Source File

SOURCE="..\..\..\C-Motion\Include\PMDMB.h"
# End Source File
# Begin Source File

SOURCE="..\..\..\C-Motion\Include\PMDMotorSetup.h"
# End Source File
# Begin Source File

SOURCE="..\..\..\C-Motion\Include\PMDmutex.h"
# End Source File
# Begin Source File

SOURCE="..\..\..\C-Motion\Include\PMDocode.h"
# End Source File
# Begin Source File

SOURCE="..\..\..\C-Motion\Include\PMDperiph.h"
# End Source File
# Begin Source File

SOURCE="..\..\..\C-Motion\Include\PMDPfunc.h"
# End Source File
# Begin Source File

SOURCE="..\..\..\C-Motion\Include\PMDRP.h"
# End Source File
# Begin Source File

SOURCE="..\..\..\C-Motion\Include\PMDrpdevice.h"
# End Source File
# Begin Source File

SOURCE="..\..\..\C-Motion\Include\PMDRPperiph.h"
# End Source File
# Begin Source File

SOURCE="..\..\..\C-Motion\Include\PMDRPtypes.h"
# End Source File
# Begin Source File

SOURCE="..\..\..\C-Motion\Include\PMDsys.h"
# End Source File
# Begin Source File

SOURCE="..\..\..\C-Motion\Include\PMDtrans.h"
# End Source File
# Begin Source File

SOURCE="..\..\..\C-Motion\Include\PMDtypes.h"
# End Source File
# Begin Source File

SOURCE="..\..\..\C-Motion\Include\PMDutil.h"
# End Source File
# End Group
# Begin Group "Resource Files"

# PROP Default_Filter "ico;cur;bmp;dlg;rc2;rct;bin;rgs;gif;jpg;jpeg;jpe"
# End Group
# Begin Source File

SOURCE="..\..\..\C-Motion\Include\IXXAT\vcisdk.lib"
# End Source File
# End Target
# End Project
