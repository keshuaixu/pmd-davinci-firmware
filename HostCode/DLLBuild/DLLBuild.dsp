# Microsoft Developer Studio Project File - Name="DLLBuild" - Package Owner=<4>
# Microsoft Developer Studio Generated Build File, Format Version 6.00
# ** DO NOT EDIT **

# TARGTYPE "Win32 (x86) Dynamic-Link Library" 0x0102

CFG=DLLBuild - Win32 Debug
!MESSAGE This is not a valid makefile. To build this project using NMAKE,
!MESSAGE use the Export Makefile command and run
!MESSAGE 
!MESSAGE NMAKE /f "DLLBuild.mak".
!MESSAGE 
!MESSAGE You can specify a configuration when running NMAKE
!MESSAGE by defining the macro CFG on the command line. For example:
!MESSAGE 
!MESSAGE NMAKE /f "DLLBuild.mak" CFG="DLLBuild - Win32 Debug"
!MESSAGE 
!MESSAGE Possible choices for configuration are:
!MESSAGE 
!MESSAGE "DLLBuild - Win32 Release" (based on "Win32 (x86) Dynamic-Link Library")
!MESSAGE "DLLBuild - Win32 Debug" (based on "Win32 (x86) Dynamic-Link Library")
!MESSAGE 

# Begin Project
# PROP AllowPerConfigDependencies 0
# PROP Scc_ProjName ""
# PROP Scc_LocalPath ""
CPP=cl.exe
MTL=midl.exe
RSC=rc.exe

!IF  "$(CFG)" == "DLLBuild - Win32 Release"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 0
# PROP BASE Output_Dir "Release"
# PROP BASE Intermediate_Dir "Release"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 0
# PROP Output_Dir "..\Release"
# PROP Intermediate_Dir "Release"
# PROP Ignore_Export_Lib 0
# PROP Target_Dir ""
# ADD BASE CPP /nologo /MT /W3 /GX /O2 /D "WIN32" /D "NDEBUG" /D "_WINDOWS" /D "_MBCS" /D "_USRDLL" /YX /FD /c
# ADD CPP /nologo /MT /W3 /GX /O2 /I "../../C-Motion/Include" /I "../../C-Motion/Include/IXXAT" /D "NDEBUG" /D "WIN32" /D "_WINDOWS" /D "_MBCS" /D "_USRDLL" /D "USE_PCI_INTERFACE" /D "PMD_EXPORTS" /YX /FD /c
# ADD BASE MTL /nologo /D "NDEBUG" /mktyplib203 /win32
# ADD MTL /nologo /D "NDEBUG" /mktyplib203 /win32
# ADD BASE RSC /l 0x409 /d "NDEBUG"
# ADD RSC /l 0x409 /d "NDEBUG"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LINK32=link.exe
# ADD BASE LINK32 kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib /nologo /dll /machine:I386
# ADD LINK32 kernel32.lib Ws2_32.lib delayimp.lib /nologo /dll /machine:I386 /out:"..\Release/C-Motion.dll" /DELAYLOAD:plxapi650.dll /DELAYLOAD:vcinpl.dll
# SUBTRACT LINK32 /pdb:none

!ELSEIF  "$(CFG)" == "DLLBuild - Win32 Debug"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 1
# PROP BASE Output_Dir "Debug"
# PROP BASE Intermediate_Dir "Debug"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 1
# PROP Output_Dir "..\Debug"
# PROP Intermediate_Dir "Debug"
# PROP Ignore_Export_Lib 0
# PROP Target_Dir ""
# ADD BASE CPP /nologo /MTd /W3 /Gm /GX /ZI /Od /D "WIN32" /D "_DEBUG" /D "_WINDOWS" /D "_MBCS" /D "_USRDLL" /YX /FD /GZ /c
# ADD CPP /nologo /MTd /W3 /Gm /GX /ZI /Od /I "../../C-Motion/Include" /I "../../C-Motion/Include/IXXAT" /D "_DEBUG" /D "WIN32" /D "_WINDOWS" /D "_MBCS" /D "_USRDLL" /D "USE_PCI_INTERFACE" /D "PMD_EXPORTS" /FR /YX /FD /GZ /c
# ADD BASE MTL /nologo /D "_DEBUG" /mktyplib203 /win32
# ADD MTL /nologo /D "_DEBUG" /mktyplib203 /win32
# ADD BASE RSC /l 0x409 /d "_DEBUG"
# ADD RSC /l 0x409 /d "_DEBUG"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LINK32=link.exe
# ADD BASE LINK32 kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib /nologo /dll /debug /machine:I386 /pdbtype:sept
# ADD LINK32 kernel32.lib Ws2_32.lib delayimp.lib /nologo /dll /debug /machine:I386 /out:"..\Debug/C-Motion.dll" /pdbtype:sept /DELAYLOAD:plxapi650.dll /DELAYLOAD:vcinpl.dll
# SUBTRACT LINK32 /pdb:none

!ENDIF 

# Begin Target

# Name "DLLBuild - Win32 Release"
# Name "DLLBuild - Win32 Debug"
# Begin Group "Source Files"

# PROP Default_Filter "cpp;c;cxx;rc;def;r;odl;idl;hpj;bat"
# Begin Source File

SOURCE="..\..\C-Motion\C\c-motion.c"
# End Source File
# Begin Source File

SOURCE=".\c-motion.def"
# End Source File
# Begin Source File

SOURCE=.\dllfuns.c
# End Source File
# Begin Source File

SOURCE="..\..\C-Motion\C\PMDcan.c"
# End Source File
# Begin Source File

SOURCE="..\..\C-Motion\C\PMDcommon.c"
# End Source File
# Begin Source File

SOURCE="..\..\C-Motion\C\PMDdiag.c"
# End Source File
# Begin Source File

SOURCE="..\..\C-Motion\C\PMDIXXATCAN3.c"
# End Source File
# Begin Source File

SOURCE="..\..\C-Motion\C\PMDMB.c"
# End Source File
# Begin Source File

SOURCE="..\..\C-Motion\C\PMDopen.c"
# End Source File
# Begin Source File

SOURCE="..\..\C-Motion\C\PMDpar.c"
# End Source File
# Begin Source File

SOURCE="..\..\C-Motion\C\PMDPcan.c"
# End Source File
# Begin Source File

SOURCE="..\..\C-Motion\C\PMDPpci.c"
# End Source File
# Begin Source File

SOURCE="..\..\C-Motion\C\PMDPser.c"
# End Source File
# Begin Source File

SOURCE="..\..\C-Motion\C\PMDPtcp.c"
# End Source File
# Begin Source File

SOURCE="..\..\C-Motion\C\PMDRP.cpp"
# End Source File
# Begin Source File

SOURCE="..\..\C-Motion\C\PMDRPdevice.cpp"
# End Source File
# Begin Source File

SOURCE="..\..\C-Motion\C\PMDRPperiph.cpp"
# End Source File
# Begin Source File

SOURCE="..\..\C-Motion\C\PMDser.c"
# End Source File
# Begin Source File

SOURCE="..\..\C-Motion\C\PMDtrans.c"
# End Source File
# End Group
# Begin Group "Header Files"

# PROP Default_Filter "h;hpp;hxx;hm;inl"
# Begin Source File

SOURCE="..\..\C-Motion\Include\c-motion.h"
# End Source File
# Begin Source File

SOURCE=.\dllfuns.h
# End Source File
# Begin Source File

SOURCE="..\..\C-Motion\Include\PMDdevice.h"
# End Source File
# Begin Source File

SOURCE="..\..\C-Motion\Include\PMDdiag.h"
# End Source File
# Begin Source File

SOURCE="..\..\C-Motion\Include\pmdecode.h"
# End Source File
# Begin Source File

SOURCE="..\..\C-Motion\Include\PMDintf.h"
# End Source File
# Begin Source File

SOURCE="..\..\C-Motion\Include\PMDIXXATCAN.h"
# End Source File
# Begin Source File

SOURCE="..\..\C-Motion\Include\PMDMB.h"
# End Source File
# Begin Source File

SOURCE="..\..\C-Motion\Include\PMDmutex.h"
# End Source File
# Begin Source File

SOURCE="..\..\C-Motion\Include\PMDocode.h"
# End Source File
# Begin Source File

SOURCE="..\..\C-Motion\Include\PMDperiph.h"
# End Source File
# Begin Source File

SOURCE="..\..\C-Motion\Include\PMDPfunc.h"
# End Source File
# Begin Source File

SOURCE="..\..\C-Motion\Include\PMDRP.h"
# End Source File
# Begin Source File

SOURCE="..\..\C-Motion\Include\PMDRPdevice.h"
# End Source File
# Begin Source File

SOURCE="..\..\C-Motion\Include\PMDRPperiph.h"
# End Source File
# Begin Source File

SOURCE="..\..\C-Motion\Include\PMDRPtypes.h"
# End Source File
# Begin Source File

SOURCE="..\..\C-Motion\Include\PMDsys.h"
# End Source File
# Begin Source File

SOURCE="..\..\C-Motion\Include\PMDtrans.h"
# End Source File
# Begin Source File

SOURCE="..\..\C-Motion\Include\PMDtypes.h"
# End Source File
# End Group
# Begin Group "Resource Files"

# PROP Default_Filter "ico;cur;bmp;dlg;rc2;rct;bin;rgs;gif;jpg;jpeg;jpe"
# End Group
# Begin Source File

SOURCE="..\..\C-Motion\Include\PLX\PlxApi.lib"
# End Source File
# Begin Source File

SOURCE="..\..\C-Motion\Include\IXXAT\vcisdk.lib"
# End Source File
# End Target
# End Project
