REM Rebuild both the release and debug versions of all customer examples.
REM Delete the /REBUILD argument to only build whatever is out of date.
REM This requires that your environment looks like vcvars32.bat has been run.

REM Visual Studio 6.0
msdev  Examples.dsw /MAKE "all - Win32 Debug" /REBUILD
msdev  Examples.dsw /MAKE "all - Win32 Release" /REBUILD

REM Visual Studio .NET 
REM devenv Examples.sln /rebuild Debug
REM devenv Examples.sln /rebuild Release

