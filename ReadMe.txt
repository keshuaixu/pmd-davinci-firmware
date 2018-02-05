CMESKD Examples Readme.txt
============================================================

This readme file contains information pertaining to the software examples
provided in the PMD Prodigy CME SDK.

Most of these examples have the option to use several different
communication interfaces.  Each example contains the necessary interface 
connection functions appropriate for the particular product 
(ie. PMDPeriphOpenTCP).  It will be necessary to uncomment the desired one.

Many of the interface options may require more editing for
configuration, for example to select IP addresses, CANBus node
identifiers, serial communications ports, and so forth.  Comments in
the individual source files indicate which settings are likely to need
editing.

Preprocessor directives may be used to select specific modes of operation
such as: 

#define CME
Some of the provided examples can run on both the Prodigy board and a Windows
PC, while others will only run on one or the other, but not both.  Whether or
not an example is being compiled to run on a Prodigy/CME or Windows PC will 
affect what commands are supported.  Once again, a preprocessor directive is
used to assist with the compilation.  When "#define CME" is true, the example
will compile for use on a Prodigy/CME board (this would be the typical case when
Programmer's Notepad is in use).  Otherwise if "CME" is not defined then the
example will compile for use on a Windows PC ( this would be the typical case
when Visual Studio is in use).  Note that "CME" is a directive defined in the
Makefile used to compile all CME code.


Some examples will have a mating example. For instance if 
HostCode\Examples\GenericUserPacket.c is to be run on a WindowsPC,  
the corresponding CMECode\Examples\GenericUserPacket.c should
be running on the Prodigy board.  


Example set as of Dec 1, 2013:

CME Examples:
CMECode\Examples\Hello
CMECode\Examples\IONCME*
CMECode\Examples\ProdigyCME*
CMECode\Examples\MachineController*
CMECode\Examples\GenericUserPacket

Host Examples (Windows PC):
CMECode\Examples\IONCME*
CMECode\Examples\ProdigyCME*
CMECode\Examples\MachineController*
HostCode\Examples\GenericUserPacket

(*) These examples share one source file that can be compiled to run on a Prodigy/CME board or a WindowsPC.


Hello.c
===============
This example runs on the Prodigy/CME only.  It will print out a debug “Hello” message
to the Console window.  It will then query the ActualPosition of Axis1 of the on board 
Magellan controller and will send the ActualPosition value over to the debug console
if the position changes.



IONCME.c
ProdigyCME.c
MachineController.c
================
These examples can be run on the device or from a PC host. After the
interface is established it will call several functions that demonstrate
features of the card.

The SetupAxis1() function sets all the parameters of the processor based on the 
ProMotionExport.c file exported from Pro-Motion. (select File/Export as C-Motion).

The AllMagellanCommands() function send all supported commands through the interface.
This example is really only meant to be used as a command reference. 

The ProfileMove() will attempt to perform a trapezoidal profile move. 
The function will return when the profile completes or a timeout occurs.

The IOExample() function reads/writes the IO signals of the device's IO connector.



GenericUserPacket.c
====================
This example contains two distinct pieces of source code.  One that runs on the
Prodigy/CME card (found in the CMECoder/Examples folder) and another that runs on
a WindowsPC (found in HostCode/Examples). 

On the Prodigy/CME side this example will open a port and query that port in a loop to
check for received data.  If data if received it will send the data back out the peripheral.

On the WindowPC side this example will open a port and send a few bytes of data, it will
then wait to receive data.









