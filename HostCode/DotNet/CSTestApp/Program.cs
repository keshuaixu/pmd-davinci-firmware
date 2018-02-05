///////////////////////////////////////////////////////////////////////////
//
//	Program.cs -- PMD C# example program
//
//	Performance Motion Devices, Inc.
//
//  This example demonstrates Magellan communications.
//
//  Dependencies:
//      PMDLibrary.dll
//      C-Motion.dll
///////////////////////////////////////////////////////////////////////////


using System;
using System.Collections.Generic;
using System.Text;
using System.Threading;
using PMDLibrary;

    class Example
    {
        PMD.PMDPeripheral periph;
        PMD.PMDDevice device;

        static void Main(string[] args)
        {
            Example ex = new Example();
            ex.Run();
        }
        public void Run()
        {
            try
            {
                bool bPRP = true;
                // connect to a PMD CME product via the PRP protocol over Ethernet.
                if (bPRP)
                {
                    String ipaddress = "192.168.2.2";
                    // connect to Machine Controller.
                    periph = new PMD.PMDPeripheralTCP(System.Net.IPAddress.Parse(ipaddress), 40100, 1000);
                    Console.WriteLine("Connected to {0}", ipaddress);
                    device = new PMD.PMDDevice(periph, PMD.PMDDeviceType.ResourceProtocol);
                }
                // connect to a PMD Magellan product via the Magellan protocol over the COM1 serial interface.
                else
                {
                    periph = new PMD.PMDPeripheralCOM(0, PMD.PMDSerialBaud.Baud57600, PMD.PMDSerialParity.None, PMD.PMDSerialStopBits.Bits1);
                    device = new PMD.PMDDevice(periph, PMD.PMDDeviceType.MotionProcessor);
                }

                PMD.PMDAxis axis1 = new PMD.PMDAxis(device, PMD.PMDAxisNumber.Axis1);

                Int32 pos;
                // C-Motion procedures returning a single value become class properties, and may be
                // retrieved or set by using an assignment.  The "Get" or "Set" part of the name is dropped.
                pos = axis1.ActualPosition;

                Console.WriteLine("Actual Position = {0}", pos);

                // The following line sets the actual position of the axis to zero.
                axis1.ActualPosition = 0;
              
                axis1.Close();
                device.Close();
                periph.Close();
            }
            catch (Exception e)
            {
                Console.WriteLine(e.Message);
            }
            Console.WriteLine("Done. Press any key to end program");
            Console.ReadKey();
        }
    }

