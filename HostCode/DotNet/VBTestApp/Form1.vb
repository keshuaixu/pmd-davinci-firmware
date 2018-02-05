Imports System.Text

Imports VBTestApp.PMDPeripheral
Imports VBTestApp.PMD


Public Class Form1
    Dim periph1 As PMDPeripheral
    Dim device1 As PMDDevice
    Dim axis1 As PMDAxis
    Dim memory1 As PMDMemory
    Dim periph2 As PMDPeripheral
    Dim device2 As PMDDevice
    Dim axis2 As PMDAxis
    Dim ra1(0 To 10) As UInt32
    Dim major, minor As UInt32
    Dim MPmajor, MPminor, NumberAxes, special, custom, family As UInt16
    Dim MotorType As PMDMotorTypeVersion
    Dim DeviceType As PMDDeviceType
    Const TCPTimeout = 1000

    Public Sub CloseAll()
        If (memory1 IsNot Nothing) Then
            memory1.Close()
        End If
        If (axis1 IsNot Nothing) Then
            axis1.Close()
        End If
        If (device1 IsNot Nothing) Then
            device1.Close()
        End If
        If (periph1 IsNot Nothing) Then
            periph1.Close()
        End If
        If (axis2 IsNot Nothing) Then
            axis2.Close()
        End If
        If (device2 IsNot Nothing) Then
            device2.Close()
        End If
        If (periph2 IsNot Nothing) Then
            periph2.Close()
        End If
    End Sub

    Public Sub New()

        DeviceType = PMDDeviceType.ResourceProtocol
        'DeviceType = PMDDeviceType.MotionProcessor

        ' This call is required by the Windows Form Designer.
        InitializeComponent()

        ' Add any initialization after the InitializeComponent() call.
        LabelVersionCOM.Text = ""
    End Sub

    Private Sub ButtonCOM_click(ByVal Sender As System.Object, ByVal e As System.EventArgs) Handles ButtonCOM.Click
        Dim i As UInt32
        Dim IPAddr As UInt32
        Try
            periph1 = New PMDPeripheralCOM(PMDSerialPort.COM1, PMDSerialBaud.Baud57600, PMDSerialParity.None, PMDSerialStopBits.Bits1)
            device1 = New PMDDevice(periph1, DeviceType)
            device1.Version(major, minor)
            axis1 = New PMDAxis(device1, PMDAxisNumber.Axis1)
            axis1.GetVersion(family, MotorType, NumberAxes, special, custom, MPmajor, MPminor)
            LabelVersionCOM.Text = "version: " & major & "." & minor.ToString("00") & " Magellan " & MPmajor & "." & MPminor
            IPAddr = device1.GetDefault(PMDDefault.IPAddress)
            LabelDefaultsCOM.Text = "IPAddr: " & (IPAddr >> 24).ToString("000") & "." & _
                                                 ((IPAddr >> 16) And &HFF).ToString("000") & "." & _
                                                 ((IPAddr >> 8) And &HFF).ToString("000") & "." & _
                                                 (IPAddr And &HFF).ToString("000")
            memory1 = New PMDMemory(device1, PMDDataSize.Size32Bit, PMDMemoryType.DPRAM)
            ListBox1.Items.Clear()
            For i = 0 To ra1.GetUpperBound(0)
                ra1(i) = i
                ListBox1.Items.Insert(i, ra1(i))
            Next
            memory1.Write(ra1, 100, ra1.GetUpperBound(0) + 1)
        Catch ex As Exception
            MsgBox(ex.Message)
        End Try
        CloseAll()

    End Sub

    Private Sub ButtonTCP_click(ByVal Sender As System.Object, ByVal e As System.EventArgs) Handles ButtonTCP.Click
        Dim ra1(0 To 10) As UInt32
        Dim major, minor As UInt32
        Dim MPmajor, MPminor, NumberAxes, special, custom, family As UInt16
        Dim MotorType As PMDMotorTypeVersion
        Dim i As UInt32
        Try
            periph1 = New PMDPeripheralTCP(System.Net.IPAddress.Parse("192.168.2.2"), DEFAULT_ETHERNET_PORT, 1000)
            device1 = New PMDDevice(periph1, PMDDeviceType.ResourceProtocol)
            device1.Version(major, minor)
            axis1 = New PMDAxis(device1, PMDAxisNumber.Axis1)
            axis1.GetVersion(family, MotorType, NumberAxes, special, custom, MPmajor, MPminor)
            LabelVersionTCP.Text = "version: " & major & "." & minor.ToString("00") & " Magellan " & MPmajor & "." & MPminor
            memory1 = New PMDMemory(device1, PMDDataSize.Size32Bit, PMDMemoryType.DPRAM)
            memory1.Read(ra1, 100, ra1.Length())
            ListBox1.Items.Clear()
            For i = 0 To ra1.Length() - 1
                ListBox1.Items.Insert(i, ra1(i))
            Next
        Catch ex As Exception
            MsgBox(ex.Message)
        End Try
        CloseAll()
    End Sub

    Private Sub ButtonTCPSend_Click(ByVal sender As System.Object, ByVal e As System.EventArgs) Handles ButtonTCPSend.Click
        Dim pUser As PMDPeripheral
        ' pUser is used to exchange data with the UserTCP CME application.
        Try
            pUser = New PMDPeripheralTCP(System.Net.IPAddress.Parse("192.168.2.2"), 40104, 1000)
            Dim BytesOut As Byte()
            Dim BytesIn(0 To 100) As Byte
            Dim nReceived As UInteger
            BytesOut = Encoding.ASCII.GetBytes(TCPMessage.Text)
            pUser.Send(BytesOut, BytesOut.Length, TCPTimeout)
            pUser.Receive(BytesIn, nReceived, BytesIn.Length, TCPTimeout)
            TCPReply.Text = Encoding.ASCII.GetString(BytesIn, 0, nReceived)
            pUser.Close()
        Catch ex As Exception
            MsgBox(ex.Message)
        End Try
    End Sub
    Private Sub CAN_Click(ByVal sender As System.Object, ByVal e As System.EventArgs) Handles CAN.Click
        Dim major, minor As UInt32
        Dim MPmajor, MPminor, NumberAxes, special, custom, family As UInt16
        Dim MotorType As PMDMotorTypeVersion
        Dim value As UInt32

        ' NODE 2 = out: Hex602 in: Hex582 
        Dim transmitID1 As UInteger = &H600
        Dim recieveID1 As UInteger = &H580
        Dim eventID1 As UInteger = &H150
        Try
            periph1 = New PMDPeripheralCAN(transmitID1, recieveID1, eventID1)
            periph2 = New PMDPeripheralMultiDrop(periph1, 2) 'nodeID=2

            ' -----Device pointer--------
            device1 = New PMDDevice(periph1, DeviceType) 'Starts as Resource Protocol, ours needs to be MotionProcessor
            device2 = New PMDDevice(periph2, DeviceType) 'Starts as Resource Protocol, ours needs to be MotionProcessor

            ' -----Axis pointer----------
            axis1 = New PMDAxis(device1, PMDAxisNumber.Axis1)
            axis2 = New PMDAxis(device2, PMDAxisNumber.Axis1)

            axis1.GetVersion(family, MotorType, NumberAxes, special, custom, MPmajor, MPminor)
            LabelVersionCOM.Text = "version: " & major & "." & minor.ToString("00") & " Magellan " & MPmajor & "." & MPminor

            value = axis1.Time
            ListBox1.Items.Add(value)
            value = axis2.Time
            ListBox1.Items.Add(value)

        Catch ex As Exception
            MsgBox(ex.Message)
        End Try
        CloseAll()

    End Sub
End Class
