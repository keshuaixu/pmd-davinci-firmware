Imports System
Imports System.Net.Sockets
Imports System.Text
Imports System.Threading


' Magellan event data
Public Class PMDEventArgs
    Inherits EventArgs
    Private axis As Integer
    Private data As Integer
    Public Sub New(axis As Integer, eventdata As Integer)
        Me.axis = axis
        Me.data = eventdata
    End Sub

    Public ReadOnly Property EventAxis() As Integer
        Get
            Return axis
        End Get
    End Property
    Public ReadOnly Property EventData() As Integer
        Get
            Return data
        End Get
    End Property
End Class


Public Class PMDMagellanEventHandler
    Inherits PMDEventHandler

    Public Sub New()
        Me.TCPport = 40200
    End Sub
End Class


Public Class PMDIOEventHandler
    Inherits PMDEventHandler

    Public Sub New()
        Me.TCPport = 40300
    End Sub
End Class

' Delegate declaration.
Public Delegate Sub PMDEventHandlerDelegate(sender As Object, e As PMDEventArgs)

Public Class PMDEventHandler
    Private client As TcpClient
    Private stream As NetworkStream
    Private allDone As New ManualResetEvent(True)
    Private newThread As New System.Threading.Thread(AddressOf ReadThread)
    Protected TCPport As Integer

    Public Event PMDEvent As PMDEventHandlerDelegate

    Protected Overridable Sub OnPMDEvent(e As PMDEventArgs)
        RaiseEvent PMDEvent(Me, e)
    End Sub

    Public Sub Close()
        If Not IsNothing(stream) Then
            stream.Close()
        End If
        If Not IsNothing(client) Then
            client.Close()
        End If
        allDone.WaitOne(1000)
    End Sub

    Public Sub Connect(ipaddress As String)
        Dim data(10) As Byte
        client = New TcpClient
        client.Connect(ipaddress, TCPport)
        stream = client.GetStream()
        allDone.Reset()
        newThread.Start()
    End Sub
    ' Thread that waits for incoming TCP data.
    ' it ends the TCP connection is closed 
    Public Sub ReadThread()
        Dim ReadBuffer(4) As Byte
        Dim numberOfBytesRead As Integer
        Dim axis As Integer
        Dim eventdata As Integer

        Try
            stream.ReadTimeout = -1
            While True
                numberOfBytesRead = stream.Read(ReadBuffer, 0, 4)
                If (numberOfBytesRead = 4) Then
                    axis = ReadBuffer(1)
                    eventdata = CInt(ReadBuffer(2)) << 8
                    eventdata += ReadBuffer(3)
                    Dim e As New PMDEventArgs(axis, eventdata)
                    OnPMDEvent(e)
                End If
            End While
        Catch e As Exception
            'Console.WriteLine(e.Message)
        End Try
        allDone.Set()
    End Sub

End Class

