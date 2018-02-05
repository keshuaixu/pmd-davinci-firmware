<Global.Microsoft.VisualBasic.CompilerServices.DesignerGenerated()> _
Partial Class Form1
    Inherits System.Windows.Forms.Form

    'Form overrides dispose to clean up the component list.
    <System.Diagnostics.DebuggerNonUserCode()> _
    Protected Overrides Sub Dispose(ByVal disposing As Boolean)
        If disposing AndAlso components IsNot Nothing Then
            components.Dispose()
        End If
        MyBase.Dispose(disposing)
    End Sub

    'Required by the Windows Form Designer
    Private components As System.ComponentModel.IContainer

    'NOTE: The following procedure is required by the Windows Form Designer
    'It can be modified using the Windows Form Designer.  
    'Do not modify it using the code editor.
    <System.Diagnostics.DebuggerStepThrough()> _
    Private Sub InitializeComponent()
        Me.ButtonCOM = New System.Windows.Forms.Button()
        Me.LabelVersionCOM = New System.Windows.Forms.Label()
        Me.LabelVersionTCP = New System.Windows.Forms.Label()
        Me.ButtonTCP = New System.Windows.Forms.Button()
        Me.LabelDefaultsCOM = New System.Windows.Forms.Label()
        Me.TCPMessage = New System.Windows.Forms.TextBox()
        Me.TCPReply = New System.Windows.Forms.Label()
        Me.ListBox1 = New System.Windows.Forms.ListBox()
        Me.ButtonTCPSend = New System.Windows.Forms.Button()
        Me.CAN = New System.Windows.Forms.Button()
        Me.SuspendLayout()
        '
        'ButtonCOM
        '
        Me.ButtonCOM.Location = New System.Drawing.Point(0, 16)
        Me.ButtonCOM.Name = "ButtonCOM"
        Me.ButtonCOM.Size = New System.Drawing.Size(75, 23)
        Me.ButtonCOM.TabIndex = 0
        Me.ButtonCOM.Text = "COM"
        Me.ButtonCOM.UseVisualStyleBackColor = True
        '
        'LabelVersionCOM
        '
        Me.LabelVersionCOM.AutoSize = True
        Me.LabelVersionCOM.Location = New System.Drawing.Point(8, 64)
        Me.LabelVersionCOM.Name = "LabelVersionCOM"
        Me.LabelVersionCOM.Size = New System.Drawing.Size(39, 13)
        Me.LabelVersionCOM.TabIndex = 1
        Me.LabelVersionCOM.Text = "Label1"
        '
        'LabelVersionTCP
        '
        Me.LabelVersionTCP.AutoSize = True
        Me.LabelVersionTCP.Location = New System.Drawing.Point(8, 200)
        Me.LabelVersionTCP.Name = "LabelVersionTCP"
        Me.LabelVersionTCP.Size = New System.Drawing.Size(39, 13)
        Me.LabelVersionTCP.TabIndex = 2
        Me.LabelVersionTCP.Text = "Label1"
        '
        'ButtonTCP
        '
        Me.ButtonTCP.Location = New System.Drawing.Point(0, 161)
        Me.ButtonTCP.Name = "ButtonTCP"
        Me.ButtonTCP.Size = New System.Drawing.Size(75, 23)
        Me.ButtonTCP.TabIndex = 3
        Me.ButtonTCP.Text = "Ethernet"
        Me.ButtonTCP.UseVisualStyleBackColor = True
        '
        'LabelDefaultsCOM
        '
        Me.LabelDefaultsCOM.AutoSize = True
        Me.LabelDefaultsCOM.Location = New System.Drawing.Point(8, 95)
        Me.LabelDefaultsCOM.Name = "LabelDefaultsCOM"
        Me.LabelDefaultsCOM.Size = New System.Drawing.Size(39, 13)
        Me.LabelDefaultsCOM.TabIndex = 5
        Me.LabelDefaultsCOM.Text = "Label1"
        '
        'TCPMessage
        '
        Me.TCPMessage.Location = New System.Drawing.Point(16, 248)
        Me.TCPMessage.Name = "TCPMessage"
        Me.TCPMessage.Size = New System.Drawing.Size(104, 20)
        Me.TCPMessage.TabIndex = 8
        '
        'TCPReply
        '
        Me.TCPReply.AutoSize = True
        Me.TCPReply.Location = New System.Drawing.Point(16, 280)
        Me.TCPReply.Name = "TCPReply"
        Me.TCPReply.Size = New System.Drawing.Size(55, 13)
        Me.TCPReply.TabIndex = 9
        Me.TCPReply.Text = "TCPReply"
        '
        'ListBox1
        '
        Me.ListBox1.FormattingEnabled = True
        Me.ListBox1.Location = New System.Drawing.Point(212, 51)
        Me.ListBox1.Name = "ListBox1"
        Me.ListBox1.Size = New System.Drawing.Size(120, 95)
        Me.ListBox1.TabIndex = 4
        '
        'ButtonTCPSend
        '
        Me.ButtonTCPSend.Location = New System.Drawing.Point(160, 248)
        Me.ButtonTCPSend.Name = "ButtonTCPSend"
        Me.ButtonTCPSend.Size = New System.Drawing.Size(75, 23)
        Me.ButtonTCPSend.TabIndex = 10
        Me.ButtonTCPSend.Text = "TCP Send"
        Me.ButtonTCPSend.UseVisualStyleBackColor = True
        '
        'CAN
        '
        Me.CAN.Location = New System.Drawing.Point(0, 123)
        Me.CAN.Name = "CAN"
        Me.CAN.Size = New System.Drawing.Size(75, 23)
        Me.CAN.TabIndex = 11
        Me.CAN.Text = "CAN"
        Me.CAN.UseVisualStyleBackColor = True
        '
        'Form1
        '
        Me.AutoScaleDimensions = New System.Drawing.SizeF(6.0!, 13.0!)
        Me.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font
        Me.ClientSize = New System.Drawing.Size(558, 595)
        Me.Controls.Add(Me.CAN)
        Me.Controls.Add(Me.ButtonTCPSend)
        Me.Controls.Add(Me.TCPReply)
        Me.Controls.Add(Me.TCPMessage)
        Me.Controls.Add(Me.LabelDefaultsCOM)
        Me.Controls.Add(Me.ListBox1)
        Me.Controls.Add(Me.ButtonTCP)
        Me.Controls.Add(Me.LabelVersionTCP)
        Me.Controls.Add(Me.LabelVersionCOM)
        Me.Controls.Add(Me.ButtonCOM)
        Me.Name = "Form1"
        Me.Text = "Form1"
        Me.ResumeLayout(False)
        Me.PerformLayout()

    End Sub
    Friend WithEvents ButtonCOM As System.Windows.Forms.Button
    Friend WithEvents LabelVersionCOM As System.Windows.Forms.Label
    Friend WithEvents LabelVersionTCP As System.Windows.Forms.Label
    Friend WithEvents ButtonTCP As System.Windows.Forms.Button
    Friend WithEvents LabelDefaultsCOM As System.Windows.Forms.Label
    Friend WithEvents TCPMessage As System.Windows.Forms.TextBox
    Friend WithEvents TCPReply As System.Windows.Forms.Label
    Friend WithEvents ListBox1 As System.Windows.Forms.ListBox
    Friend WithEvents ButtonTCPSend As System.Windows.Forms.Button
    Friend WithEvents CAN As System.Windows.Forms.Button

End Class
