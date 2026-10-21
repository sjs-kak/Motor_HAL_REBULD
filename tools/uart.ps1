<#
    串口调试小工具 —— 给开发者在本机终端里用，不参与固件编译、不被 CMake 引用。

    为什么要有这个脚本：
      本工程的固件协议要求一帧以 CR LF（0x0D 0x0A）结尾（见 Service/Src/protocol.c 的
      帧状态机），很多串口助手默认只发 LF 或什么都不发，结果就是"命令发出去了但板子没反应"。
      这里统一补 CRLF；急停 E 是单字节命令，不能补 CRLF，所以单独给 -Estop。

    用法（在工程根目录的终端里）：
      powershell -ExecutionPolicy Bypass -File tools\uart.ps1 -List
      powershell -ExecutionPolicy Bypass -File tools\uart.ps1 -Port COM22 -Send "?"
      powershell -ExecutionPolicy Bypass -File tools\uart.ps1 -Port COM22 -Send "F1000"
      powershell -ExecutionPolicy Bypass -File tools\uart.ps1 -Port COM22 -Estop
      powershell -ExecutionPolicy Bypass -File tools\uart.ps1 -Port COM22 -Send "?" -ListenMs 1500

    命令速查（详见 README §4）：
      ?            查询状态，回 SPD L=.. R=.. ACC=.. [ESTOP] TXDROP=..
      H / F<Hz>    左/右轮转动，F 是右轮
      L / A<..>    左轮 / 加速度
      E            急停（单字节，不需要回车）
      D            演示动作（约 12 秒，调试档最高 1200 Hz）
#>
[CmdletBinding(DefaultParameterSetName = 'Send')]
param(
    [string] $Port = 'COM22',
    [int]    $Baud = 115200,

    [Parameter(ParameterSetName = 'Send', Mandatory = $true)]
    [string] $Send,

    [Parameter(ParameterSetName = 'Stop', Mandatory = $true)]
    [switch] $Estop,

    [Parameter(ParameterSetName = 'List', Mandatory = $true)]
    [switch] $List,

    [int] $ListenMs = 800
)

function Show-Ports {
    [System.IO.Ports.SerialPort]::GetPortNames() | Sort-Object | ForEach-Object {
        Write-Host ('  ' + $_) -ForegroundColor Cyan
    }
}

if ($List) {
    Write-Host '可用串口：'
    Show-Ports
    return
}

$sp = New-Object System.IO.Ports.SerialPort -ArgumentList @(
    $Port, $Baud, [System.IO.Ports.Parity]::None, 8, [System.IO.Ports.StopBits]::One
)
$sp.ReadTimeout = 200
$sp.WriteTimeout = 500
$sp.DtrEnable = $false
$sp.RtsEnable = $false

try {
    $sp.Open()
} catch {
    Write-Host ('打不开 ' + $Port + '：' + $_.Exception.Message) -ForegroundColor Red
    Write-Host '常见原因：' -ForegroundColor Yellow
    Write-Host '  1) 别的程序占着这个口（另一个 VS Code 窗口的串口监视器最常见）'
    Write-Host '  2) 串口号写错了，用 -List 看看'
    Write-Host '  3) 接收器没插好 —— 拔插一次再试'
    return
}

try {
    Start-Sleep -Milliseconds 150
    $sp.DiscardInBuffer()

    if ($Estop) {
        # 急停是单字节，不带 CRLF：App_RxByte 收到 'E' 立刻置 abort
        $sp.Write([byte[]]@(0x45), 0, 1)
        Write-Host '已发送急停 E（单字节）' -ForegroundColor Yellow
    }
    else {
        $payload = $Send + "`r`n"
        $bytes = [System.Text.Encoding]::ASCII.GetBytes($payload)
        $sp.Write($bytes, 0, $bytes.Length)
        Write-Host ('已发送：' + $Send + ' + CRLF') -ForegroundColor Yellow
    }

    $sp.BaseStream.Flush()
    Start-Sleep -Milliseconds $ListenMs

    $n = $sp.BytesToRead
    if ($n -gt 0) {
        $buf = New-Object byte[] $n
        $sp.Read($buf, 0, $n) | Out-Null
        $text = [System.Text.Encoding]::ASCII.GetString($buf)
        Write-Host ('收到 ' + $n + ' 字节：') -ForegroundColor Green
        Write-Host ($text.TrimEnd("`r", "`n"))
        Write-Host ('HEX: ' + (($buf | ForEach-Object { $_.ToString('X2') }) -join ' ')) -ForegroundColor DarkGray
    }
    else {
        Write-Host ('等 ' + $ListenMs + ' ms 没有回包。') -ForegroundColor Red
        Write-Host '排查顺序：' -ForegroundColor Yellow
        Write-Host '  1) 板子上电了吗（LED 亮不亮）'
        Write-Host '  2) 接收器的 TX 接到 PA10 了吗、GND 共地了吗（PA9=TX PA10=RX）'
        Write-Host '  3) 固件是不是这次编译的（H 命令会回 OK，说明主循环活着）'
        Write-Host '  4) 波特率 115200 对不对'
    }
}
finally {
    if ($sp.IsOpen) { $sp.Close() }
    $sp.Dispose()
}
