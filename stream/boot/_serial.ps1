# COM13 串口批命令: powershell -File _serial.ps1 -Script "cmd1||cmd2||cmd3" [-WaitMs 1200]
param([string]$Port='COM13', [string]$Script='', [int]$WaitMs=1200)
$sp = New-Object System.IO.Ports.SerialPort $Port,115200,([System.IO.Ports.Parity]::None),8,([System.IO.Ports.StopBits]::One)
$sp.NewLine = "`n"
$sp.ReadTimeout = 300
$sp.Open()
Start-Sleep -Milliseconds 200
try { $junk = $sp.ReadExisting() } catch {}
foreach ($l in ($Script -split '\|\|')) {
    $sp.Write("$l`n")
    Start-Sleep -Milliseconds $WaitMs
    try { $out = $sp.ReadExisting(); Write-Output $out } catch {}
}
Start-Sleep -Milliseconds 500
try { Write-Output $sp.ReadExisting() } catch {}
$sp.Close()
