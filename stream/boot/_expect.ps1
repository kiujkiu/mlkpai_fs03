# 串口 expect: 自动应答 login/Password, 到 shell 后执行 -Cmd, 打印到 ___DONE___
param([string]$Port='COM13', [string]$User='uisrc', [string]$Pass='root',
      [string]$Cmd='echo hi', [int]$TimeoutS=45)
$sp = New-Object System.IO.Ports.SerialPort $Port,115200,([System.IO.Ports.Parity]::None),8,([System.IO.Ports.StopBits]::One)
$sp.ReadTimeout = 200
$sp.Open()
Start-Sleep -Milliseconds 300
try { $null = $sp.ReadExisting() } catch {}
$sp.Write("`n")
$buf = ''
$deadline = (Get-Date).AddSeconds($TimeoutS)
$stage = 'login'
while ((Get-Date) -lt $deadline) {
    Start-Sleep -Milliseconds 150
    try { $buf += $sp.ReadExisting() } catch {}
    if ($stage -eq 'login') {
        if ($buf -match 'login:\s*$')    { $sp.Write("$User`n"); $buf = ''; continue }
        if ($buf -match 'Password:\s*$') { $sp.Write("$Pass`n"); $buf = ''; $stage = 'shell'; continue }
        if ($buf -match '\$\s*$')        { $stage = 'ready' }
    }
    elseif ($stage -eq 'shell') {
        if ($buf -match 'Login incorrect') { $stage = 'login'; $buf = ''; $sp.Write("`n"); continue }
        if ($buf -match '\$\s*$')          { $stage = 'ready' }
    }
    if ($stage -eq 'ready') {
        $sp.Write("$Cmd; echo ___DO`"`"NE___`n")   # 拆开写防命令回显误匹配
        $buf = ''
        while ((Get-Date) -lt $deadline) {
            Start-Sleep -Milliseconds 200
            try { $buf += $sp.ReadExisting() } catch {}
            if ($buf -match '___DONE___') { break }
        }
        Write-Output $buf
        $sp.Close()
        exit 0
    }
}
Write-Output "TIMEOUT stage=$stage buf_tail: $($buf.Substring([Math]::Max(0,$buf.Length-300)))"
$sp.Close()
exit 1
