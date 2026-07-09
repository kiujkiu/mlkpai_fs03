# Serial file upload via base64 chunks. Board must be at a logged-in shell.
param([string]$Src, [string]$Dest, [string]$Port='COM13', [int]$ChunkB=2000)
if (-not (Test-Path $Src)) { Write-Output "SRC_NOT_FOUND: [$Src]"; exit 1 }
$bytes = [System.IO.File]::ReadAllBytes($Src)
$ms = New-Object System.IO.MemoryStream
$gz = New-Object System.IO.Compression.GZipStream($ms, [System.IO.Compression.CompressionLevel]::Optimal)
$gz.Write($bytes, 0, $bytes.Length); $gz.Close()
$b64 = [Convert]::ToBase64String($ms.ToArray())
Write-Output "src=$($bytes.Length)B gz+b64=$($b64.Length)B chunks=$([math]::Ceiling($b64.Length/$ChunkB))"
$sp = New-Object System.IO.Ports.SerialPort $Port,115200,([System.IO.Ports.Parity]::None),8,([System.IO.Ports.StopBits]::One)
$sp.ReadTimeout = 300
$sp.Open()
$sp.Write("`n"); Start-Sleep -Milliseconds 400
try { $null = $sp.ReadExisting() } catch {}
$sp.Write("stty -echo; rm -f /tmp/_up.b64`n"); Start-Sleep -Milliseconds 400
try { $null = $sp.ReadExisting() } catch {}
for ($i = 0; $i -lt $b64.Length; $i += $ChunkB) {
    $chunk = $b64.Substring($i, [Math]::Min($ChunkB, $b64.Length - $i))
    $sp.Write("echo $chunk >> /tmp/_up.b64`n")
    Start-Sleep -Milliseconds ([Math]::Max(300, $chunk.Length/4))
    try { $null = $sp.ReadExisting() } catch {}
    if (($i / $ChunkB) % 20 -eq 0) { Write-Output "sent $i/$($b64.Length)" }
}
$sp.Write("stty echo; base64 -d /tmp/_up.b64 | gunzip > $Dest && md5sum $Dest && echo UPLOAD_DONE`n")
$deadline = (Get-Date).AddSeconds(30); $buf = ''
while ((Get-Date) -lt $deadline) {
    Start-Sleep -Milliseconds 300
    try { $buf += $sp.ReadExisting() } catch {}
    if ($buf -match 'UPLOAD_DONE') { break }
}
Write-Output $buf
$sp.Close()
