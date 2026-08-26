# TF card preparation for MLKPAI-FS03 (Zynq7020) - STEP 1 of 2 (boot partition)
#
#   powershell -File prepare_card.ps1                     # inspect only
#   powershell -File prepare_card.ps1 -Drive F -Write     # format FAT32 + write boot files
#
# WARNING: -Write ERASES the FAT partition. Refuses non-removable volumes.
#
# ASCII-only on purpose: Windows PowerShell 5.1 reads .ps1 as ANSI (GBK here),
# so non-ASCII in this file breaks the parser.
#
# STEP 2 (rootfs -> ext4) CANNOT be done from Windows. See MAKE_CARD_FROM_BLANK.md:
# run `wsl --mount \\.\PHYSICALDRIVE<N> --bare` in an ADMIN PowerShell, then Linux side
# does partitioning + mkfs.ext4 + untar.
param(
    [string]$Drive = "",
    [switch]$Write
)

$SRC = "D:\claude_workspace\pov3d\mlkpai_fs03\stream\boot"

# name-on-card = source path
$FILES = @{
    "BOOT.BIN"       = "$SRC\BOOT.BIN"
    "uImage"         = "$SRC\board_backup\uImage.6.6.0-xilinx"
    "devicetree.dtb" = "$SRC\board_backup\devicetree.dtb"
    "uEnv.txt"       = "$SRC\uEnv.txt"
}

Write-Output "=== Removable volumes ==="
$rem = Get-Volume | Where-Object { $_.DriveType -eq 'Removable' -and $_.DriveLetter }
if (-not $rem) { Write-Output "NONE - no card in reader" }
else {
    $rem | Select-Object DriveLetter,FileSystemLabel,FileSystem,
        @{n='SizeGB';e={[math]::Round($_.Size/1GB,2)}},
        @{n='FreeGB';e={[math]::Round($_.SizeRemaining/1GB,2)}} |
        Format-Table -AutoSize | Out-String | Write-Output
}

Write-Output "=== Source files ==="
$missing = $false
foreach ($k in $FILES.Keys) {
    $p = $FILES[$k]
    if (Test-Path $p) {
        $h = (Get-FileHash $p -Algorithm MD5).Hash.ToLower()
        Write-Output ("  {0,-16} {1,10} {2}" -f $k, (Get-Item $p).Length, $h.Substring(0,12))
    } else { Write-Output "  $k  MISSING: $p"; $missing = $true }
}
if ($missing) { Write-Output "ABORT: source files missing"; exit 1 }

if (-not $Drive) { Write-Output ""; Write-Output "(inspect mode; to write: -Drive <letter> -Write)"; exit 0 }

$d = $Drive.TrimEnd(':')
$vol = Get-Volume -DriveLetter $d -ErrorAction SilentlyContinue
if (-not $vol)                      { Write-Output "ABORT: drive $d not found"; exit 1 }
if ($vol.DriveType -ne 'Removable') { Write-Output "ABORT: $d is $($vol.DriveType), not Removable - refusing"; exit 1 }

Write-Output ""
Write-Output "=== Current contents of ${d}: (would be erased) ==="
Get-ChildItem "${d}:\" -Force -ErrorAction SilentlyContinue |
    Select-Object Length,Name | Format-Table -AutoSize | Out-String | Write-Output

if (-not $Write) { Write-Output "(no -Write given; nothing changed)"; exit 0 }

Write-Output "=== Formatting ${d}: as FAT32 (label BOOT) ==="
Format-Volume -DriveLetter $d -FileSystem FAT32 -NewFileSystemLabel "BOOT" -Confirm:$false -Force | Out-Null
if (-not $?) { Write-Output "ABORT: format failed"; exit 1 }

Write-Output "=== Copying boot files ==="
foreach ($k in $FILES.Keys) { Copy-Item $FILES[$k] "${d}:\$k" -Force }

Write-Output "=== Verify (MD5 card vs source) ==="
$ok = $true
foreach ($k in $FILES.Keys) {
    $a = (Get-FileHash $FILES[$k] -Algorithm MD5).Hash.ToLower()
    $b = (Get-FileHash "${d}:\$k" -Algorithm MD5).Hash.ToLower()
    if ($a -eq $b) { $m = "OK " } else { $ok = $false; $m = "BAD" }
    Write-Output "$m $k  $a"
}
# u-boot's fpga_loadbit would OVERWRITE the PL that FSBL just programmed
if (Test-Path "${d}:\system.bit.bin") { Write-Output "WARNING: system.bit.bin present - rename it, see SD_CARD_GUIDE.md" }

if ($ok) {
    Write-Output ""
    Write-Output "BOOT PARTITION DONE - but the card is NOT bootable yet:"
    Write-Output "  rootfs (ext4, 559MB) still missing. See MAKE_CARD_FROM_BLANK.md step 3."
} else { Write-Output "VERIFY FAILED - do not use this card" }
