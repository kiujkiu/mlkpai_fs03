Get-CimInstance Win32_Process -Filter "name='python.exe'" | ForEach-Object {
    if ($_.CommandLine -like '*frames_rgb*' -or $_.CommandLine -like '*frames_diagdisk*') {
        Write-Output ("KILL " + $_.ProcessId)
        Stop-Process -Id $_.ProcessId -Force
    }
}
Write-Output "DONE"
