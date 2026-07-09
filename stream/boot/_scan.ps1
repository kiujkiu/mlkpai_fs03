# Async ping sweep 10.10.20-21.x then find board MAC in ARP table
$tasks = @()
foreach ($sub in 20,21) {
  foreach ($i in 1..254) {
    $p = New-Object System.Net.NetworkInformation.Ping
    $tasks += ,@($p.SendPingAsync("10.10.$sub.$i", 700))
  }
}
[System.Threading.Tasks.Task]::WaitAll(($tasks | ForEach-Object { $_[0] }))
$hits = arp -a | Select-String -Pattern '90-de-80-35-1c-47'
if ($hits) { Write-Output "BOARD_AT: $hits" } else { Write-Output "MAC_NOT_FOUND" }
