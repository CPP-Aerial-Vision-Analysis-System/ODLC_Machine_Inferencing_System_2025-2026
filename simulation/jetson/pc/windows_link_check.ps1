<#
windows_link_check.ps1 -- PURPOSE: the WINDOWS side's view of the link to the Jetson, READ-ONLY. The simulator runs in
WSL (Ubuntu), which sits behind Windows like a home network behind a router: when WSL cannot reach the Jetson, the
first question is "can Windows itself reach it?". This script answers that, for `jetson.sh check` step 1
(docs/jetson/HIL_DESIGN.md 9.2 row 1), and changes NOTHING (no driver, adapter, address or firewall setting).

It reports, as KEY=VALUE lines (one per line, easy for the shell to read):
  USB_ADAPTER=yes|no          a "Remote NDIS" / "USB Ethernet" network adapter exists (the Jetson's USB-C network)
  USB_ADAPTER_NAME=...        its name, USB_ADAPTER_STATUS=Up|Disconnected|..., USB_ADAPTER_IP=192.168.55.100 (or "")
  USB_UNKNOWN_DEVICE=yes|no   a device without a driver that looks like the Jetson (Device Manager "Other devices")
  USB_SERIAL_PORT=COM5|""     the Jetson's USB serial console (the rescue console, PuTTY 115200 8N1)
  PING=ok|fail                Windows' own ping to HOST
  TCP22=ok|fail               Windows' own TCP connection to HOST port PORT (SSH)
  HINT=...                    one line in plain words: what to do next

HOW TO RUN (from WSL, what jetson.sh check does):
  powershell.exe -NoProfile -ExecutionPolicy Bypass -File "$(wslpath -w simulation/jetson/pc/windows_link_check.ps1)" -Target 192.168.55.1
From Windows PowerShell:
  powershell -NoProfile -ExecutionPolicy Bypass -File simulation\jetson\pc\windows_link_check.ps1 -Target 192.168.55.1 -Port 22
Exit code: 0 always when it could run (the answers are in the lines), 2 when typed wrong.
#>
param(
    [string]$Target = "192.168.55.1",
    [int]$Port = 22
)

function Out-KV([string]$Key, [string]$Value) { Write-Output ("{0}={1}" -f $Key, ($Value -replace "[\r\n]", " ")) }

if ($Port -lt 1 -or $Port -gt 65535) { Write-Output "[windows_link_check] ERROR: -Port must be 1-65535"; exit 2 }

# 1. the USB network adapter (Windows' own RNDIS driver names it "Remote NDIS Compatible Device")
$usb = $null
try {
    $usb = Get-NetAdapter -IncludeHidden -ErrorAction Stop |
        Where-Object { $_.InterfaceDescription -match "Remote NDIS|RNDIS|USB Ethernet|Linux USB Ethernet|NVIDIA" } |
        Select-Object -First 1
} catch { $usb = $null }
if ($usb) {
    Out-KV "USB_ADAPTER" "yes"
    Out-KV "USB_ADAPTER_NAME" ("{0} ({1})" -f $usb.Name, $usb.InterfaceDescription)
    Out-KV "USB_ADAPTER_STATUS" $usb.Status
    $ip = ""
    try {
        $ip = (Get-NetIPAddress -InterfaceIndex $usb.ifIndex -AddressFamily IPv4 -ErrorAction Stop |
               Select-Object -First 1).IPAddress
    } catch { $ip = "" }
    Out-KV "USB_ADAPTER_IP" $ip
} else {
    Out-KV "USB_ADAPTER" "no"
    Out-KV "USB_ADAPTER_NAME" ""
    Out-KV "USB_ADAPTER_STATUS" ""
    Out-KV "USB_ADAPTER_IP" ""
}

# 2. a device without a driver (Device Manager, "Other devices"), and the USB serial console
$unknown = "no"
$com = ""
try {
    $bad = Get-PnpDevice -PresentOnly -ErrorAction Stop |
        Where-Object { $_.Status -eq "Error" -and ($_.FriendlyName -match "RNDIS|NDIS|Jetson|NVIDIA|L4T|USB Ethernet" -or
                                                   $_.InstanceId -match "VID_0955") }
    if ($bad) { $unknown = "yes" }
    $port = Get-PnpDevice -PresentOnly -Class Ports -ErrorAction Stop |
        Where-Object { $_.InstanceId -match "VID_0955" -or $_.FriendlyName -match "USB Serial Device" } | Select-Object -First 1
    if ($port -and $port.FriendlyName -match "\((COM\d+)\)") { $com = $Matches[1] }
} catch { }
Out-KV "USB_UNKNOWN_DEVICE" $unknown
Out-KV "USB_SERIAL_PORT" $com

# 3. can Windows itself reach the Jetson? (ping, then TCP to the SSH port, 3 s each)
$ping = "fail"
try { if (Test-Connection -ComputerName $Target -Count 1 -Quiet -ErrorAction Stop) { $ping = "ok" } } catch { }
Out-KV "PING" $ping
$tcp = "fail"
try {
    $client = New-Object System.Net.Sockets.TcpClient
    $iar = $client.BeginConnect($Target, $Port, $null, $null)
    if ($iar.AsyncWaitHandle.WaitOne(3000, $false) -and $client.Connected) { $tcp = "ok" }
    $client.Close()
} catch { }
Out-KV "TCP22" $tcp

# 4. one line of advice
if ($tcp -eq "ok") {
    $hint = "Windows reaches ${Target} on TCP ${Port}. If WSL does not, see docs/jetson/LINK.md 'Windows reaches it but Ubuntu does not'."
} elseif ($Target -eq "192.168.55.1" -and -not $usb -and $unknown -eq "yes") {
    $hint = "The Jetson is plugged in but its USB network has no driver: Device Manager > Other devices > right-click > Update driver > Browse my computer > Let me pick > Network adapters > Microsoft > Remote NDIS Compatible Device (docs/jetson/LINK.md)."
} elseif ($Target -eq "192.168.55.1" -and -not $usb) {
    $hint = "No USB network adapter: use the Jetson's USB-C port (not a USB-A port) with a DATA cable, power the Jetson from its barrel jack, wait 1 minute after boot."
} elseif ($Target -eq "192.168.55.1" -and $usb.Status -ne "Up") {
    $hint = "The USB network adapter exists but is '$($usb.Status)'; unplug and plug the USB-C cable again, or wait until the Jetson has finished booting."
} elseif ($ping -eq "ok") {
    $hint = "${Target} answers ping but not TCP ${Port}: the Jetson's SSH server is not running yet (wait for boot) or a firewall blocks port ${Port}."
} else {
    $hint = "Windows cannot reach ${Target}: check the address (jetson.sh find), the cable / Wi-Fi, and that both are on the same network (campus Wi-Fi often blocks device-to-device traffic)."
}
Out-KV "HINT" $hint
exit 0
