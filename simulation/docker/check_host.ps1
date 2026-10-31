# =============================================================================
#  check_host.ps1  --  the WINDOWS half of the Docker "doctor" (simulation/docker/check_host.sh is the Linux half)
#
#  PURPOSE: before a teammate's first Docker build on Windows, look at the things only Windows can see and that
#  stop everything later: virtualization switched off in the BIOS, WSL missing or old (no WSLg windows), the
#  Ubuntu-22.04 distro missing or on WSL version 1, Docker Desktop installed but not running or without WSL
#  integration for Ubuntu-22.04, too little memory for WSL, a full disk, Git's core.autocrlf=true having written
#  Windows line endings (CRLF) into the scripts of this checkout, a checkout inside OneDrive, a wrong clock, a
#  proxy. For every problem it prints what is wrong and EXACTLY what to do. It changes nothing: no setting, no
#  file, no program is started (it does not start Docker Desktop or a WSL distro).
#
#  HOW TO RUN (PowerShell, from the repository folder):
#      powershell -ExecutionPolicy Bypass -File simulation\docker\check_host.ps1
#      powershell -ExecutionPolicy Bypass -File simulation\docker\check_host.ps1 -Offline      (no internet checks)
#  ("-ExecutionPolicy Bypass" only allows this one run of a script; it changes no setting.)
#  Exit code: 0 = nothing on the Windows side blocks Docker (warnings may remain), 1 = a [FAIL] to fix first.
#  Next, inside Ubuntu:  wsl -d Ubuntu-22.04   then   bash simulation/docker/check_host.sh
#  Every message is explained in docs/simulation/DOCKER.md, "Troubleshooting".
# =============================================================================
param([switch]$Offline)

$script:Fails = 0
$script:Warns = 0
function Pass([string]$m)    { Write-Output "  [ OK ]  $m" }
function Note([string]$m)    { Write-Output "  [ .. ]  $m" }
function Caution([string]$m) { Write-Output "  [WARN]  $m"; $script:Warns++ }
function Fail([string]$m)    { Write-Output "  [FAIL]  $m"; $script:Fails++ }
function Tip([string]$m)     { Write-Output "          -> $m" }
function Title([string]$m)   { Write-Output ""; Write-Output $m }

$Repo = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
$Distro = "Ubuntu-22.04"

Write-Output "ASTRA Docker check, Windows side (changes nothing; every problem says what to do)"
Write-Output "------------------------------------------------------------------------------------------"

# ---- 1. Windows, virtualization, memory, disk ---------------------------------------------------------------
Title "1. This Windows PC"
$build = [System.Environment]::OSVersion.Version.Build
if ($build -ge 22000) { Pass "Windows 11 (build $build): WSL2 with WSLg windows works" }
elseif ($build -ge 19044) { Note "Windows 10 (build $build): works with the Microsoft Store WSL (wsl --update gives WSLg windows)" }
else { Caution "Windows build $build is old: WSL2 needs Windows 10 21H2 or newer, Linux windows (WSLg) need Windows 11"; Tip "install the Windows updates (Settings -> Windows Update)" }

$cs = $null; try { $cs = Get-CimInstance Win32_ComputerSystem -ErrorAction Stop } catch { }
$cpu = $null; try { $cpu = Get-CimInstance Win32_Processor -ErrorAction Stop | Select-Object -First 1 } catch { }
if ($cs -and $cs.HypervisorPresent) { Pass "virtualization is on (a hypervisor runs: WSL2 / Docker can start)" }
elseif ($cpu -and ($cpu.VirtualizationFirmwareEnabled -eq $false)) {
  Fail "virtualization is switched OFF in the BIOS/UEFI: WSL2 and Docker cannot start"
  Tip "restart, open the BIOS/UEFI setup (often F2, F10, Del or Esc while starting) and enable 'Intel VT-x' / 'AMD-V (SVM)'"
} else { Note "could not tell whether virtualization is on (WSL2 says so when it starts)" }
if ($cpu) {
  if ($env:PROCESSOR_ARCHITECTURE -ne "AMD64") { Caution "processor type $($env:PROCESSOR_ARCHITECTURE): the simulator image needs an Intel/AMD (x64) PC" }
}

$totalGB = 0
if ($cs) { $totalGB = [math]::Round($cs.TotalPhysicalMemory / 1GB, 1) }
$wslcfg = Join-Path $env:USERPROFILE ".wslconfig"
$wslMemGB = [math]::Round($totalGB / 2, 1); $memSource = "half of the PC's $totalGB GB (WSL's default)"
if (Test-Path $wslcfg) {
  $m = Select-String -Path $wslcfg -Pattern '^\s*memory\s*=\s*(\d+)\s*(GB|MB)' -ErrorAction SilentlyContinue | Select-Object -First 1
  if ($m) {
    $v = [double]$m.Matches[0].Groups[1].Value; if ($m.Matches[0].Groups[2].Value -eq "MB") { $v = $v / 1024 }
    $wslMemGB = [math]::Round($v, 1); $memSource = "set in $wslcfg"
  }
}
if ($wslMemGB -lt 7.5) {
  Caution "WSL gets about $wslMemGB GB of memory ($memSource): the simulator plus the detector need about 8 GB"
  if ($totalGB -ge 12) {
    Tip "give WSL more: put these two lines into $wslcfg   [wsl2]   and   memory=$([math]::Min(12, [math]::Floor($totalGB * 0.75)))GB"
    Tip "then in PowerShell:  wsl --shutdown   (closes all Linux programs) and open Ubuntu again"
  } else { Tip "this PC has only $totalGB GB: use the light answer-key detector (compose_up.sh --detector truth)" }
} else { Pass "WSL can use about $wslMemGB GB of memory ($memSource)" }

function FreeGB([string]$path) {
  try { $q = Split-Path -Qualifier $path; $d = Get-PSDrive -Name $q.TrimEnd(':') -ErrorAction Stop; return [math]::Round($d.Free / 1GB, 0) } catch { return -1 }
}
$vhdDir = $null                           # where Windows keeps the distro's virtual disk (ext4.vhdx)
$lxss = @(); try { $lxss = @(Get-ChildItem "HKCU:\Software\Microsoft\Windows\CurrentVersion\Lxss" -ErrorAction Stop) } catch { }
foreach ($key in $lxss) {
  $p = Get-ItemProperty $key.PSPath
  if ($p.DistributionName -eq $Distro) { $vhdDir = [string]$p.BasePath }
}
if ($vhdDir) {
  $vhdDir = $vhdDir -replace '^\\\\\?\\', ''
  $f = FreeGB $vhdDir
  if ($f -ge 0 -and $f -lt 40) { Caution "only $f GB free on the drive of $Distro's virtual disk ($vhdDir): the images need about 25 GB plus build space (40 GB)"; Tip "free space on that drive (Docker Engine inside $Distro keeps its images there)" }
  elseif ($f -ge 0) { Pass "$f GB free on the drive of $Distro's virtual disk ($vhdDir)" }
}

# ---- 2. WSL and the Ubuntu-22.04 distro ------------------------------------------------------------------
Title "2. WSL (the Linux part of Windows) and $Distro"
$env:WSL_UTF8 = "1"                       # newer WSL prints UTF-8 instead of UTF-16 with this
$wslVersion = ""
try { $wslVersion = ((& wsl.exe --version 2>$null) -join "`n") -replace "`0", "" } catch { }
if (-not (Get-Command wsl.exe -ErrorAction SilentlyContinue)) {
  Fail "WSL is not installed"
  Tip "in an administrator PowerShell:  wsl --install -d $Distro    then restart the PC (docs/simulation/DOCKER.md, A1)"
} else {
  if ($wslVersion -match 'WSL[^:]*:\s*([0-9.]+)') { Pass "WSL $($Matches[1]) (the Microsoft Store version: WSLg windows work)" }
  else { Caution "an old built-in WSL (no 'wsl --version'): Linux windows (WSLg) and newer fixes are missing"; Tip "in PowerShell:  wsl --update   then  wsl --shutdown" }
  $list = @()
  try { $list = ((& wsl.exe -l -v 2>$null) -join "`n") -replace "`0", "" -split "`n" } catch { }
  $found = $null; $default = $null
  foreach ($line in $list) {
    $t = $line.Trim(); if (-not $t -or $t -match '^NAME\s') { continue }
    $isDefault = $t.StartsWith("*"); $t = $t.TrimStart("*").Trim()
    $parts = $t -split '\s+'
    if ($parts.Count -lt 3) { continue }
    if ($isDefault) { $default = $parts[0] }
    if ($parts[0] -eq $Distro) { $found = @{ State = $parts[1]; Version = $parts[2] } }
  }
  if (-not $found) {
    Fail "the Linux distro $Distro is not installed (our scripts and the native simulator need exactly this one)"
    Tip "in PowerShell:  wsl --install -d $Distro     then finish its first start (user name + password)"
  } elseif ($found.Version -ne "2") {
    Fail "$Distro runs on WSL version $($found.Version): Docker needs version 2"
    Tip "in PowerShell:  wsl --set-version $Distro 2"
  } else { Pass "$Distro is installed (WSL 2, now $($found.State.ToLower()))" }
  if ($found -and $default -and ($default -ne $Distro)) {
    Note "the default distro is $default, not ${Distro}: 'bash' typed in PowerShell or cmd opens $default"
    Tip "always open the right one:  wsl -d $Distro     (or make it the default:  wsl --set-default $Distro)"
  }
}

# ---- 3. Docker Desktop (optional: Docker Engine inside Ubuntu is the tested way) ---------------------------
Title "3. Docker on Windows"
$ddExe = Join-Path $env:ProgramFiles "Docker\Docker\Docker Desktop.exe"
if (-not (Test-Path $ddExe)) {
  Note "Docker Desktop is not installed: fine, the tested way is Docker Engine inside $Distro (install_docker_engine.sh, DOCKER.md A3)"
} else {
  $running = [bool](Get-Process -Name "Docker Desktop" -ErrorAction SilentlyContinue)
  if ($running) { Pass "Docker Desktop is installed and running" }
  else { Note "Docker Desktop is installed but not running (needed only if you use it instead of Docker Engine inside $Distro)" }
  $cfg = $null
  foreach ($n in @("settings-store.json", "settings.json")) {
    $p = Join-Path $env:APPDATA "Docker\$n"
    if (Test-Path $p) { try { $cfg = Get-Content $p -Raw | ConvertFrom-Json; break } catch { } }
  }
  if ($cfg) {
    $distros = @(); foreach ($k in @("IntegratedWslDistros", "integratedWslDistros")) { if ($cfg.PSObject.Properties[$k]) { $distros = @($cfg.$k) } }
    $wslEngine = $true; foreach ($k in @("WslEngineEnabled", "wslEngineEnabled")) { if ($cfg.PSObject.Properties[$k]) { $wslEngine = [bool]$cfg.$k } }
    if (-not $wslEngine) { Caution "Docker Desktop uses its old Hyper-V engine, not WSL 2: no WSL integration, no WSLg windows for containers"; Tip "Docker Desktop -> Settings -> General -> tick 'Use the WSL 2 based engine'" }
    if ($distros -contains $Distro) {
      Note "Docker Desktop's WSL integration is ON for ${Distro}: then do NOT also install Docker Engine there (two Dockers fight)"
      if (-not $running) { Caution "with the integration on, 'docker' in $Distro only works while Docker Desktop runs"; Tip "start Docker Desktop and wait for 'Engine running'" }
    } else {
      Note "Docker Desktop's WSL integration is off for ${Distro}: right if $Distro has its own Docker Engine (our tested way)"
      Tip "to use Docker Desktop instead: Settings -> Resources -> WSL integration -> switch on $Distro -> Apply & restart"
    }
  }
  $ddDisk = Join-Path $env:LOCALAPPDATA "Docker\wsl"
  if (Test-Path $ddDisk) { $f = FreeGB $ddDisk; if ($f -ge 0 -and $f -lt 40) { Caution "only $f GB free on the drive of Docker Desktop's disk ($ddDisk)"; Tip "Docker Desktop -> Settings -> Resources -> Advanced -> Disk image location: pick a bigger drive" } }
}

# ---- 4. git and this checkout --------------------------------------------------------------------------------
Title "4. git and this copy of the code ($Repo)"
if (-not (Get-Command git -ErrorAction SilentlyContinue)) {
  Note "git is not installed on Windows: fine if you clone inside Ubuntu (recommended: git clone ... in the $Distro terminal)"
} else {
  # Ask git about THIS checkout (git -C): run from another folder, plain "git config" would answer for that folder
  # (or only the global/system value) and could say "true" although this repo sets false.
  $isCheckout = Test-Path (Join-Path $Repo ".git")
  if ($isCheckout) { $autocrlf = (& git -C $Repo config --get core.autocrlf 2>$null) } else { $autocrlf = (& git config --get core.autocrlf 2>$null) }
  if ($autocrlf -eq "true") {
    # The new .gitattributes has a default rule for EVERY text file ("* text=auto eol=lf"); an older one only covered *.sh.
    $ga = Join-Path $Repo ".gitattributes"
    $allLf = (Test-Path $ga) -and [bool](Select-String -Path $ga -Pattern '^\*\s+text=auto\s+eol=lf' -Quiet -ErrorAction SilentlyContinue)
    if ($allLf) { Note "Git for Windows writes Windows line endings (core.autocrlf=true); this repo's .gitattributes overrides it (LF in every text file)" }
    else {
      Caution "Git writes Windows line endings here (core.autocrlf=true) and this checkout's .gitattributes is the old one (it protects only *.sh files)"
      Tip "get the new one:  git pull   then, in the $Distro terminal:  bash simulation/docker/check_host.sh --fix-line-endings"
    }
  }
  if ($isCheckout) {
    $scripts = @(& git -C $Repo ls-files -- '*.sh' '*.bash' 2>$null) + @(& git -C $Repo grep --cached -I -l -e '^#!' -- . ':(exclude)*.md' ':(exclude)*.txt' ':(exclude)*.html' ':(exclude)*.mhtml' 2>$null) | Sort-Object -Unique
    $bad = @()
    if ($scripts.Count -gt 0) {
      foreach ($l in (& git -C $Repo ls-files --eol -- $scripts 2>$null)) {
        $meta, $path = $l -split "`t", 2
        if ($meta -match '^i/lf\s+w/(crlf|mixed)' -and $meta -notmatch 'eol=crlf') { $bad += $path }
      }
    }
    if ($bad.Count -gt 0) {
      $ex = $bad | Where-Object { $_ -like "*main_controller.py" } | Select-Object -First 1; if (-not $ex) { $ex = $bad[0] }
      Fail "$($bad.Count) script(s) in this checkout have Windows line endings (CRLF), e.g. ${ex}: in Linux/Docker they fail ('python3\r: No such file')"
      Tip "repair (keeps your edits), in the $Distro terminal:  bash simulation/docker/check_host.sh --fix-line-endings"
      Tip "or clone again inside Ubuntu (recommended, also faster):  wsl -d $Distro   then   git clone <repo URL>"
    } else { Pass "the scripts of this checkout have Linux line endings (LF)" }
  } else { Note "this folder is not a git checkout (a ZIP download?): line endings are checked by check_host.sh inside Ubuntu" }
}
if ($env:OneDrive -and $Repo.StartsWith($env:OneDrive, [System.StringComparison]::OrdinalIgnoreCase)) {
  Caution "this checkout is inside OneDrive ($env:OneDrive): syncing locks and 'online-only' files break git and builds"
  Tip "clone inside Ubuntu instead (wsl -d $Distro, then git clone ...), or into a folder outside OneDrive such as C:\src"
}
if (($Repo.Length + 115) -gt 259) { Caution "the folder path is long ($($Repo.Length) characters): the longest repo file would pass Windows' 260-character limit"; Tip "git config --global core.longpaths true    (or clone into a shorter path, or inside Ubuntu)" }
if ($Repo -match ' ') { Note "the folder path has a space: fine for the scripts (they quote it); the native build copies the repo to a path without spaces" }

# ---- 5. NVIDIA, Mission Planner, proxy, clock ------------------------------------------------------------
Title "5. Graphics card, Mission Planner, proxy and clock"
$smi = Join-Path $env:SystemRoot "System32\nvidia-smi.exe"
if (Test-Path $smi) {
  $g = (& $smi --query-gpu=name,driver_version --format=csv,noheader 2>$null | Select-Object -First 1)
  if ($g) { Pass "NVIDIA card: $g (WSL2 and Docker reach it through the Windows driver; nothing to install inside Linux)" }
} else { Note "no NVIDIA driver found: the detector runs on the processor (slower; --detector truth is the light choice)" }
$mp = $null; try { $mp = Get-NetUDPEndpoint -LocalPort 14550 -ErrorAction Stop | Select-Object -First 1 } catch { }
if ($mp) {
  $pn = ""; try { $pn = (Get-Process -Id $mp.OwningProcess -ErrorAction Stop).ProcessName } catch { }
  Note "UDP port 14550 is open on Windows ($pn): that is where the simulator sends Mission Planner its MAVLink copy"
} else { Note "nothing listens on UDP 14550 yet (Mission Planner: connect with UDP, port 14550, when a simulator runs)" }
# Proxy: read the settings themselves, not the text of "netsh winhttp show proxy" (that text is translated on a
# non-English Windows). Three places: proxy variables, the user's proxy (Settings -> Network -> Proxy, which
# Docker Desktop follows) and the machine-wide WinHTTP proxy (binary value: byte 8 has bit 2 set when a proxy is
# configured; bytes 12-15 hold the length of the proxy name that follows).
$px = @(); foreach ($v in @("HTTP_PROXY", "HTTPS_PROXY", "http_proxy", "https_proxy")) { if ((Get-Item "Env:$v" -ErrorAction SilentlyContinue)) { $px += "variable $v" } }
$inet = $null; try { $inet = Get-ItemProperty "HKCU:\Software\Microsoft\Windows\CurrentVersion\Internet Settings" -ErrorAction Stop } catch { }
if ($inet) {
  if (($inet.ProxyEnable -eq 1) -and $inet.ProxyServer) { $px += "Windows proxy $($inet.ProxyServer)" }
  if ($inet.AutoConfigURL) { $px += "proxy script $($inet.AutoConfigURL)" }
}
$wh = $null; try { $wh = (Get-ItemProperty "HKLM:\SOFTWARE\Microsoft\Windows\CurrentVersion\Internet Settings\Connections" -Name WinHttpSettings -ErrorAction Stop).WinHttpSettings } catch { }
if ($wh -and $wh.Length -gt 16 -and ($wh[8] -band 2)) {
  $n = [BitConverter]::ToInt32($wh, 12); $name = ""
  if ($n -gt 0 -and (16 + $n) -le $wh.Length) { $name = [Text.Encoding]::ASCII.GetString($wh, 16, $n) }
  $px += "WinHTTP proxy $name"
}
if ($px.Count -gt 0) {
  Caution "this PC uses a proxy ($($px -join '; ')): Docker and the Linux build need it too"
  Tip "Docker Desktop -> Settings -> Resources -> Proxies; Docker Engine in Ubuntu: see check_host.sh (section 5)"
}
if (-not $Offline) {
  # Windows PowerShell 5.1 may offer only old TLS versions, which github.com refuses: allow TLS 1.2 for this run only.
  [Net.ServicePointManager]::SecurityProtocol = [Net.ServicePointManager]::SecurityProtocol -bor [Net.SecurityProtocolType]::Tls12
  $r = $null
  try {
    $r = Invoke-WebRequest -Uri "https://github.com/" -Method Head -UseBasicParsing -TimeoutSec 8 -ErrorAction Stop
  } catch {
    $msg = $_.Exception.Message
    if ($msg -match 'trust relationship|certificate') { Caution "github.com answered with a certificate Windows does not trust: the network inspects HTTPS"; Tip "build on another network (home, phone hotspot), or ask IT" }
    else { Caution "github.com cannot be reached: $($msg.Substring(0, [math]::Min(80, $msg.Length)))"; Tip "the first build downloads about 10 GB: use a working internet connection" }
  }
  if ($r) {
    # The server's clock comes as "Sat, 31 Oct 2026 12:00:00 GMT" (English on every PC): read it with the invariant
    # culture, so a German or French Windows reads it too. If it still cannot be read, only the clock check is skipped.
    $remote = $null
    try {
      $styles = [Globalization.DateTimeStyles]::AdjustToUniversal -bor [Globalization.DateTimeStyles]::AssumeUniversal
      $remote = [DateTime]::Parse([string]$r.Headers["Date"], [Globalization.CultureInfo]::InvariantCulture, $styles)
    } catch { }
    if (-not $remote) { Pass "internet works (github.com answered; its clock could not be read, so the clock was not compared)" }
    else {
      $skew = [math]::Abs(((Get-Date).ToUniversalTime() - $remote).TotalSeconds)
      if ($skew -gt 120) { Caution "the Windows clock is off by $([math]::Round($skew)) s: downloads with certificates and apt can fail"; Tip "Settings -> Time & language -> Date & time -> Sync now" }
      else { Pass "internet works and the clock is right (within $([math]::Round($skew)) s)" }
    }
  }
}

Write-Output ""
Write-Output "------------------------------------------------------------------------------------------"
if ($script:Fails -gt 0) {
  Write-Output "$($script:Fails) problem(s) to fix first (the [FAIL] lines), $($script:Warns) warning(s). Then run this check again."
  Write-Output "Every message is explained in docs/simulation/DOCKER.md, 'Troubleshooting'."
  exit 1
}
Write-Output "Nothing on the Windows side blocks Docker ($($script:Warns) warning(s)). Next, inside Ubuntu:"
Write-Output "    wsl -d $Distro      then, in the repo folder:   bash simulation/docker/check_host.sh"
exit 0
