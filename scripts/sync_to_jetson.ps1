<#
.SYNOPSIS
    Synchronizes the James Useful Home Robot project from Windows to a Jetson machine.

.DESCRIPTION
    This script uses rsync (via Cygwin) to sync the project root
    to the Jetson machine while excluding unnecessary directories like build artifacts and git history.

.PARAMETER JetsonIP
    The IP address of the Jetson machine. Defaults to 192.168.0.99.

.PARAMETER Username
    The username for the Jetson machine. Defaults to jetson.

.PARAMETER DryRun
    If present, performs a dry run without actually transferring files.

.EXAMPLE
    .\scripts\sync_to_jetson.ps1 -JetsonIP 192.168.0.99 -DryRun
#>

param (
    [string]$JetsonIP = "192.168.0.99",
    [string]$Username = "jetson",
    [switch]$DryRun
)

# --- Configuration ---
$SourcePath = "c:\Users\denis\Documents\James_Useful_Home_Robot\ros2_ws\"
$DestPath = "$($Username)@$($JetsonIP):~/james-useful-home-robot/ros2_ws"

# Exclude list for rsync
$Excludes = @(
    ".git/",
    "__pycache__/",
    "node_modules/",
    "ros2_ws/build/",
    "ros2_ws/install/",
    "ros2_ws/log/",
    "build"
    ".kiro/",
    "temp/",
    "log",
    "*.pyc",
    ".gemini/",
    ".vscode/"
)

# Convert excludes to rsync arguments
$ExcludeArgs = $Excludes | ForEach-Object { "--exclude='$_'" }

# --- Check for rsync ---
$RsyncCmd = "C:\cygwin\bin\rsync.exe"

if (-not (Test-Path $RsyncCmd)) {
    # If not at the Cygwin path, check if it's in the system PATH
    $RsyncPath = Get-Command "rsync" -ErrorAction SilentlyContinue
    if ($RsyncPath) {
        $RsyncCmd = "rsync"
    } else {
        Write-Error "rsync not found at 'C:\cygwin\bin\rsync.exe' or in your system PATH. Please ensure Cygwin rsync is installed."
        exit 1
    }
}

# --- Path Translation for Cygwin ---
# Cygwin rsync often struggles with Windows paths like 'C:\...'. 
# We'll convert the source path to a Cygwin-style path (/cygdrive/c/...) if needed.
$CygwinSource = $SourcePath.ToString().Replace(":", "").Replace("\", "/")
$CygwinSource = "/cygdrive/$($CygwinSource.Substring(0, 1).ToLower())$($CygwinSource.Substring(1))"

# --- Find SSH ---
# Cygwin rsync is INCOMPATIBLE with native Windows OpenSSH.
# We must find a Cygwin/MSYS2-based SSH client (like from Cygwin itself or Git Bash).
$SshCmd = "ssh" # Fallback
$CompatibleSshFound = $false

$SshPaths = @(
    "C:\cygwin\bin\ssh.exe",
    "C:\Program Files\Git\usr\bin\ssh.exe",
    "C:\Program Files\Git\bin\ssh.exe"
)

foreach ($path in $SshPaths) {
    if (Test-Path $path) {
        $SshCmd = $path.Replace("\", "/")
        $CompatibleSshFound = $true
        Write-Host "Found compatible SSH at: $SshCmd" -ForegroundColor Gray
        break
    }
}

if (-not $CompatibleSshFound) {
    Write-Warning "No Cygwin/Git-Bash SSH found. Rsync will likely fail with 'code 12' if it uses Windows OpenSSH."
    Write-Host "Tip: Install the 'openssh' package in Cygwin or use the Git Bash version." -ForegroundColor Cyan
}

# --- Construction of command ---
# We use an array for arguments to avoid quoting issues with Invoke-Expression
$RsyncArgs = @("-av", "--delete") 
$RsyncArgs += "--no-compress" # Disable compression to avoid protocol mismatches
$RsyncArgs += "--protocol=31" # Force protocol version to match Jetson (rsync 3.1.3)

if ($DryRun) {
    $RsyncArgs += "-n"
    Write-Host "--- DRY RUN ENABLED ---" -ForegroundColor Yellow
}

# Construct full destination using absolute path to be safe
$AbsoluteDest = "$($Username)@$($JetsonIP):/home/$($Username)/james-useful-home-robot/ros2_ws"

# Build the argument list
$AllArgs = $RsyncArgs
$AllArgs += "--rsh=$SshCmd"
$AllArgs += "--rsync-path=/usr/bin/rsync"
foreach ($ex in $Excludes) {
    $AllArgs += "--exclude=$ex"
}
$AllArgs += "$CygwinSource/"
$AllArgs += $AbsoluteDest

Write-Host "Starting Sync..." -ForegroundColor Cyan
Write-Host "Syncing from: $SourcePath" -ForegroundColor Green
Write-Host "Syncing to  : $AbsoluteDest" -ForegroundColor Green
Write-Host "Executing   : & '$RsyncCmd' $($AllArgs -join ' ')" -ForegroundColor Gray

# --- Execute ---
# Using the call operator & with an array is much more robust
& $RsyncCmd $AllArgs
