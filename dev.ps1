<#
.SYNOPSIS
    Runs the Kinect scanner backend and the Vite dev server together.

.EXAMPLE
    .\dev.ps1
    Opens the physical Kinect. Falls back to synthetic frames if none is found.

.EXAMPLE
    .\dev.ps1 -Mock
    Forces synthetic frames -- no sensor, no USB driver needed.
#>

[CmdletBinding()]
param(
    [switch]$Mock,
    [int]$Port = 8000,
    # Surface detail. Finer costs a lot more in time AND memory: 20mm extracts
    # in 45ms, 12mm in 120ms, 8mm in ~1.1s. Below ~5mm the sensor's own depth
    # quantisation dominates, so the extra voxels resolve noise, not geometry.
    # 8mm once got this server OOM-killed on an 8 GB machine -- check free RAM
    # before going below 12.
    [double]$VoxelMm = 12.0
)

$ErrorActionPreference = "Stop"

# These are ordinary "you forgot something" conditions, not crashes -- print
# the fix and leave, instead of burying it in a PowerShell stack trace.
function Fail($message) {
    Write-Host $message -ForegroundColor Red
    exit 1
}

$root     = $PSScriptRoot
$python   = Join-Path $root ".venv\Scripts\python.exe"
$frontend = Join-Path $root "frontend"

if (-not (Test-Path $python)) {
    Fail ("No virtualenv found at .venv. Create one first:`n" +
          "  py -m venv .venv`n" +
          "  .venv\Scripts\python.exe -m pip install -r backend\requirements.txt")
}

# A leftover backend still holds the Kinect's USB handle, so the next run would
# silently fall back to mock mode. Say so instead.
$busy = Get-NetTCPConnection -LocalPort $Port -State Listen -ErrorAction SilentlyContinue
if ($busy) {
    $owner = (Get-Process -Id $busy[0].OwningProcess -ErrorAction SilentlyContinue).ProcessName
    Fail ("Port $Port is already in use by $owner (PID $($busy[0].OwningProcess)).`n" +
          "  Stop it:          Stop-Process -Id $($busy[0].OwningProcess) -Force`n" +
          "  Or use another:   .\dev.ps1 -Port 8001")
}

if (-not (Test-Path (Join-Path $frontend "node_modules"))) {
    Write-Host "Installing frontend dependencies..." -ForegroundColor Yellow
    & npm.cmd --prefix $frontend install
    if ($LASTEXITCODE -ne 0) { Fail "npm install failed." }
}

$procs = @()

function Stop-All {
    foreach ($p in $script:procs) {
        if ($p -and -not $p.HasExited) {
            # /T kills the tree: npm runs vite as a child process.
            & taskkill.exe /PID $p.Id /T /F 2>&1 | Out-Null
        }
    }
}

try {
    # Start-Process joins ArgumentList on spaces without quoting, and this
    # repo lives under a path that has one -- so quote the script path here.
    $serverPy = Join-Path $root "backend\server.py"
    $backendArgs = @("`"$serverPy`"", "--port", "$Port", "--voxel-mm", "$VoxelMm")
    if ($Mock) { $backendArgs += "--mock" }

    # Both share this console, so the driver's "REAL hardware" vs "MOCK mode"
    # line and Vite's output land in one place.
    $procs += Start-Process -FilePath $python -ArgumentList $backendArgs `
        -WorkingDirectory $root -PassThru -NoNewWindow

    $procs += Start-Process -FilePath "npm.cmd" -ArgumentList "run", "dev" `
        -WorkingDirectory $frontend -PassThru -NoNewWindow

    Write-Host ""
    Write-Host "  scanner    http://localhost:5173" -ForegroundColor Green
    Write-Host "  api        http://localhost:$Port/api/status" -ForegroundColor DarkGray
    Write-Host "  surface    $VoxelMm mm voxels" -ForegroundColor DarkGray
    Write-Host "  pids       $($procs.Id -join ', ')" -ForegroundColor DarkGray
    Write-Host "  Ctrl+C to stop both." -ForegroundColor DarkGray
    Write-Host ""

    Wait-Process -Id $procs.Id
}
finally {
    Stop-All
}
