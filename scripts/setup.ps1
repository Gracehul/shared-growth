param(
    [string]$PythonExecutable = "",
    [switch]$Hardware
)

$ErrorActionPreference = "Stop"
$ProjectRoot = Split-Path -Parent $PSScriptRoot
Set-Location -LiteralPath $ProjectRoot

if (-not $PythonExecutable) {
    $Candidates = @("py", "python", "python3")
    foreach ($Candidate in $Candidates) {
        if (Get-Command $Candidate -ErrorAction SilentlyContinue) {
            $PythonExecutable = $Candidate
            break
        }
    }
}

if (-not $PythonExecutable) {
    throw "Python 3.10+ was not found. Install Python, or pass -PythonExecutable C:\path\to\python.exe"
}

Write-Host "Using Python: $PythonExecutable"
& $PythonExecutable -c "import sys; assert sys.version_info >= (3, 10), 'Python 3.10+ required'; print(sys.version)"
if ($LASTEXITCODE -ne 0) {
    throw "Python version check failed with exit code $LASTEXITCODE"
}

if (-not (Test-Path -LiteralPath ".venv")) {
    & $PythonExecutable -m venv .venv
    if ($LASTEXITCODE -ne 0) {
        throw "Creating .venv failed with exit code $LASTEXITCODE"
    }
}

$VenvPython = Join-Path $ProjectRoot ".venv\Scripts\python.exe"
& $VenvPython -m pip install --upgrade pip
if ($LASTEXITCODE -ne 0) {
    throw "Upgrading pip failed with exit code $LASTEXITCODE"
}

$InstallTarget = if ($Hardware) { ".[hardware]" } else { "." }
& $VenvPython -m pip install -e $InstallTarget
if ($LASTEXITCODE -ne 0) {
    throw "Installing $InstallTarget failed with exit code $LASTEXITCODE"
}
& $VenvPython -m drawing_robot.preflight
if ($LASTEXITCODE -ne 0) {
    throw "Preflight failed with exit code $LASTEXITCODE"
}

Write-Host "Setup complete. Activate with: .\.venv\Scripts\Activate.ps1"
if ($Hardware) {
    Write-Host "Hardware packages are installed. No robot connection was opened."
}
