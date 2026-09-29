param([string]$Python = 'python')
$ErrorActionPreference = 'Stop'
Set-Location -LiteralPath $PSScriptRoot
$pythonExe = Join-Path $PSScriptRoot '.venv-gesture/Scripts/python.exe'
if (-not (Test-Path -LiteralPath $pythonExe)) {
    & $Python -m venv .venv-gesture
    if ($LASTEXITCODE -ne 0) { throw 'Cannot create Python environment. Install Python 3.12 or pass -Python C:\path\python.exe' }
}
& $pythonExe -m pip install -r requirements-tested.txt
if ($LASTEXITCODE -ne 0) { throw 'Gesture dependencies could not be installed' }
$model = Join-Path $PSScriptRoot 'models/gesture_recognizer.task'
if (-not (Test-Path -LiteralPath $model)) {
    $download = "$model.download"
    try {
        Invoke-WebRequest -Uri 'https://storage.googleapis.com/mediapipe-models/gesture_recognizer/gesture_recognizer/float16/latest/gesture_recognizer.task' -OutFile $download
        Move-Item -LiteralPath $download -Destination $model
    } finally {
        if (Test-Path -LiteralPath $download) { Remove-Item -LiteralPath $download }
    }
}
Write-Output 'Ready. Demo: .\.venv-gesture\Scripts\python.exe lever_control.py --demo'
Write-Output 'Camera preview: .\.venv-gesture\Scripts\python.exe lever_control.py --camera 0'
