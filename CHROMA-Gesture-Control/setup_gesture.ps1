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
$model = Join-Path $PSScriptRoot 'models/hand_landmarker.task'
if (-not (Test-Path -LiteralPath $model)) {
    New-Item -ItemType Directory -Force -Path models | Out-Null
    Invoke-WebRequest -Uri 'https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task' -OutFile "$model.download"
    Move-Item -LiteralPath "$model.download" -Destination $model
}
Write-Output 'Ready. Record: .\.venv-gesture\Scripts\python.exe gesture_collect.py --split train'
Write-Output 'Demo UI:     .\.venv-gesture\Scripts\python.exe gesture_control.py --demo'
