$ErrorActionPreference = 'Stop'
Set-Location -LiteralPath $PSScriptRoot
python -m venv .venv-gesture
if ($LASTEXITCODE -ne 0) { throw 'Could not create gesture environment' }
& '.\.venv-gesture\Scripts\python.exe' -m pip install -r requirements-gesture.txt
if ($LASTEXITCODE -ne 0) { throw 'Could not install gesture dependencies' }
New-Item -ItemType Directory -Force -Path models | Out-Null
if (-not (Test-Path -LiteralPath 'models\hand_landmarker.task')) {
    Invoke-WebRequest -Uri 'https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task' -OutFile 'models\hand_landmarker.task.download'
    Move-Item -LiteralPath 'models\hand_landmarker.task.download' -Destination 'models\hand_landmarker.task'
}
if (-not (Test-Path -LiteralPath 'models\gesture_recognizer.task')) {
    Invoke-WebRequest -Uri 'https://storage.googleapis.com/mediapipe-models/gesture_recognizer/gesture_recognizer/float16/latest/gesture_recognizer.task' -OutFile 'models\gesture_recognizer.task.download'
    Move-Item -LiteralPath 'models\gesture_recognizer.task.download' -Destination 'models\gesture_recognizer.task'
}
Write-Output 'Ready. Preview: .\.venv-gesture\Scripts\python.exe gesture_control.py --camera 0'
