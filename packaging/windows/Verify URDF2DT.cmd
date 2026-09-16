@echo off
setlocal
set "URDF2DT_VERIFY_ROOT=%~dp0"
powershell.exe -NoProfile -Command "$out = Join-Path ([Environment]::GetFolderPath('MyDocuments')) ('URDF2DT\checks\' + [DateTime]::Now.ToString('yyyyMMdd-HHmmss')); $exe = Join-Path $env:URDF2DT_VERIFY_ROOT 'URDF2DT.exe'; $p = Start-Process -FilePath $exe -ArgumentList ('--verify-package "' + $out + '"') -WindowStyle Hidden -Wait -PassThru; if ($p.ExitCode -ne 0) { Write-Host 'Verification failed. See Local AppData URDF2DT logs.'; exit 1 }; Write-Host ('Verification passed. Results: ' + $out)"
pause
