$p = Start-Process -FilePath 'powershell.exe' -Verb RunAs -Wait -PassThru -ArgumentList @('-NoProfile','-ExecutionPolicy','Bypass','-File','D:\Project\ByTime\D-2026Q1\5-NTE\4-drift\nfs2015-chase-cam\install_phase0.ps1')
Write-Output ("EXIT=" + $p.ExitCode)
