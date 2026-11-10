$ErrorActionPreference = 'Stop'
$src = 'D:\Project\ByTime\D-2026Q1\5-NTE\4-drift\nfs2015-chase-cam\build.mingw\package\robineatcorn.nfs2015-chase-cam'
$dst = 'C:\Program Files (x86)\Neverness To Everness\Anomaly\Anomaly-2.7.0-runtime\Anomaly\plugins\robineatcorn.nfs2015-chase-cam'
$result = 'D:\Project\ByTime\D-2026Q1\5-NTE\4-drift\nfs2015-chase-cam\wheel-steer-install-result.txt'
$lines = @()
try {
  $lines += "START $(Get-Date -Format o)"
  $lines += "SRC=$src"
  $lines += "DST=$dst"
  $old = Join-Path $dst 'plugin.dll'
  $backup = Join-Path $dst 'plugin.dll.pre-wheel-steer'
  if (Test-Path -LiteralPath $old) {
    Copy-Item -LiteralPath $old -Destination $backup -Force
    $lines += "BACKUP_OK $backup"
    Set-ItemProperty -LiteralPath $old -Name IsReadOnly -Value $false
  }
  Copy-Item -LiteralPath (Join-Path $src 'plugin.dll') -Destination $old -Force
  $lines += "PLUGIN_COPY_OK"
  $manifest = Join-Path $src 'manifest.json'
  if (Test-Path -LiteralPath $manifest) {
    Copy-Item -LiteralPath $manifest -Destination (Join-Path $dst 'manifest.json') -Force
    $lines += "MANIFEST_COPY_OK"
  }
  $srcLocales = Join-Path $src 'locales'
  if (Test-Path -LiteralPath $srcLocales) {
    Copy-Item -LiteralPath $srcLocales -Destination $dst -Recurse -Force
    $lines += "LOCALES_COPY_OK"
  }
  $lines += "DONE $(Get-Date -Format o)"
} catch {
  $lines += "FAIL $($_.Exception.GetType().FullName): $($_.Exception.Message)"
  $lines += "STACK $($_.ScriptStackTrace)"
}
$lines | Set-Content -LiteralPath $result -Encoding UTF8
