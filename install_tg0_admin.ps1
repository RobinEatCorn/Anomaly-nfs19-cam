$ErrorActionPreference = 'Stop'
$src     = 'D:\Project\ByTime\D-2026Q1\5-NTE\4-drift\nfs2015-chase-cam\build.mingw\package\robineatcorn.nfs2015-chase-cam'
$dst     = 'C:\Program Files (x86)\Neverness To Everness\Anomaly\Anomaly-2.7.0-runtime\Anomaly\plugins\robineatcorn.nfs2015-chase-cam'
$enable  = 'C:\Program Files (x86)\Neverness To Everness\Anomaly\Anomaly-2.7.0-runtime\Anomaly\config\plugin-enablement.json'
$result  = 'D:\Project\ByTime\D-2026Q1\5-NTE\4-drift\nfs2015-chase-cam\tg0-install-result.txt'
$lines   = @()
try {
  $lines += "START $(Get-Date -Format o)"
  $lines += "ADMIN $(([Security.Principal.WindowsPrincipal][Security.Principal.WindowsIdentity]::GetCurrent()).IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator))"

  # 1) plugin payload
  $oldDll = Join-Path $dst 'plugin.dll'
  if (Test-Path -LiteralPath $oldDll) {
    Set-ItemProperty -LiteralPath $oldDll -Name IsReadOnly -Value $false
    Copy-Item -LiteralPath $oldDll -Destination (Join-Path $dst 'plugin.dll.pre-tg0') -Force
    $lines += "BACKUP_OK"
  }
  Copy-Item -LiteralPath (Join-Path $src 'plugin.dll')      -Destination $oldDll -Force
  Copy-Item -LiteralPath (Join-Path $src 'manifest.json')   -Destination (Join-Path $dst 'manifest.json') -Force
  Copy-Item -LiteralPath (Join-Path $src 'locales')         -Destination $dst -Recurse -Force
  $lines += "PLUGIN_COPY_OK $((Get-Item $oldDll).Length)"

  # 2) enable the plugin (the host reads this at startup)
  $json = Get-Content -LiteralPath $enable -Raw | ConvertFrom-Json
  $json.plugins | Add-Member -NotePropertyName 'robineatcorn.nfs2015-chase-cam' -NotePropertyValue $true -Force
  $json.plugins | Add-Member -NotePropertyName 'anomaly.nte-vehicle' -NotePropertyValue $false -Force
  ($json | ConvertTo-Json -Depth 8) | Set-Content -LiteralPath $enable -Encoding UTF8
  $lines += "ENABLE_OK"
  $lines += "DONE $(Get-Date -Format o)"
} catch {
  $lines += "FAIL $($_.Exception.GetType().FullName): $($_.Exception.Message)"
  $lines += "STACK $($_.ScriptStackTrace)"
}
$lines | Set-Content -LiteralPath $result -Encoding UTF8
