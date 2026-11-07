#requires -Version 5.1
<#
.SYNOPSIS
  Preview or copy NFS 2015 Chase Cam data from the old plugin id to the
  Anomaly 2.7.0 two-segment id.

.DESCRIPTION
  The plugin id changed from io.github.robineatcorn.nfs2015-chase-cam to
  robineatcorn.nfs2015-chase-cam. Anomaly keys settings and window state by
  plugin id, so old data does not follow automatically.

  This script never deletes the old settings, old window state, or old install
  directory. It only adds the new id and copies missing files. Run it without
  -Apply first to see the plan. Run with -Apply to write. Use -Force only when
  the destination already has a file and you want the old file to replace it.

.PARAMETER AnomalyRoot
  The Anomaly directory (the one containing config\ and state\), or the game
  root containing Anomaly\. If omitted, the script checks the current directory
  and the repository's parent directories.

.PARAMETER Apply
  Write the changes. Without this switch the script only prints the plan.

.PARAMETER Force
  Overwrite files that already exist in the new config directory.

.EXAMPLE
  .\migrate-2.7.ps1 -AnomalyRoot "C:\Program Files\Neverness To Everness\Anomaly\Anomaly-2.7.0-runtime\Anomaly"

.EXAMPLE
  .\migrate-2.7.ps1 -AnomalyRoot "C:\Program Files\Neverness To Everness\Anomaly\Anomaly-2.7.0-runtime\Anomaly" -Apply
#>
[CmdletBinding()]
param(
    [string]$AnomalyRoot,
    [switch]$Apply,
    [switch]$Force
)

$ErrorActionPreference = 'Stop'
$OldId = 'io.github.robineatcorn.nfs2015-chase-cam'
$NewId = 'robineatcorn.nfs2015-chase-cam'
$Utf8NoBom = New-Object System.Text.UTF8Encoding($false)

function Resolve-AnomalyRoot {
    param([string]$Path)

    if (-not [string]::IsNullOrWhiteSpace($Path)) {
        if (Test-Path -LiteralPath (Join-Path $Path 'Anomaly\config')) {
            $Path = Join-Path $Path 'Anomaly'
        }
        if (-not (Test-Path -LiteralPath (Join-Path $Path 'config'))) {
            throw "Anomaly root not found: $Path (expected config\ inside it)"
        }
        return (Resolve-Path -LiteralPath $Path).Path
    }

    $candidates = @(
        (Join-Path (Get-Location) 'Anomaly'),
        (Join-Path $PSScriptRoot '..\Anomaly'),
        (Join-Path $PSScriptRoot '..\..\Anomaly')
    )
    foreach ($candidate in $candidates) {
        if (Test-Path -LiteralPath (Join-Path $candidate 'config')) {
            return (Resolve-Path -LiteralPath $candidate).Path
        }
    }
    throw 'Pass -AnomalyRoot <Anomaly directory or game root>; no Anomaly\config was found automatically.'
}

function Read-JsonFile {
    param([string]$Path)
    return (Get-Content -LiteralPath $Path -Raw -Encoding UTF8 | ConvertFrom-Json)
}

function Write-JsonFile {
    param([string]$Path, $Document)

    if (-not $Apply) {
        Write-Host "  [preview] would update $Path"
        return
    }
    $directory = Split-Path -Parent $Path
    if ($directory -and -not (Test-Path -LiteralPath $directory)) {
        New-Item -ItemType Directory -Path $directory -Force | Out-Null
    }
    if (Test-Path -LiteralPath $Path) {
        $backup = "$Path.bak-$(Get-Date -Format 'yyyyMMdd-HHmmss')"
        Copy-Item -LiteralPath $Path -Destination $backup -Force
        Write-Host "  backup: $backup"
    }
    $temporary = "$Path.tmp-$PID"
    $json = ($Document | ConvertTo-Json -Depth 32) + "`r`n"
    [System.IO.File]::WriteAllText($temporary, $json, $Utf8NoBom)
    if (Test-Path -LiteralPath $Path) {
        Remove-Item -LiteralPath $Path -Force
    }
    Move-Item -LiteralPath $temporary -Destination $Path -Force
    Write-Host "  updated: $Path"
}

function Copy-MissingTree {
    param([string]$Source, [string]$Destination)

    if (-not (Test-Path -LiteralPath $Source)) {
        Write-Host "  skip: $Source does not exist"
        return
    }
    $sourceRoot = (Resolve-Path -LiteralPath $Source).Path.TrimEnd('\', '/')
    $files = @(Get-ChildItem -LiteralPath $sourceRoot -Recurse -File)
    if ($files.Count -eq 0) {
        Write-Host "  skip: $sourceRoot has no files"
        return
    }
    foreach ($file in $files) {
        $relative = $file.FullName.Substring($sourceRoot.Length).TrimStart('\', '/')
        $target = Join-Path $Destination $relative
        if ((Test-Path -LiteralPath $target) -and -not $Force) {
            Write-Host "  skip existing: $target"
            continue
        }
        Write-Host "  copy: $($file.FullName) -> $target"
        if ($Apply) {
            $parent = Split-Path -Parent $target
            if ($parent -and -not (Test-Path -LiteralPath $parent)) {
                New-Item -ItemType Directory -Path $parent -Force | Out-Null
            }
            Copy-Item -LiteralPath $file.FullName -Destination $target -Force
        }
    }
}

function Merge-Enablement {
    param([string]$Path)

    if (-not (Test-Path -LiteralPath $Path)) {
        Write-Host "  skip: $Path does not exist"
        return
    }
    $document = Read-JsonFile $Path
    if ($null -eq $document.plugins) {
        Write-Host "  skip: enablement document has no plugins object"
        return
    }
    $old = $document.plugins.PSObject.Properties[$OldId]
    if ($null -eq $old) {
        Write-Host "  skip: old id is not present in $Path"
        return
    }
    $new = $document.plugins.PSObject.Properties[$NewId]
    if ($null -ne $new) {
        if ([bool]$new.Value -ne [bool]$old.Value) {
            Write-Host "  keep existing new id value ($($new.Value)); old id is $($old.Value)"
        } else {
            Write-Host "  skip: new id is already configured with the same value"
        }
        return
    }
    Write-Host "  enablement: $OldId = $($old.Value) -> $NewId"
    if ($Apply) {
        Add-Member -InputObject $document.plugins -NotePropertyName $NewId -NotePropertyValue ([bool]$old.Value)
        Write-JsonFile $Path $document
    }
}

function Merge-WindowState {
    param([string]$Path)

    if (-not (Test-Path -LiteralPath $Path)) {
        Write-Host "  skip: $Path does not exist"
        return
    }
    $document = Read-JsonFile $Path
    $changed = $false

    $windows = @($document.pluginWindows)
    $newPresent = $false
    foreach ($window in $windows) {
        if ($null -ne $window -and $window.pluginId -eq $NewId) {
            $newPresent = $true
        }
    }
    foreach ($window in $windows) {
        if ($null -ne $window -and $window.pluginId -eq $OldId) {
            if ($newPresent) {
                Write-Host "  window state: new id already exists; leaving old record"
            } else {
                Write-Host "  window state: $OldId -> $NewId"
                if ($Apply) {
                    $window.pluginId = $NewId
                    $changed = $true
                }
                $newPresent = $true
            }
        }
    }

    $favorites = @($document.favorites)
    $newFavorite = $false
    foreach ($favorite in $favorites) {
        if ($favorite -eq $NewId) {
            $newFavorite = $true
        }
    }
    $oldFavorite = $false
    foreach ($favorite in $favorites) {
        if ($favorite -eq $OldId) {
            $oldFavorite = $true
        }
    }
    if ($oldFavorite) {
        Write-Host "  favorites: $OldId -> $NewId"
        if ($Apply) {
            $updated = New-Object System.Collections.ArrayList
            $seenNew = $false
            foreach ($favorite in $favorites) {
                if ($favorite -eq $OldId) {
                    if (-not $seenNew) {
                        [void]$updated.Add($NewId)
                        $seenNew = $true
                    }
                } elseif ($favorite -eq $NewId) {
                    if (-not $seenNew) {
                        [void]$updated.Add($NewId)
                        $seenNew = $true
                    }
                } else {
                    [void]$updated.Add($favorite)
                }
            }
            $document.favorites = @($updated)
            $changed = $true
        }
    } elseif ($newFavorite) {
        Write-Host "  favorites: new id already present"
    }

    if ($changed) {
        Write-JsonFile $Path $document
    }
}

$root = Resolve-AnomalyRoot $AnomalyRoot
Write-Host "Anomaly root: $root"
Write-Host "Mode: $(if ($Apply) { 'APPLY' } else { 'PREVIEW (add -Apply to write)' })"
Write-Host ''

$oldConfig = Join-Path $root "config\plugins\$OldId"
$newConfig = Join-Path $root "config\plugins\$NewId"
Write-Host 'Settings:'
Copy-MissingTree $oldConfig $newConfig

$enablement = Join-Path $root 'config\plugin-enablement.json'
Write-Host 'Enablement:'
Merge-Enablement $enablement

$windowState = Join-Path $root 'state\ui-window-state.json'
Write-Host 'Window state and favorites:'
Merge-WindowState $windowState

Write-Host 'Old install directories (left in place):'
foreach ($install in @(
    (Join-Path $root "plugins\$OldId"),
    (Join-Path $root 'plugins\Nfs2015ChaseCam'))) {
    if (Test-Path -LiteralPath $install) {
        Write-Host "  $install"
    }
}
Write-Host ''
if (-not $Apply) {
    Write-Host 'No files were changed. Re-run with -Apply after checking the preview.'
} else {
    Write-Host 'Done. The old settings and install directories were not deleted.'
}
