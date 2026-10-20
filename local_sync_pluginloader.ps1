# Local deployment helper. Run from any working directory with PowerShell.
[CmdletBinding()]
param(
    [string]$SourceRoot = '',
    [string]$PluginLoaderRoot = 'D:\_Code_Here\Git\MayaTools\vaccineKiller\pluginLoader'
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

try {
    # Windows PowerShell 5.1 may evaluate parameter defaults before PSScriptRoot
    # is available when launched with -File. Resolve it in the script body.
    if ([string]::IsNullOrWhiteSpace($SourceRoot)) {
        $SourceRoot = Split-Path -Path $PSCommandPath -Parent
    }
    $sourcePath = (Resolve-Path -LiteralPath $SourceRoot).ProviderPath
    $loaderPath = [IO.Path]::GetFullPath($PluginLoaderRoot)
    if (-not (Test-Path -LiteralPath $loaderPath -PathType Container)) {
        throw "PluginLoader directory does not exist: $loaderPath"
    }

    $scriptSource = Join-Path $sourcePath 'scripts'
    if (-not (Test-Path -LiteralPath $scriptSource -PathType Container)) {
        throw "Scripts directory is missing: $scriptSource"
    }

    $pluginSource = Join-Path $sourcePath 'plug-ins'
    $pluginCopies = @(
        foreach ($version in Get-ChildItem -LiteralPath $pluginSource -Directory) {
            if ($version.Name -notmatch '^\d{4}$') { continue }
            foreach ($plugin in Get-ChildItem -LiteralPath $version.FullName -Filter '*.mll' -File) {
                [PSCustomObject]@{
                    Source = $plugin.FullName
                    Destination = Join-Path $loaderPath ("maya_{0}\Plug-ins\{1}" -f $version.Name, $plugin.Name)
                    Kind = 'MLL'
                }
            }
        }
    )
    if ($pluginCopies.Count -eq 0) {
        throw "No versioned MLL files found under: $pluginSource"
    }

    $scriptsPath = Join-Path $loaderPath 'scripts'
    $scriptPrefix = $scriptSource.TrimEnd('\', '/') + [IO.Path]::DirectorySeparatorChar
    $scriptCopies = @(
        foreach ($script in Get-ChildItem -LiteralPath $scriptSource -Recurse -File) {
            $relativePath = $script.FullName.Substring($scriptPrefix.Length)
            if ($relativePath -match '(^|[\\/])__pycache__([\\/]|$)' -or
                $script.Extension -in @('.pyc', '.pyo')) { continue }
            [PSCustomObject]@{
                Source = $script.FullName
                Destination = Join-Path $scriptsPath $relativePath
                Kind = 'Script'
            }
        }
    )
    if ($scriptCopies.Count -eq 0) {
        throw "No scripts found under: $scriptSource"
    }

    $copies = @($pluginCopies) + @($scriptCopies)
    # Validate every exact destination before making any changes.
    $loaderPrefix = $loaderPath.TrimEnd('\', '/') + [IO.Path]::DirectorySeparatorChar
    foreach ($copy in $copies) {
        $destination = [IO.Path]::GetFullPath($copy.Destination)
        if (-not $destination.StartsWith($loaderPrefix, [StringComparison]::OrdinalIgnoreCase)) {
            throw "Destination is outside PluginLoader: $destination"
        }
        if ($destination.Equals($copy.Source, [StringComparison]::OrdinalIgnoreCase)) {
            throw "Source and destination must be different: $destination"
        }
        # Include the destination file itself when checking junctions/symlinks.
        $cursor = $destination
        while ($cursor) {
            if (Test-Path -LiteralPath $cursor) {
                $item = Get-Item -LiteralPath $cursor -Force
                if ($item.Attributes -band [IO.FileAttributes]::ReparsePoint) {
                    throw "Destination path contains a junction or symlink: $cursor"
                }
            }
            $cursor = Split-Path -Path $cursor -Parent
        }
    }

    foreach ($copy in $copies) {
        $directory = Split-Path -Path $copy.Destination -Parent
        New-Item -ItemType Directory -Path $directory -Force | Out-Null
        Copy-Item -LiteralPath $copy.Source -Destination $copy.Destination -Force
        Write-Host ("[{0}] {1} -> {2}" -f $copy.Kind, $copy.Source, $copy.Destination)
    }

    Write-Host ("[OK] Synced {0} MLL file(s) and {1} script file(s)." -f $pluginCopies.Count, $scriptCopies.Count)
}
catch {
    Write-Error -Message ("{0}`n{1}" -f $_.Exception.Message, $_.ScriptStackTrace) -ErrorAction Continue
    exit 1
}
