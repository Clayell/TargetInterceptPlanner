# put this in .git

$ProjectRoot = Split-Path -Parent $PSScriptRoot

$AssemblyPath = Join-Path $ProjectRoot "bin\Release\TargetInterceptPlanner.dll"
$SourceDir    = Join-Path $ProjectRoot "GameData"
$OutputDir    = Join-Path $ProjectRoot ".git"

if (-not (Test-Path $AssemblyPath)) {
    Write-Error "Assembly not found: $AssemblyPath"
    exit 1
}

if (-not (Test-Path $SourceDir)) {
    Write-Error "Source directory not found: $SourceDir"
    exit 1
}

$Version = (Get-Item $AssemblyPath).VersionInfo.FileVersion

if (-not $Version) {
    Write-Error "AssemblyFileVersion not found"
    exit 1
}

$OutputZip = Join-Path $OutputDir "TargetInterceptPlanner-$Version.zip"

if (Test-Path $OutputZip) {
    Remove-Item $OutputZip
}

Compress-Archive -Path $SourceDir -DestinationPath $OutputZip

Write-Host "Created $OutputZip"