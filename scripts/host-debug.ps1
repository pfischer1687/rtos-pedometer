$ErrorActionPreference = "Stop"

Write-Host "Host Debug Build" -ForegroundColor Cyan

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$RepoRoot  = Resolve-Path (Join-Path $ScriptDir "..")

$TestDir   = Join-Path $RepoRoot "test"
$BuildDir  = Join-Path $TestDir "build"
$VSCodeDir = Join-Path $RepoRoot ".vscode"

Push-Location $TestDir

if (Test-Path $BuildDir) {
    Write-Host "Removing old build directory..." -ForegroundColor Yellow
    Remove-Item $BuildDir -Recurse -Force
}

Write-Host "Configuring with CMakePresets" -ForegroundColor Green
cmake --preset host-debug

Write-Host "Building Debug" -ForegroundColor Green
cmake --build --preset host-debug

Pop-Location

Write-Host "Switching VS Code launch.json to host version" -ForegroundColor Green
Copy-Item `
    (Join-Path $VSCodeDir "launch_host.json") `
    (Join-Path $VSCodeDir "launch.json") `
    -Force

Write-Host "Done. Press F5 to start debugging." -ForegroundColor Cyan
