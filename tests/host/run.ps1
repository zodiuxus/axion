# Host unit tests for Axion's pure logic.
#
# Compiles the pure modules from main/ plus every tests/host/*_test.cpp for the
# host and runs them. No ESP-IDF, no board, no serial port.
#
#   powershell -File tests/host/run.ps1
#
# Requires Visual Studio Build Tools (MSVC). On Linux/macOS see tests/README.md.

$ErrorActionPreference = 'Stop'

$repoRoot = Split-Path -Parent (Split-Path -Parent $PSScriptRoot)
$buildDir = Join-Path $PSScriptRoot 'build'
New-Item -ItemType Directory -Force -Path $buildDir | Out-Null

$vswhere = Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio\Installer\vswhere.exe'
if (-not (Test-Path $vswhere)) {
    throw "vswhere.exe not found. Install Visual Studio Build Tools, or compile manually (see tests/README.md)."
}

$vsPath = & $vswhere -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
if (-not $vsPath) {
    throw "No MSVC toolset found. Install the 'Desktop development with C++' workload."
}
$vsPath = $vsPath.Trim()
$vcvars = Join-Path $vsPath 'VC\Auxiliary\Build\vcvars64.bat'
if (-not (Test-Path $vcvars)) {
    throw "vcvars64.bat not found under $vsPath"
}

# Pure modules under test. Everything else in main/ needs ESP-IDF.
$sources = @(
    (Join-Path $repoRoot 'main\alert_policy.cpp'),
    (Join-Path $repoRoot 'main\alert_message.cpp')
)
# Test translation units, including run_all.cpp.
$sources += Get-ChildItem -Path $PSScriptRoot -Filter '*.cpp' | ForEach-Object { $_.FullName }

$quoted  = ($sources | ForEach-Object { '"' + $_ + '"' }) -join ' '
$include = Join-Path $repoRoot 'main'

$command = '"' + $vcvars + '" >nul 2>&1 && cl /nologo /EHsc /std:c++17 /W4 /I"' + $include + '" ' + $quoted + ' /Fe:axion_host_tests.exe'
Push-Location $buildDir
try {
    cmd /c $command
    if ($LASTEXITCODE -ne 0) {
        throw "Compilation failed (exit $LASTEXITCODE)."
    }
    & (Join-Path $buildDir 'axion_host_tests.exe')
    $testExit = $LASTEXITCODE
} finally {
    Pop-Location
}

if ($testExit -ne 0) {
    throw "Tests failed (exit $testExit)."
}
Write-Host "All host tests passed."
