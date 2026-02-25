<#
.SYNOPSIS
    Check installed RoadBLD UHT headers against the AAA_Traffic API baseline.

.DESCRIPTION
    Scans RoadBLD's UHT-generated headers (*.generated.h) for the classes,
    functions, and properties that AAA_Traffic's reflection provider depends on.
    Reports any missing symbols so developers can detect breaking changes
    before they show up as runtime failures.

    This script is offline / CI-safe — it reads .generated.h text files only
    and does not require Unreal Engine to be running.

.PARAMETER RoadBLDIntermediatePath
    Path to RoadBLD's Intermediate/Build directory containing generated headers.
    If omitted, the script searches common locations.

.PARAMETER BaselinePath
    Path to RoadBLD_API_Baseline.json.  Defaults to the Scripts/ folder next
    to this script.

.EXAMPLE
    .\CheckRoadBLDCompat.ps1
    .\CheckRoadBLDCompat.ps1 -RoadBLDIntermediatePath "D:\Plugins\RoadBLD\Intermediate\Build\Win64\UnrealEditor\Inc\RoadBLDRuntime"
#>

[CmdletBinding()]
param(
    [string]$RoadBLDIntermediatePath,
    [string]$BaselinePath
)

$ErrorActionPreference = 'Stop'

# ── Resolve baseline path ────────────────────────────────────────────────
if (-not $BaselinePath) {
    $BaselinePath = Join-Path $PSScriptRoot 'RoadBLD_API_Baseline.json'
}

if (-not (Test-Path $BaselinePath)) {
    Write-Error "Baseline file not found: $BaselinePath"
    exit 1
}

$baseline = Get-Content $BaselinePath -Raw | ConvertFrom-Json
Write-Host "Loaded baseline for RoadBLD v$($baseline.version) (captured $($baseline.captured))" -ForegroundColor Cyan

# ── Resolve RoadBLD intermediate path ────────────────────────────────────
if (-not $RoadBLDIntermediatePath) {
    # Search common locations relative to workspace root.
    $searchRoots = @(
        (Join-Path $PSScriptRoot '..\..\..\..\RoadBLD\Intermediate\Build'),
        (Join-Path $PSScriptRoot '..\..\..\Plugins\RoadBLD\Intermediate\Build')
    )
    foreach ($root in $searchRoots) {
        $candidates = Get-ChildItem -Path $root -Filter '*.generated.h' -Recurse -ErrorAction SilentlyContinue | Select-Object -First 1
        if ($candidates) {
            $RoadBLDIntermediatePath = Split-Path $candidates.FullName -Parent
            break
        }
    }
}

if (-not $RoadBLDIntermediatePath -or -not (Test-Path $RoadBLDIntermediatePath)) {
    Write-Warning "RoadBLD generated headers not found. Provide -RoadBLDIntermediatePath."
    Write-Warning "Skipping header scan. Runtime API contract validation (in-engine) will still catch issues."
    exit 0
}

Write-Host "Scanning headers in: $RoadBLDIntermediatePath" -ForegroundColor Cyan

# ── Gather all generated header content ──────────────────────────────────
$allHeaders = Get-ChildItem -Path $RoadBLDIntermediatePath -Filter '*.generated.h' -Recurse
$headerContent = @{}
foreach ($h in $allHeaders) {
    $headerContent[$h.BaseName] = Get-Content $h.FullName -Raw
}

Write-Host "Found $($allHeaders.Count) generated header(s)." -ForegroundColor Cyan

# ── Check functions and properties ───────────────────────────────────────
$failures = @()
$passes = 0

foreach ($className in $baseline.classes.PSObject.Properties.Name) {
    $classSpec = $baseline.classes.$className

    # Narrow search to the class-specific generated header when available.
    # UHT generates <ClassName>.generated.h — searching only that file avoids
    # false positives from identically-named symbols on unrelated classes.
    $scopedContent = $headerContent
    if ($headerContent.ContainsKey($className)) {
        $scopedContent = @{ $className = $headerContent[$className] }
    } else {
        Write-Warning "No header matching '$className.generated.h' found — falling back to broad search."
    }

    # Check functions.
    foreach ($func in $classSpec.functions) {
        $funcName = $func.name
        $found = $false
        foreach ($content in $scopedContent.Values) {
            if ($content -match "\b$funcName\b") {
                $found = $true
                break
            }
        }
        if ($found) {
            $passes++
        } else {
            $failures += [PSCustomObject]@{
                Class    = $className
                Category = 'UFunction'
                Symbol   = $funcName
                Status   = 'MISSING'
            }
        }
    }

    # Check properties.
    foreach ($prop in $classSpec.properties) {
        $propName = $prop.name
        $found = $false
        foreach ($content in $scopedContent.Values) {
            if ($content -match "\b$propName\b") {
                $found = $true
                break
            }
        }
        if ($found) {
            $passes++
        } else {
            $failures += [PSCustomObject]@{
                Class    = $className
                Category = 'FProperty'
                Symbol   = $propName
                Status   = 'MISSING'
            }
        }
    }
}

# Check struct properties.
if ($baseline.structs) {
    foreach ($structName in $baseline.structs.PSObject.Properties.Name) {
        $structSpec = $baseline.structs.$structName
        foreach ($prop in $structSpec.properties) {
            $propName = $prop.name
            $found = $false
            foreach ($content in $headerContent.Values) {
                if ($content -match "\b$propName\b") {
                    $found = $true
                    break
                }
            }
            if ($found) {
                $passes++
            } else {
                $failures += [PSCustomObject]@{
                    Class    = $structName
                    Category = 'FProperty'
                    Symbol   = $propName
                    Status   = 'MISSING'
                }
            }
        }
    }
}

# ── Report ───────────────────────────────────────────────────────────────
Write-Host ""
Write-Host "Results: $passes passed, $($failures.Count) failed" -ForegroundColor $(if ($failures.Count -eq 0) { 'Green' } else { 'Red' })

if ($failures.Count -gt 0) {
    Write-Host ""
    Write-Host "FAILURES:" -ForegroundColor Red
    $failures | Format-Table -AutoSize | Out-String | Write-Host
    Write-Host "The installed RoadBLD version may have breaking API changes." -ForegroundColor Yellow
    Write-Host "Update RoadBLD_API_Baseline.json after verifying the reflection provider still works." -ForegroundColor Yellow
    exit 1
}

Write-Host "All baseline symbols found in RoadBLD generated headers." -ForegroundColor Green
exit 0
