#Requires -Version 5.1
<#
.SYNOPSIS
    运行 Cirrova P0 自动化验证。

.DESCRIPTION
    以无头模式启动 UnrealEditor-Cmd，加载空关卡并执行指定场景。
    结果写入 Saved/CirrovaTests/<时间戳>，命令退出码即测试总判定。

.PARAMETER Scenario
    场景名：all、T0、T0,T1 ...（逗号分隔）。

.PARAMETER UE
    Unreal Engine 5.8 安装目录。

.PARAMETER OutputDir
    输出目录；省略时使用 Saved/CirrovaTests/<时间戳>。
#>
[CmdletBinding()]
param(
    [string[]]$Scenario = @("all"),
    [string]$UE = "C:\Program Files\Epic Games\UE_5.8",
    [string]$OutputDir = ""
)

$ErrorActionPreference = "Stop"

# 多场景参数在 PowerShell 下会绑定为数组，拼接为引擎接受的逗号分隔列表。
$ScenarioList = @($Scenario | Where-Object { -not [string]::IsNullOrWhiteSpace($_) })
if ($ScenarioList.Count -eq 0) { $ScenarioList = @("all") }
$ScenarioText = $ScenarioList -join ","

$ProjectRoot = Split-Path -Parent (Split-Path -Parent (Split-Path -Parent $PSScriptRoot))
$Project = Join-Path $ProjectRoot "Vehicle.uproject"
$Editor = Join-Path $UE "Engine\Binaries\Win64\UnrealEditor-Cmd.exe"

if (-not (Test-Path $Editor)) {
    throw "未找到 UnrealEditor-Cmd.exe：$Editor"
}
if (-not (Test-Path $Project)) {
    throw "未找到项目：$Project"
}
if ([string]::IsNullOrWhiteSpace($OutputDir)) {
    $OutputDir = Join-Path $ProjectRoot ("Saved\CirrovaTests\" + (Get-Date -Format "yyyyMMdd-HHmmss"))
}

$Arguments = @(
    $Project,
    "-game",
    "-nullrhi",
    "-unattended",
    "-nosplash",
    "-log",
    "-Deterministic",
    "-FPS=60",
    "-map=/Engine/Maps/Entry",
    "-CirrovaTest=$ScenarioText",
    "-CirrovaTestOut=$OutputDir",
    "-CirrovaTestExit=1"
)

Write-Host "Cirrova P0 自动化验证"
Write-Host "  场景：$ScenarioText"
Write-Host "  输出：$OutputDir"
Write-Host ""

& $Editor @Arguments
$ExitCode = $LASTEXITCODE

$Summary = Join-Path $OutputDir "summary.txt"
if (Test-Path $Summary) {
    Get-Content $Summary
}
else {
    Write-Warning "未找到 summary.txt：$Summary"
}

$Report = Join-Path $OutputDir "report.json"
if (-not (Test-Path $Report)) {
    Write-Warning "未找到 report.json：$Report"
    exit $ExitCode
}

$Data = Get-Content -LiteralPath $Report -Raw | ConvertFrom-Json
$Scenarios = @($Data.scenarios)
$Passed = @($Scenarios | Where-Object { $_.passed }).Count
$Failed = @($Scenarios | Where-Object { -not $_.passed })

Write-Host ""
Write-Host ("场景 {0}：通过 {1}，失败 {2}" -f $Scenarios.Count, $Passed, $Failed.Count)
foreach ($Item in $Failed) {
    Write-Host ("  失败 {0}：{1}" -f $Item.name, $Item.summary)
}

$PluginErrors = @(Select-String -Path (Join-Path $ProjectRoot "Saved\Logs\Vehicle.log") -Pattern 'LogCirrova: (Warning|Error):' -ErrorAction SilentlyContinue)
Write-Host ("LogCirrova 告警/错误：{0}" -f $PluginErrors.Count)

if (-not $Data.all_passed -or $PluginErrors.Count -gt 0) {
    exit 1
}

exit $ExitCode
