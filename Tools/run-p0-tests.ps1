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
    [string]$Scenario = "all",
    [string]$UE = "C:\Program Files\Epic Games\UE_5.8",
    [string]$OutputDir = ""
)

$ErrorActionPreference = "Stop"

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
    "-CirrovaTest=$Scenario",
    "-CirrovaTestOut=$OutputDir",
    "-CirrovaTestExit=1"
)

Write-Host "Cirrova P0 自动化验证"
Write-Host "  场景：$Scenario"
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

exit $ExitCode
