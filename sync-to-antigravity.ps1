# 将 d:\AI_project\RJ_SKILLS\embedded-control-copilot 同步复制到 Antigravity 全局真实目录
# （Antigravity 在 Windows 上扫描 skills 目录时要求真实目录 d----，会忽略目录联接 Junction）
$ErrorActionPreference = "Stop"

$Source = Join-Path $PSScriptRoot "embedded-control-copilot"
$Target = Join-Path $env:USERPROFILE ".gemini\config\skills\embedded-control-copilot"

if (Test-Path $Target) {
    $item = Get-Item $Target -Force
    if ($item.LinkType) {
        [System.IO.Directory]::Delete($Target)
    } else {
        Remove-Item -Recurse -Force $Target
    }
}

Copy-Item -Recurse -Force $Source $Target
Write-Host "[OK] 已将 embedded-control-copilot 同步至 Antigravity 全局目录: $Target" -ForegroundColor Green
