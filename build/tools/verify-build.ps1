$ErrorActionPreference = 'Stop'
$taskRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '../..')).Path
$taskBuild = Join-Path $taskRoot 'build/Debug'
$taskEntries = @(Get-Content -LiteralPath (Join-Path $taskBuild 'compile_commands.json') -Raw | ConvertFrom-Json)
$taskTemplate = $taskEntries | Where-Object { $_.file -like '*/Application/shoot_test.c' } | Select-Object -First 1
$taskPitch = [pscustomobject]@{
    command = $taskTemplate.command.Replace('shoot_test.c', 'pitch_test.c')
    output = $taskTemplate.output.Replace('shoot_test.c', 'pitch_test.c')
}
$taskEntries += $taskPitch
$taskObjects = @()
Push-Location -LiteralPath $taskBuild
try {
    foreach ($taskEntry in $taskEntries) {
        $taskTokens = @([regex]::Matches($taskEntry.command, '"[^"]*"|\S+') | ForEach-Object { $_.Value.Trim('"') })
        $taskCompiler = $taskTokens[0]
        $taskCompilerArgs = $taskTokens[1..($taskTokens.Count - 1)]
        New-Item -ItemType Directory -Path (Split-Path -Parent $taskEntry.output) -Force | Out-Null
        & $taskCompiler @taskCompilerArgs
        if ($LASTEXITCODE -ne 0) { throw "编译失败：$($taskEntry.output)" }
        $taskObjects += $taskEntry.output
    }
    $taskLinkArgs = @('-mcpu=cortex-m4', '-mfpu=fpv4-sp-d16', '-mfloat-abi=hard', '-O0', '-g3',
        '-T', (Join-Path $taskRoot 'STM32F407XX_FLASH.ld'), '--specs=nano.specs',
        '-Wl,-Map=shaobing_yuntai.map', '-Wl,--gc-sections', '-Wl,--print-memory-usage',
        '-o', 'shaobing_yuntai.elf')
    $taskLinkArgs += $taskObjects
    $taskLinkArgs += @('-L' + (Join-Path $taskRoot 'Middlewares/ST/ARM/DSP/Lib'), '-l:libarm_cortexM4lf_math.a', '-lm')
    & $taskCompiler @taskLinkArgs
    if ($LASTEXITCODE -ne 0) { throw '链接失败' }
    Write-Output "已编译并链接 $($taskEntries.Count) 个源文件：$taskBuild/shaobing_yuntai.elf"
} finally {
    Pop-Location
}
