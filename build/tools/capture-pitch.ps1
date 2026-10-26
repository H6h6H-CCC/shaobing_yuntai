param([int]$Seconds = 25, [string]$Port = 'COM16', [int]$ExpectedKp = 0,
      [int]$ExpectedSpeedKp = 0, [int]$ExpectedKiMilli = 0, [int]$ExpectedRate = 0,
      [int]$ExpectedAngleKi = 0)

# 直接采集串口；完整数据保存在本工程build内，控制台每0.5秒显示一次。
$logDir = Join-Path $PSScriptRoot '../pitch-logs'
[System.IO.Directory]::CreateDirectory($logDir) | Out-Null
$logPath = Join-Path $logDir ('pitch-' + (Get-Date -Format 'yyyyMMdd-HHmmss') + '.log')
$writer = [System.IO.StreamWriter]::new($logPath, $false)
$serial = [System.IO.Ports.SerialPort]::new($Port, 115200, 'None', 8, 'One')
$serial.ReadTimeout = 200
$timer = [System.Diagnostics.Stopwatch]::StartNew()
$records = [System.Collections.Generic.List[object]]::new()
$lastPrint = -1.0
try {
    $serial.Open()
    $serial.DiscardInBuffer()
    while ($timer.Elapsed.TotalSeconds -lt $Seconds) {
        try { $line = $serial.ReadLine().Trim() }
        catch [System.TimeoutException] { continue }
        $writer.WriteLine($line)
        # 串级版追加目标/滤波转速、积分、控制周期和内环参数，仍兼容单P历史数据。
        if ($line -match '^PIT:ms=(\d+) en=(\d+) rc=(\d+)/(\d+) on=(\d+) ch=(-?\d+) a10=(-?\d+) t10=(-?\d+) e10=(-?\d+) rpm=(-?\d+) cmd=(-?\d+) fbI=-?\d+ T=\d+ lim=-?\d+ age=\d+ kp10=(\d+) txfail=\d+ err=0x[0-9A-Fa-f]+(?: tr10=(-?\d+) fr10=(-?\d+) iout=(-?\d+) dtmax=(\d+) sp10=(\d+) ki1000=(\d+) ctl=PI(?: fault=(\d+))?)?(?: vmax10=(?<rate10>\d+))?(?: aki10=(?<angleki10>\d+) iout=(?<angleiout>-?\d+))?$') {
            # USB串口可能残留上一版数据，只统计当前待验证参数。
            if (($ExpectedKp -gt 0) -and ([int]$Matches[12] -ne $ExpectedKp * 10)) { continue }
            if (($ExpectedSpeedKp -gt 0) -and ([int]$Matches[17] -ne $ExpectedSpeedKp * 10)) { continue }
            if (($ExpectedKiMilli -gt 0) -and ([int]$Matches[18] -ne $ExpectedKiMilli)) { continue }
            # 同P但目标速度不同时也过滤旧缓存，避免把25度/s混入40度/s的测试。
            if (($ExpectedRate -gt 0) -and ([int]$Matches['rate10'] -ne $ExpectedRate * 10)) { continue }
            # 位置小积分版独立校验KI，不与旧串级速度环的KI混用。
            if (($ExpectedAngleKi -gt 0) -and ([int]$Matches['angleki10'] -ne $ExpectedAngleKi * 10)) { continue }
            $records.Add([pscustomobject]@{
                ms=[int]$Matches[1]; en=[int]$Matches[2]; ch=[int]$Matches[6]
                a10=[int]$Matches[7]; t10=[int]$Matches[8]; e10=[int]$Matches[9]
                rpm=[int]$Matches[10]; cmd=[int]$Matches[11]
            })
            if (($timer.Elapsed.TotalSeconds - $lastPrint) -ge 0.5) {
                Write-Output $line
                $lastPrint = $timer.Elapsed.TotalSeconds
            }
        }
    }
} finally {
    if ($serial.IsOpen) { $serial.Close() }
    $serial.Dispose()
    $writer.Dispose()
}
$active = @($records | Where-Object en -eq 1)
$moving = @($active | Where-Object { [Math]::Abs($_.ch) -gt 40 })
Write-Output "LOG=$logPath samples=$($records.Count) enabled=$($active.Count) stick=$($moving.Count)"
if ($active.Count -gt 0) {
    $active | Measure-Object -Property a10,t10,rpm,cmd -Minimum -Maximum |
        Select-Object Property,Minimum,Maximum | Format-Table
}
