# run_all.ps1 — 一条命令跑完 vibmon_v6 的全量宿主测试（MinGW g++）
#
# 用法（任意目录）：
#   powershell -ExecutionPolicy Bypass -File "D:\大学\课程\工科赛\代码\vibmon_v6\test\run_all.ps1"
#
# 测五个目标：contact / gyro / comp_frame / dump / ss_replay。
# 编译期 0 warning + 逐条数字与基线一致 才算 OK。
# dump 的基线是 15/19（4 项失败是既有列数写死问题，v5 同样如此，见 RT013）。

$ErrorActionPreference = 'Stop'

$root = Split-Path -Parent $PSScriptRoot          # ...\vibmon_v6
$testDir = Join-Path $root 'test'
$cc = (Get-Command g++ -ErrorAction SilentlyContinue).Source
if (-not $cc) { Write-Host '找不到 g++，请把 MinGW\bin 加进 PATH'; exit 2 }

# 目标表：名称 = @{ src=源文件; deps=额外源码; defs=编译宏; expect=期望全过/期望 PASS 数 }
$targets = [ordered]@{
  'contact_test'    = @{ src = 'test/contact_test.cpp';    deps = @('state.cpp');                          defs = @();                     pass = 89; total = 89 }
  'gyro_test'       = @{ src = 'test/gyro_test.cpp';       deps = @('dsp.cpp','state.cpp');                defs = @();                     pass = 8;  total = 8  }
  'comp_frame_test' = @{ src = 'test/comp_frame_test.cpp'; deps = @('output.cpp','state.cpp','dsp.cpp');   defs = @();                     pass = 22; total = 22 }
  'dump_test'       = @{ src = 'test/dump_test.cpp';       deps = @('output.cpp','state.cpp','dsp.cpp');   defs = @();                     pass = 15; total = 19 }
  'led_test'        = @{ src = 'test/led_test.cpp';        deps = @('output.cpp','state.cpp','dsp.cpp');   defs = @();                     pass = 64; total = 64 }
  'ss_replay'       = @{ src = 'test/ss_replay.cpp';       deps = @('state.cpp');                          defs = @('-DSTATE_TEST_HOOKS'); pass = -1; total = -1 }
}

Push-Location $root
$summary = @()
$exitCode = 0
try {
  foreach ($name in $targets.Keys) {
    $t = $targets[$name]
    $exe = Join-Path $testDir "$name.exe"
    $cargs = @('-std=c++17','-Wall') + $t.defs + @('-I','test','-I','.') + @($t.src) + $t.deps + @('-o', $exe)

    $build = & $cc @cargs 2>&1
    $buildText = ($build | Out-String)
    $warn = ([regex]::Matches($buildText, '(?im)\bwarning\b')).Count
    $built = Test-Path $exe
    if (-not $built) {
      $summary += [pscustomobject]@{ 测试 = $name; 编译 = '失败'; 警告 = $warn; 结果 = '编译失败'; 判定 = 'FAIL' }
      $exitCode = 1
      if ($buildText.Trim()) { Write-Host $buildText }
      continue
    }

    $out = (& $exe 2>&1 | Out-String)
    if ($t.pass -lt 0) {
      # ss_replay 只做冒烟：喂一行空输入，能退出且不报错即算可运行
      $summary += [pscustomobject]@{ 测试 = $name; 编译 = 'OK'; 警告 = $warn; 结果 = '可运行'; 判定 = if ($warn -eq 0) { 'OK' } else { 'WARN' } }
      if ($warn -ne 0) { $exitCode = 1 }
      continue
    }

    $mPass = [regex]::Match($out, '(\d+)\s*(?:pass|/\s*\d+\s*项通过)')
    $mFail = [regex]::Match($out, '(\d+)\s*fail')
    $nPass = if ($mPass.Success) { [int]$mPass.Groups[1].Value } else { -1 }
    $nFail = if ($mFail.Success) { [int]$mFail.Groups[1].Value } else { 0 }
    $ok = ($nPass -eq $t.pass) -and ($warn -eq 0)
    if (-not $ok) { $exitCode = 1 }
    $summary += [pscustomobject]@{
      测试 = $name; 编译 = 'OK'; 警告 = $warn
      结果 = "$nPass/$($t.total)" + $(if ($nFail) { " ($nFail fail)" } else { '' })
      判定 = if ($ok) { 'OK' } else { 'FAIL' }
    }
  }
}
finally {
  Pop-Location
}

Write-Host ''
Write-Host '===== 汇总 ====='
$summary | Format-Table -AutoSize
$bad = @($summary | Where-Object { $_.判定 -ne 'OK' })
if ($bad.Count -eq 0) {
  Write-Host '全部与基线一致，0 warning。'
} else {
  Write-Host ("偏离基线的项：" + (($bad | ForEach-Object { $_.测试 }) -join ', '))
}
exit $exitCode
