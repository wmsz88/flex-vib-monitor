# T009 烧录重试脚本  (ESP32-C3 + 外接 CH340 转串口)
#
# 背景：电脑上只有 COM3 = "USB-SERIAL CH340"，板子自带的 USB 口没有枚举出来。
#       走外接 CH340 走 UART 烧录时，CH340 的 DTR/RTS 没有接到板子的 EN/GPIO9，
#       所以 esptool 的自动复位进下载模式是失效的 —— 必须手动进下载模式。
#
# 用法：
#   powershell -ExecutionPolicy Bypass -File .\flash_retry.ps1
#
# 运行后按提示操作：按住 BOOT(GPIO9) -> 点一下 RST(EN) -> 松开 BOOT
# 脚本会每 2 秒重试一次，抓到芯片就自动烧进去。
#
# 接线（务必核对，TX/RX 是交叉的）：
#   CH340 TX  -> 板子 GPIO20   (芯片的 RXD)
#   CH340 RX  -> 板子 GPIO21   (芯片的 TXD)
#   CH340 GND -> 板子 GND      (必须共地!)
#   板子供电单独给（12V 模块那路）

param(
    [string]$Port  = "COM3",
    [int]   $Tries = 40,
    [switch]$Cdc
)

$cli   = "C:\Users\Lenovo\AppData\Local\Programs\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe"
# 默认 = USB CDC On Boot: Disabled -> Serial 走 UART0(GPIO20/21) -> CH340 看得见
# -Cdc  = USB CDC On Boot: Enabled  -> Serial 走板子自带 USB 口
if ($Cdc) {
    $fqbn  = "esp32:esp32:esp32c3:CDCOnBoot=cdc"
    $build = Join-Path $PSScriptRoot "build"
} else {
    $fqbn  = "esp32:esp32:esp32c3"
    $build = Join-Path $PSScriptRoot "build_uart"
}
$sketch = "C:\Users\Lenovo\Desktop\STM32project\vibsource_c3_key"

if (-not (Test-Path $build)) {
    Write-Host "找不到编译产物目录: $build" -ForegroundColor Red
    Write-Host "先跑一次编译生成它。" -ForegroundColor Red
    exit 1
}

Write-Host "端口 = $Port    最多重试 $Tries 次" -ForegroundColor Cyan
Write-Host "请现在开始: 按住 BOOT(GPIO9) -> 点一下 RST(EN) -> 松开 BOOT" -ForegroundColor Yellow
Write-Host ""

for ($i = 1; $i -le $Tries; $i++) {
    Write-Host ("--- 第 {0}/{1} 次 ---" -f $i, $Tries) -ForegroundColor DarkGray
    $out = & $cli upload -p $Port -b $fqbn --input-dir $build $sketch 2>&1 | Out-String

    if ($out -match "Hash of data verified|Hard resetting|Leaving\.\.\.") {
        Write-Host "=== 烧录成功 ===" -ForegroundColor Green
        $out
        exit 0
    }

    Write-Host "还没连上芯片，继续按住 BOOT 点 RST 重试 ..." -ForegroundColor DarkYellow
    Start-Sleep -Seconds 2
}

Write-Host "=== $Tries 次都失败 ===" -ForegroundColor Red
Write-Host "检查: 1) TX->GPIO20 / RX->GPIO21 是否交叉正确" -ForegroundColor Red
Write-Host "      2) GND 是否共地" -ForegroundColor Red
Write-Host "      3) 板子是否单独供上电" -ForegroundColor Red
$out
exit 1
