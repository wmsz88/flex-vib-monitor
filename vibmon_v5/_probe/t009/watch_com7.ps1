# 监听 COM7（板子自带 USB / USB-Serial-JTAG），把收到的字节实时打出来
# 用法: powershell -ExecutionPolicy Bypass -File .\watch_com7.ps1
# 运行后按一下板子上的 RST 键，正常会立刻打印启动横幅
param([string]$Port = "COM7", [int]$Secs = 30)

python -c @"
import serial, time, sys
p = serial.Serial(r'$Port', 115200, timeout=0.2)
print('listening on $Port for $Secs s ... press RST now')
t0 = time.time()
n = 0
while time.time() - t0 < $Secs:
    d = p.read(4096)
    if d:
        n += len(d)
        sys.stdout.write(d.decode('utf-8','replace')); sys.stdout.flush()
p.close()
print('\n--- total %d bytes ---' % n)
"@
