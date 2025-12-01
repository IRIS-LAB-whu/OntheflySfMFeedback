@echo off

REM 设置要搜索的目录路径
set "target_dir=C:/Program Files/NVIDIA GPU Computing Toolkit/CUDA/v11.8/lib/x64"

REM 遍历目录并输出.lib文件的完整名称
for /r "%target_dir%" %%f in (*.lib) do (
    echo %%~nxf
)

pause