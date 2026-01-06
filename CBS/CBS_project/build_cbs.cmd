@echo off
setlocal

REM 进入脚本所在目录（也就是工程根目录）这样无论把 CBS_project 拷到哪里，都能编译
cd /d "%~dp0"

REM 用 MSYS2 bash 执行编译（通配符由 bash 展开）
D:\msys64\usr\bin\bash.exe -lc "/d/msys64/ucrt64/bin/g++.exe -std=c++17 -g src/*.cpp -Iinclude -o cbs.exe"

endlocal
