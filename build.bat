@echo off
setlocal
if "%CXX%"=="" set "CXX=clang++"
if not exist build mkdir build

%CXX% -std=c++20 -O2 -Wall -Wextra -Iinclude src\chip.cpp tests\chip_tests.cpp -o build\chip_tests.exe
if errorlevel 1 exit /b 1
%CXX% -std=c++20 -O2 -Wall -Wextra -Iinclude src\chip.cpp examples\render.cpp -o build\chip_render.exe
if errorlevel 1 exit /b 1
%CXX% -std=c++20 -O2 -Wall -Wextra -Iinclude src\chip.cpp examples\chip_demo.cpp -o build\chip_demo.exe
if errorlevel 1 exit /b 1

build\chip_tests.exe
if errorlevel 1 exit /b 1
echo Built build\chip_tests.exe, build\chip_render.exe, build\chip_demo.exe
