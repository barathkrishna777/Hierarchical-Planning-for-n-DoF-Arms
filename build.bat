@echo off
setlocal enabledelayedexpansion

echo Compiling planner.cpp...
g++ planner.cpp -o planner.exe
if errorlevel 1 goto :error
echo ✔ planner.exe built

echo Compiling verifier.cpp...
g++ verifier.cpp -o verifier.exe
if errorlevel 1 goto :error
echo ✔ verifier.exe built

echo Compiling config_checker.cpp...
g++ config_checker.cpp -o config_checker.exe
if errorlevel 1 goto :error
echo ✔ config_checker.exe built

echo ✅ All files compiled successfully.
goto :eof

:error
echo ❌ Compilation failed.
exit /b 1