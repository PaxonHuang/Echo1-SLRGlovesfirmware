@echo off
echo Starting ESP-IDF Hand Signal Recognition System Build Verification
echo.

REM Check if ESP-IDF environment is set
if not defined IDF_PATH (
    echo ERROR: ESP-IDF environment not found. Please run get_idf.bat first.
    pause
    exit /b 1
)

echo IDF_PATH: %IDF_PATH%
echo.

REM Navigate to project directory
cd /d "E:\NJTS-Codeprojects-2023\NJTS-HandSignal-Cloud-sEMG\Arduino2IDFtest\esp-idf-handsignal"

echo Building ESP-IDF project...
echo.

REM Clean and build
idf.py fullclean
idf.py build

if %ERRORLEVEL% EQU 0 (
    echo.
    echo ========================================
    echo BUILD SUCCESSFUL!
    echo ========================================
    echo.
    echo The Arduino firmware has been successfully converted to ESP-IDF.
    echo.
    echo Next steps:
    echo 1. Connect your ESP32 development board
    echo 2. Run: idf.py -p COMX flash monitor
    echo 3. Verify serial output format matches PyQt expectations
    echo.
) else (
    echo.
    echo ========================================
    echo BUILD FAILED!
    echo ========================================
    echo.
    echo Please check the error messages above and fix any issues.
    echo.
)

pause