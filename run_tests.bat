@echo off
setlocal EnableDelayedExpansion

set IDF_PATH=C:\esp\v5.5.3\esp-idf

if not exist "%IDF_PATH%" (
    echo ERROR: IDF_PATH not found at %IDF_PATH%
    echo Please set IDF_PATH to your ESP-IDF installation
    exit /b 1
)

set "PROJECT_PATH=%~dp0"
set "PROJECT_PATH=%PROJECT_PATH:~0,-1%"

echo ========================================
echo Running Unit Tests
echo Project: %PROJECT_PATH%
echo IDF: %IDF_PATH%
echo ========================================
echo.

cd /d "%IDF_PATH%\tools\unit-test-app"

echo Building tests...
python "%IDF_PATH%\tools\idf.py" -D EXTRA_COMPONENT_DIRS="%PROJECT_PATH%\components" build

if %ERRORLEVEL% NEQ 0 (
    echo.
    echo ERROR: Build failed!
    exit /b 1
)

echo.
echo ========================================
echo Build successful!
echo ========================================
echo.

python "%IDF_PATH%\tools\idf.py" qemu monitor

if %ERRORLEVEL% NEQ 0 (
    echo.
    echo ERROR: Build failed!
    exit /b 1
)