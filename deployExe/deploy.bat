@echo off
setlocal EnableDelayedExpansion

::  ROBOT DEPLOY SCRIPT
::  Pulls latest code, connects to robot WiFi, then deploys via WPILib

::  CONFIGURATION (edit these)
set WIFI_SSID=FRC-7476-EXTRA
set TEAM_NUMBER=7476
set DEPLOY_BRANCH=main
set REPO_NAME=2026-7476-Rebuilt
set JAVA_HOME=C:\Users\Public\wpilib\2026\jdk

set PATH=%JAVA_HOME%\bin;%PATH%

title Robot Deploy Script - Team %TEAM_NUMBER%

echo.
echo ROBOT DEPLOY SCRIPT  ^|  Team %TEAM_NUMBER%
echo.

::FIND REPO
echo [*] Searching for repo "%REPO_NAME%"...

set REPO_PATH=

:: Check common locations first (fast)
for %%D in (
    "%USERPROFILE%\Documents\GitHub\%REPO_NAME%"
    "%USERPROFILE%\Documents\%REPO_NAME%"
    "%USERPROFILE%\Desktop\%REPO_NAME%"
    "%USERPROFILE%\Documents\FRC\%REPO_NAME%"
    "%USERPROFILE%\source\repos\%REPO_NAME%"
    "%USERPROFILE%\OneDrive\Documents\GitHub\%REPO_NAME%"
    "%USERPROFILE%\OneDrive\Documents\%REPO_NAME%"
    "C:\%REPO_NAME%"
    "C:\Users\Public\%REPO_NAME%"
) do (
    if exist "%%~D\.git" (
        set REPO_PATH=%%~D
        goto :found_repo
    )
)

:: Full drive search as fallback (slower)
echo [*] Not found in common locations, scanning drives...
for %%D in (C D E F G) do (
    if exist "%%D:\" (
        for /f "delims=" %%F in ('dir /b /s /ad "%%D:\%REPO_NAME%" 2^>nul') do (
            if exist "%%F\.git" (
                set REPO_PATH=%%F
                goto :found_repo
            )
        )
    )
)

echo [ERROR] Could not find repo "%REPO_NAME%" anywhere on this computer.
echo         Please clone it first from GitHub.
pause
exit /b 1

:found_repo
echo [OK] Found repo at: %REPO_PATH%
echo.

set GRADLEW=%REPO_PATH%\gradlew.bat

:: STEP 1: Pull Latest Code from GitHub
echo [1/3] Pulling latest code from GitHub...
echo.

cd /d "%REPO_PATH%"

git rev-parse --is-inside-work-tree >nul 2>&1
if %errorlevel% neq 0 (
    echo [ERROR] "%REPO_PATH%" is not a valid Git repository.
    goto :error
)

echo  Switching to %DEPLOY_BRANCH% branch...
git checkout %DEPLOY_BRANCH%
if %errorlevel% neq 0 (
    echo [ERROR] Could not switch to %DEPLOY_BRANCH% branch.
    goto :error
)

git pull origin %DEPLOY_BRANCH%
if %errorlevel% neq 0 (
    echo [ERROR] Git pull failed. Check your internet connection or repo status.
    goto :error
)
echo.
echo [OK] Repository is up to date on %DEPLOY_BRANCH%.
echo.

:: STEP 2: Connect to Robot WiFi
echo [2/3] Connecting to WiFi: "%WIFI_SSID%"...
echo.

netsh wlan disconnect >nul 2>&1
timeout /t 2 /nobreak >nul

powershell -Command "netsh wlan connect name='%WIFI_SSID%'"
if %errorlevel% neq 0 (
    echo [ERROR] Could not initiate WiFi connection.
    echo         Make sure "%WIFI_SSID%" is a saved network profile.
    goto :error
)

echo  Waiting for connection...
timeout /t 5 /nobreak >nul

netsh wlan show interfaces | findstr /i "SSID" | findstr /v "BSSID" | findstr /i "%WIFI_SSID%" >nul 2>&1
if %errorlevel% neq 0 (
    echo [WARN] Could not verify WiFi connection to "%WIFI_SSID%".
    echo        Attempting to continue anyway...
) else (
    echo [OK] Connected to "%WIFI_SSID%"
)
echo.

:: STEP 3: Build and Deploy to Robot
echo [3/3] Building and deploying to robot (Team %TEAM_NUMBER%)...
echo.

if not exist "%GRADLEW%" (
    echo [ERROR] gradlew.bat not found at: %GRADLEW%
    goto :error
)

call "%GRADLEW%" deploy --console=plain
if %errorlevel% neq 0 (
    echo.
    echo [ERROR] Deploy failed.
    echo         - Is the robot powered on?
    echo         - Are you connected to the robot's WiFi?
    echo         - Is the roboRIO reachable at roborio-%TEAM_NUMBER%-frc.local?
    goto :error
)

:: SUCCESS
echo.
echo SUCCESS! Code deployed to robot.
echo.
start "" "C:\Program Files (x86)\FRC Driver Station\DriverStation.exe"
goto :done

:error
echo.
echo DEPLOY FAILED. See errors above.
echo.

:done
pause
endlocal