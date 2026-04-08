@echo off
setlocal

echo ============================================================
echo  RoboRIO Log Cleaner - Team 7476
echo ============================================================
echo.

:: Check that plink is available
where plink >nul 2>&1
if %ERRORLEVEL% NEQ 0 (
  echo [ERROR] plink.exe not found. Please install PuTTY and make sure
  echo  plink.exe is on your PATH, then re-run this script.
  echo  Download: https://www.putty.org/
  pause
  exit /b 1
)

set TEAM_NUM=7476
set HOST=roboRIO-%TEAM_NUM%-frc.local
set USER=admin
set LOG_DIR=/home/lvuser/logs

echo Connecting to: %HOST%
echo.

:: Ping check
echo [1/4] Checking connection to %HOST% ...
ping -n 1 -w 2000 %HOST% >nul 2>&1
if %ERRORLEVEL% NEQ 0 (
  echo.
  echo [ERROR] Cannot reach %HOST%.
  echo  Make sure you are connected to the robot network
  echo  ^(USB, Ethernet, or robot WiFi^) and try again.
  pause
  exit /b 1
)
echo  Connected! Robot is reachable.
echo.

:: Auto-fetch and cache the host key (one-time trust)
echo [2/4] Fetching and caching host key from roboRIO ...
echo.
echo  If PuTTY asks you to trust the host key, type "y" and press Enter.
echo.
echo exit | plink -ssh %USER%@%HOST% -pw "" exit >nul 2>&1

:: List logs before deleting
echo [3/4] Current log files on the roboRIO:
echo ----------------------------------------
plink -batch -ssh %USER%@%HOST% -pw "" "ls %LOG_DIR% 2>/dev/null || echo 'directory is empty or does not exist'"
echo ----------------------------------------
echo.

:: Confirm before deleting
set /p CONFIRM=Are you sure you want to delete ALL logs? (Y/N):
if /i not "%CONFIRM%"=="Y" (
  echo Cancelled. No files were deleted.
  pause
  exit /b 0
)
echo.

:: Delete all logs
echo [4/4] Deleting logs in %LOG_DIR% ...
plink -batch -ssh %USER%@%HOST% -pw "" "cd %LOG_DIR% && rm -rf * && echo Done."
if %ERRORLEVEL% NEQ 0 (
  echo.
  echo [ERROR] Something went wrong during deletion.
  echo  Check that the roboRIO is still connected and try again.
  pause
  exit /b 1
)

echo.
echo [SUCCESS] All logs deleted from %LOG_DIR% on %HOST%.
echo.
pause
exit /b 0