@echo off
title RoboRIO Log Cleaner - Team 7476

echo RoboRIO Log Cleaner - FRC Team 7476
echo.
echo Make sure you are connected to the robot before continuing!
echo.
pause

echo.
echo Connecting to roboRIO and deleting logs...
echo.

plink -ssh -pw "" -batch admin@roboRIO-7476-frc.local "cd /home/lvuser/logs && echo Files to delete: && ls && rm -rf * && echo Logs deleted successfully."

echo.
if %ERRORLEVEL% EQU 0 (
    echo Done! All logs have been deleted.
) else (
    echo Something went wrong. Make sure you are connected to the robot and try again.
)
echo.
pause