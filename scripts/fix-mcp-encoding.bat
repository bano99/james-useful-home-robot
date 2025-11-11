@echo off
REM Script to fix Unicode encoding issues with MCP servers on Windows

echo ==========================================
echo MCP Encoding Fix for Windows
echo ==========================================
echo.
echo This script fixes Unicode encoding errors like:
echo "UnicodeEncodeError: 'charmap' codec can't encode character"
echo.

echo Setting Python UTF-8 environment variables...
echo.

REM Set environment variables for current user
setx PYTHONIOENCODING "utf-8" >nul
if %ERRORLEVEL% EQU 0 (
    echo [OK] PYTHONIOENCODING set to utf-8
) else (
    echo [X] Failed to set PYTHONIOENCODING
)

setx PYTHONUTF8 "1" >nul
if %ERRORLEVEL% EQU 0 (
    echo [OK] PYTHONUTF8 set to 1
) else (
    echo [X] Failed to set PYTHONUTF8
)

echo.
echo ==========================================
echo Configuration Updated
echo ==========================================
echo.
echo The following environment variables have been set:
echo   PYTHONIOENCODING = utf-8
echo   PYTHONUTF8 = 1
echo.
echo These are also configured in .kiro/settings/mcp.json
echo.
echo IMPORTANT: 
echo 1. Close this window
echo 2. Completely close Kiro (not just the window)
echo 3. Restart Kiro
echo.
echo The MCP servers should now work without encoding errors.
echo.

REM Check if MCP config exists
if exist ".kiro\settings\mcp.json" (
    echo [OK] MCP configuration file found
    echo.
    echo The configuration already includes UTF-8 encoding settings.
) else (
    echo [!] MCP configuration file not found
    echo Expected: .kiro\settings\mcp.json
    echo.
)

echo ==========================================
echo Additional Fix (Optional)
echo ==========================================
echo.
echo If you still experience issues, enable system-wide UTF-8:
echo.
echo 1. Open Settings
echo 2. Go to Time ^& Language ^> Language
echo 3. Click "Administrative language settings"
echo 4. Click "Change system locale"
echo 5. Check "Beta: Use Unicode UTF-8 for worldwide language support"
echo 6. Restart your computer
echo.
pause
