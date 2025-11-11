@echo off
REM Diagnostic script for MCP server issues

echo ==========================================
echo MCP Server Diagnostics
echo ==========================================
echo.

echo [1/5] Checking UV installation...
where uv >nul 2>nul
if %ERRORLEVEL% EQU 0 (
    echo [OK] UV is installed
    uv --version
) else (
    echo [X] UV is NOT installed
    echo Install with: powershell -c "irm https://astral.sh/uv/install.ps1 | iex"
)
echo.

echo [2/5] Checking GITHUB_TOKEN...
if defined GITHUB_TOKEN (
    echo [OK] GITHUB_TOKEN is set
    echo Length: %GITHUB_TOKEN:~0,0%
    set TOKEN_LEN=0
    for /f %%a in ('echo %GITHUB_TOKEN%^| find /c /v ""') do set TOKEN_LEN=%%a
    if %TOKEN_LEN% GTR 20 (
        echo [OK] Token length looks valid
    ) else (
        echo [!] Token seems too short
    )
) else (
    echo [X] GITHUB_TOKEN is NOT set
    echo.
    echo To set it:
    echo   1. Get token from: https://github.com/settings/tokens
    echo   2. Run: setx GITHUB_TOKEN "your_token_here"
    echo   3. Restart this terminal
)
echo.

echo [3/5] Checking Python UTF-8 settings...
if defined PYTHONIOENCODING (
    echo [OK] PYTHONIOENCODING = %PYTHONIOENCODING%
) else (
    echo [!] PYTHONIOENCODING not set
)

if defined PYTHONUTF8 (
    echo [OK] PYTHONUTF8 = %PYTHONUTF8%
) else (
    echo [!] PYTHONUTF8 not set
)
echo.

echo [4/5] Checking MCP configuration...
if exist ".kiro\settings\mcp.json" (
    echo [OK] MCP config exists: .kiro\settings\mcp.json
) else (
    echo [X] MCP config NOT found: .kiro\settings\mcp.json
)
echo.

echo [5/5] Testing GitHub MCP server manually...
if defined GITHUB_TOKEN (
    echo Running: uvx mcp-server-github
    echo This will test if the server can start...
    echo Press Ctrl+C to stop after a few seconds
    echo.
    set GITHUB_PERSONAL_ACCESS_TOKEN=%GITHUB_TOKEN%
    set PYTHONIOENCODING=utf-8
    set PYTHONUTF8=1
    timeout /t 2 /nobreak >nul
    uvx mcp-server-github
) else (
    echo [!] Skipping test - GITHUB_TOKEN not set
)

echo.
echo ==========================================
echo Diagnosis Complete
echo ==========================================
echo.
echo Common fixes:
echo 1. Set GITHUB_TOKEN: setx GITHUB_TOKEN "your_token"
echo 2. Restart terminal and Kiro
echo 3. Check token is valid at: https://github.com/settings/tokens
echo 4. Ensure token has 'repo' and 'workflow' scopes
echo.
pause
