@echo off
REM Script to set up MCP servers for Kiro
REM This script helps configure GitHub MCP server

echo ==========================================
echo MCP Server Setup for Kiro
echo ==========================================
echo.

REM Check if uv is installed
where uv >nul 2>nul
if %ERRORLEVEL% NEQ 0 (
    echo [!] UV package manager not found
    echo.
    echo Installing UV...
    powershell -Command "irm https://astral.sh/uv/install.ps1 | iex"
    
    if %ERRORLEVEL% NEQ 0 (
        echo [X] Failed to install UV
        echo Please install manually from: https://docs.astral.sh/uv/
        pause
        exit /b 1
    )
    
    echo [OK] UV installed successfully
    echo.
    echo Please close this window and run the script again in a new terminal
    pause
    exit /b 0
)

echo [OK] UV package manager found
uv --version
echo.

REM Check if GITHUB_TOKEN is set
if defined GITHUB_TOKEN (
    echo [OK] GITHUB_TOKEN environment variable is set
    echo Token: %GITHUB_TOKEN:~0,4%****%GITHUB_TOKEN:~-4%
    echo.
) else (
    echo [!] GITHUB_TOKEN environment variable not set
    echo.
    echo To set up GitHub MCP server, you need a Personal Access Token:
    echo.
    echo 1. Go to: https://github.com/settings/tokens
    echo 2. Click "Generate new token (classic)"
    echo 3. Name: "Kiro MCP Server - James Robot"
    echo 4. Select scopes: repo, workflow
    echo 5. Generate and copy the token
    echo.
    set /p TOKEN="Paste your GitHub token here (or press Enter to skip): "
    
    if not "!TOKEN!"=="" (
        echo.
        echo Setting GITHUB_TOKEN environment variable...
        setx GITHUB_TOKEN "!TOKEN!" >nul
        
        if %ERRORLEVEL% EQU 0 (
            echo [OK] GITHUB_TOKEN set successfully
            echo.
            echo IMPORTANT: Close and reopen your terminal for changes to take effect
        ) else (
            echo [X] Failed to set GITHUB_TOKEN
        )
    ) else (
        echo.
        echo Skipped token setup. You can set it later with:
        echo   setx GITHUB_TOKEN "your_token_here"
    )
    echo.
)

echo ==========================================
echo MCP Configuration Status
echo ==========================================
echo.

if exist ".kiro\settings\mcp.json" (
    echo [OK] MCP configuration file exists
    echo Location: .kiro\settings\mcp.json
) else (
    echo [!] MCP configuration file not found
    echo Expected location: .kiro\settings\mcp.json
)
echo.

echo ==========================================
echo Next Steps
echo ==========================================
echo.
echo 1. Ensure GITHUB_TOKEN is set (see above)
echo 2. Restart Kiro completely
echo 3. Check MCP Server view in Kiro
echo 4. Test by asking: "Show me the latest CI/CD logs"
echo.
echo For detailed instructions, see: docs\mcp_setup.md
echo.
pause
