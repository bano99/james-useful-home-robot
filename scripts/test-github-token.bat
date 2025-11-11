@echo off
REM Script to test GitHub token and API access

echo ==========================================
echo GitHub Token Test
echo ==========================================
echo.

REM Check if GITHUB_TOKEN is set
if defined GITHUB_TOKEN (
    echo [OK] GITHUB_TOKEN environment variable is set
    echo Token preview: %GITHUB_TOKEN:~0,4%****%GITHUB_TOKEN:~-4%
    echo.
) else (
    echo [X] GITHUB_TOKEN environment variable is NOT set
    echo.
    echo Please set it with:
    echo   setx GITHUB_TOKEN "your_token_here"
    echo.
    pause
    exit /b 1
)

echo Testing GitHub API access...
echo.

REM Test GitHub API with curl
curl -s -H "Authorization: token %GITHUB_TOKEN%" https://api.github.com/user > nul 2>&1

if %ERRORLEVEL% EQU 0 (
    echo [OK] GitHub API is accessible
    echo.
    
    REM Get user info
    echo Your GitHub user info:
    curl -s -H "Authorization: token %GITHUB_TOKEN%" https://api.github.com/user
    echo.
    echo.
    
    REM Test repository access
    echo Testing repository access...
    curl -s -H "Authorization: token %GITHUB_TOKEN%" https://api.github.com/repos/bano99/james-useful-home-robot > nul 2>&1
    
    if %ERRORLEVEL% EQU 0 (
        echo [OK] Can access repository: bano99/james-useful-home-robot
    ) else (
        echo [!] Cannot access repository (might be private or token lacks permissions)
    )
) else (
    echo [X] Cannot access GitHub API
    echo.
    echo Possible issues:
    echo 1. Token is invalid or expired
    echo 2. No internet connection
    echo 3. GitHub API is down
    echo.
    echo Please verify your token at: https://github.com/settings/tokens
)

echo.
echo ==========================================
echo Token Scopes Check
echo ==========================================
echo.
echo Your token should have these scopes:
echo   - repo (Full control of private repositories)
echo   - workflow (Update GitHub Action workflows)
echo.
echo To check your token scopes, visit:
echo https://github.com/settings/tokens
echo.
pause
