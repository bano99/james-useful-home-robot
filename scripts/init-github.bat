@echo off
REM Script to initialize Git repository and push to GitHub
REM Repository: james-useful-home-robot
REM GitHub User: bano99

setlocal enabledelayedexpansion

set REPO_NAME=james-useful-home-robot
set GITHUB_USER=bano99
set REPO_URL=https://github.com/%GITHUB_USER%/%REPO_NAME%.git

echo ==========================================
echo GitHub Repository Initialization Script
echo ==========================================
echo.
echo Repository: %REPO_NAME%
echo GitHub User: %GITHUB_USER%
echo Repository URL: %REPO_URL%
echo.

REM Check if git is installed
where git >nul 2>nul
if %ERRORLEVEL% NEQ 0 (
    echo Error: git is not installed. Please install git first.
    echo Download from: https://git-scm.com/download/win
    pause
    exit /b 1
)

REM Check if we're in the right directory
if not exist "README.md" (
    echo Error: README.md not found. Please run this script from the project root.
    pause
    exit /b 1
)

REM Check if already initialized
if exist ".git" (
    echo Warning: Git repository already initialized.
    set /p REINIT="Do you want to reinitialize? This will remove existing git history. (y/N): "
    if /i "!REINIT!"=="y" (
        rmdir /s /q .git
        echo Removed existing .git directory
    ) else (
        echo Keeping existing git repository
        pause
        exit /b 0
    )
)

echo.
echo Step 1: Initializing Git repository...
git init
if %ERRORLEVEL% NEQ 0 (
    echo Error: Failed to initialize git repository
    pause
    exit /b 1
)
echo [OK] Git repository initialized

echo.
echo Step 2: Adding all files...
git add .
if %ERRORLEVEL% NEQ 0 (
    echo Error: Failed to add files
    pause
    exit /b 1
)
echo [OK] Files added to staging

echo.
echo Step 3: Creating initial commit...
git commit -m "Initial commit: Project setup and infrastructure" -m "- Add project structure (docs, platform, ros2_ws, models, scripts, tests)" -m "- Add comprehensive README with quick start guide" -m "- Add hardware setup guide with detailed installation instructions" -m "- Add CI/CD pipeline with GitHub Actions" -m "- Add contributing guidelines and code of conduct" -m "- Add issue and PR templates" -m "- Add MIT License" -m "- Configure .gitignore for ROS2, Python, and ESP32"
if %ERRORLEVEL% NEQ 0 (
    echo Error: Failed to create commit
    pause
    exit /b 1
)
echo [OK] Initial commit created

echo.
echo Step 4: Renaming branch to main...
git branch -M main
if %ERRORLEVEL% NEQ 0 (
    echo Error: Failed to rename branch
    pause
    exit /b 1
)
echo [OK] Branch renamed to main

echo.
echo ==========================================
echo IMPORTANT: Create GitHub Repository
echo ==========================================
echo.
echo Before proceeding, you need to create the repository on GitHub:
echo.
echo 1. Go to: https://github.com/new
echo 2. Repository name: %REPO_NAME%
echo 3. Description: Autonomous home cleanup robot with ROS2, Jetson Nano, and AR4-MK3 arm
echo 4. Choose: Public or Private
echo 5. DO NOT initialize with README, .gitignore, or license (we already have these)
echo 6. Click 'Create repository'
echo.
pause

echo.
echo Step 5: Adding remote origin...
git remote add origin "%REPO_URL%"
if %ERRORLEVEL% NEQ 0 (
    echo Error: Failed to add remote origin
    pause
    exit /b 1
)
echo [OK] Remote origin added

echo.
echo Step 6: Pushing to GitHub...
echo You may be prompted for your GitHub credentials.
echo.

REM Try to push
git push -u origin main
if %ERRORLEVEL% NEQ 0 (
    echo.
    echo ==========================================
    echo Push Failed - Authentication Required
    echo ==========================================
    echo.
    echo If you're using HTTPS and don't have credentials configured:
    echo.
    echo Option 1: Use Personal Access Token (Recommended)
    echo   1. Go to: https://github.com/settings/tokens
    echo   2. Click 'Generate new token (classic)'
    echo   3. Select scopes: 'repo' (full control of private repositories)
    echo   4. Generate and copy the token
    echo   5. Use the token as your password when prompted
    echo.
    echo Option 2: Use GitHub CLI
    echo   1. Install: https://cli.github.com/
    echo   2. Authenticate: gh auth login
    echo   3. Push again: git push -u origin main
    echo.
    echo Option 3: Use Git Credential Manager
    echo   1. Should be included with Git for Windows
    echo   2. Will prompt for credentials on first push
    echo   3. Credentials will be saved for future use
    echo.
    pause
    exit /b 1
)

echo.
echo ==========================================
echo Success! Repository Published
echo ==========================================
echo.
echo Your repository is now available at:
echo https://github.com/%GITHUB_USER%/%REPO_NAME%
echo.
echo Next steps:
echo 1. Visit your repository on GitHub
echo 2. Add a repository description and topics
echo 3. Enable GitHub Pages (optional) for documentation
echo 4. Configure branch protection rules (optional)
echo 5. Set up GitHub Actions secrets if needed
echo.
echo To clone on another machine:
echo git clone %REPO_URL%
echo.
echo Happy coding! 🤖
echo.
pause
