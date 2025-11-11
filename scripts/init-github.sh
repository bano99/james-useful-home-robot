#!/bin/bash

# Script to initialize Git repository and push to GitHub
# Repository: james-useful-home-robot
# GitHub User: bano99

set -e  # Exit on error

REPO_NAME="james-useful-home-robot"
GITHUB_USER="bano99"
REPO_URL="https://github.com/${GITHUB_USER}/${REPO_NAME}.git"

echo "=========================================="
echo "GitHub Repository Initialization Script"
echo "=========================================="
echo ""
echo "Repository: ${REPO_NAME}"
echo "GitHub User: ${GITHUB_USER}"
echo "Repository URL: ${REPO_URL}"
echo ""

# Check if git is installed
if ! command -v git &> /dev/null; then
    echo "Error: git is not installed. Please install git first."
    exit 1
fi

# Check if we're in the right directory
if [ ! -f "README.md" ]; then
    echo "Error: README.md not found. Please run this script from the project root."
    exit 1
fi

# Check if already initialized
if [ -d ".git" ]; then
    echo "Warning: Git repository already initialized."
    read -p "Do you want to reinitialize? This will remove existing git history. (y/N): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        rm -rf .git
        echo "Removed existing .git directory"
    else
        echo "Keeping existing git repository"
        exit 0
    fi
fi

echo ""
echo "Step 1: Initializing Git repository..."
git init
echo "✓ Git repository initialized"

echo ""
echo "Step 2: Adding all files..."
git add .
echo "✓ Files added to staging"

echo ""
echo "Step 3: Creating initial commit..."
git commit -m "Initial commit: Project setup and infrastructure

- Add project structure (docs, platform, ros2_ws, models, scripts, tests)
- Add comprehensive README with quick start guide
- Add hardware setup guide with detailed installation instructions
- Add CI/CD pipeline with GitHub Actions
- Add contributing guidelines and code of conduct
- Add issue and PR templates
- Add MIT License
- Configure .gitignore for ROS2, Python, and ESP32"
echo "✓ Initial commit created"

echo ""
echo "Step 4: Renaming branch to main..."
git branch -M main
echo "✓ Branch renamed to main"

echo ""
echo "=========================================="
echo "IMPORTANT: Create GitHub Repository"
echo "=========================================="
echo ""
echo "Before proceeding, you need to create the repository on GitHub:"
echo ""
echo "1. Go to: https://github.com/new"
echo "2. Repository name: ${REPO_NAME}"
echo "3. Description: Autonomous home cleanup robot with ROS2, Jetson Nano, and AR4-MK3 arm"
echo "4. Choose: Public or Private"
echo "5. DO NOT initialize with README, .gitignore, or license (we already have these)"
echo "6. Click 'Create repository'"
echo ""
read -p "Press Enter once you've created the repository on GitHub..."

echo ""
echo "Step 5: Adding remote origin..."
git remote add origin "${REPO_URL}"
echo "✓ Remote origin added"

echo ""
echo "Step 6: Pushing to GitHub..."
echo "You may be prompted for your GitHub credentials."
echo ""

# Try to push
if git push -u origin main; then
    echo "✓ Successfully pushed to GitHub!"
else
    echo ""
    echo "=========================================="
    echo "Push Failed - Authentication Required"
    echo "=========================================="
    echo ""
    echo "If you're using HTTPS and don't have credentials configured:"
    echo ""
    echo "Option 1: Use Personal Access Token (Recommended)"
    echo "  1. Go to: https://github.com/settings/tokens"
    echo "  2. Click 'Generate new token (classic)'"
    echo "  3. Select scopes: 'repo' (full control of private repositories)"
    echo "  4. Generate and copy the token"
    echo "  5. Use the token as your password when prompted"
    echo ""
    echo "Option 2: Use SSH instead of HTTPS"
    echo "  1. Generate SSH key: ssh-keygen -t ed25519 -C 'your_email@example.com'"
    echo "  2. Add to GitHub: https://github.com/settings/keys"
    echo "  3. Change remote URL:"
    echo "     git remote set-url origin git@github.com:${GITHUB_USER}/${REPO_NAME}.git"
    echo "  4. Push again: git push -u origin main"
    echo ""
    echo "Option 3: Use GitHub CLI"
    echo "  1. Install: https://cli.github.com/"
    echo "  2. Authenticate: gh auth login"
    echo "  3. Push again: git push -u origin main"
    echo ""
    exit 1
fi

echo ""
echo "=========================================="
echo "Success! Repository Published"
echo "=========================================="
echo ""
echo "Your repository is now available at:"
echo "https://github.com/${GITHUB_USER}/${REPO_NAME}"
echo ""
echo "Next steps:"
echo "1. Visit your repository on GitHub"
echo "2. Add a repository description and topics"
echo "3. Enable GitHub Pages (optional) for documentation"
echo "4. Configure branch protection rules (optional)"
echo "5. Set up GitHub Actions secrets if needed"
echo ""
echo "To clone on another machine:"
echo "git clone ${REPO_URL}"
echo ""
echo "Happy coding! 🤖"
