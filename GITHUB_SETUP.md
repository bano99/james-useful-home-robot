# GitHub Setup Guide

This guide will help you publish the James project to GitHub under your account.

## Quick Start

### Option 1: Using the Automated Script (Recommended)

**On Windows:**
```cmd
cd path\to\james-useful-home-robot
scripts\init-github.bat
```

**On Linux/Mac:**
```bash
cd /path/to/james-useful-home-robot
chmod +x scripts/init-github.sh
./scripts/init-github.sh
```

The script will guide you through the entire process.

### Option 2: Manual Setup

If you prefer to do it manually, follow these steps:

#### 1. Create Repository on GitHub

1. Go to https://github.com/new
2. Fill in the details:
   - **Repository name**: `james-useful-home-robot`
   - **Description**: `Autonomous home cleanup robot with ROS2, Jetson Nano, and AR4-MK3 arm`
   - **Visibility**: Choose Public or Private
   - **DO NOT** check "Initialize this repository with a README"
   - **DO NOT** add .gitignore or license (we already have these)
3. Click "Create repository"

#### 2. Initialize Local Repository

```bash
# Navigate to project directory
cd /path/to/james-useful-home-robot

# Initialize git repository
git init

# Add all files
git add .

# Create initial commit
git commit -m "Initial commit: Project setup and infrastructure"

# Rename branch to main
git branch -M main
```

#### 3. Connect to GitHub

```bash
# Add remote repository
git remote add origin https://github.com/bano99/james-useful-home-robot.git

# Push to GitHub
git push -u origin main
```

## Authentication Options

### Option 1: Personal Access Token (Recommended)

1. Go to https://github.com/settings/tokens
2. Click "Generate new token (classic)"
3. Give it a descriptive name (e.g., "James Robot Project")
4. Select scopes:
   - ✅ `repo` (Full control of private repositories)
5. Click "Generate token"
6. **Copy the token immediately** (you won't see it again!)
7. When prompted for password during `git push`, use the token instead

**Save credentials (optional):**
```bash
# Cache credentials for 1 hour
git config --global credential.helper cache

# Or store permanently (less secure)
git config --global credential.helper store
```

### Option 2: SSH Key

1. Generate SSH key:
```bash
ssh-keygen -t ed25519 -C "your_email@example.com"
```

2. Add to SSH agent:
```bash
eval "$(ssh-agent -s)"
ssh-add ~/.ssh/id_ed25519
```

3. Copy public key:
```bash
cat ~/.ssh/id_ed25519.pub
```

4. Add to GitHub:
   - Go to https://github.com/settings/keys
   - Click "New SSH key"
   - Paste your public key
   - Click "Add SSH key"

5. Change remote URL to SSH:
```bash
git remote set-url origin git@github.com:bano99/james-useful-home-robot.git
```

6. Push:
```bash
git push -u origin main
```

### Option 3: GitHub CLI

1. Install GitHub CLI:
   - **Windows**: Download from https://cli.github.com/
   - **Linux**: `sudo apt install gh`
   - **Mac**: `brew install gh`

2. Authenticate:
```bash
gh auth login
```

3. Follow the prompts to authenticate

4. Push:
```bash
git push -u origin main
```

## Post-Setup Configuration

### 1. Add Repository Description and Topics

1. Go to your repository: https://github.com/bano99/james-useful-home-robot
2. Click the gear icon ⚙️ next to "About"
3. Add description: `Autonomous home cleanup robot with ROS2, Jetson Nano, and AR4-MK3 arm`
4. Add topics (tags):
   - `robotics`
   - `ros2`
   - `jetson-nano`
   - `autonomous-robot`
   - `home-automation`
   - `computer-vision`
   - `slam`
   - `manipulation`
   - `mecanum-wheels`
   - `realsense`

### 2. Configure Branch Protection (Optional)

1. Go to Settings → Branches
2. Add rule for `main` branch:
   - ✅ Require pull request reviews before merging
   - ✅ Require status checks to pass before merging
   - ✅ Require branches to be up to date before merging

### 3. Enable GitHub Actions

GitHub Actions should be enabled by default. Verify:
1. Go to Actions tab
2. If prompted, click "I understand my workflows, go ahead and enable them"

### 4. Set Up GitHub Pages (Optional)

To host documentation:
1. Go to Settings → Pages
2. Source: Deploy from a branch
3. Branch: `main` / `docs` folder
4. Click Save

### 5. Add Collaborators (Optional)

1. Go to Settings → Collaborators
2. Click "Add people"
3. Enter GitHub username or email

## Common Issues and Solutions

### Issue: "Permission denied (publickey)"

**Solution**: You need to set up SSH keys or use HTTPS with a token.

### Issue: "Authentication failed"

**Solution**: 
- If using HTTPS: Use a Personal Access Token instead of your password
- If using SSH: Ensure your SSH key is added to GitHub

### Issue: "Repository not found"

**Solution**: 
- Verify the repository exists on GitHub
- Check the remote URL: `git remote -v`
- Ensure you have access to the repository

### Issue: "Failed to push some refs"

**Solution**:
```bash
# Pull first if repository has changes
git pull origin main --rebase

# Then push
git push -u origin main
```

## Useful Git Commands

```bash
# Check repository status
git status

# View commit history
git log --oneline

# View remote repositories
git remote -v

# Create a new branch
git checkout -b feature/new-feature

# Switch branches
git checkout main

# Pull latest changes
git pull origin main

# Push changes
git push origin main

# View differences
git diff

# Undo last commit (keep changes)
git reset --soft HEAD~1

# Undo last commit (discard changes)
git reset --hard HEAD~1
```

## Next Steps

After publishing to GitHub:

1. ✅ Verify repository is accessible: https://github.com/bano99/james-useful-home-robot
2. ✅ Check that all files are present
3. ✅ Verify GitHub Actions CI/CD pipeline runs successfully
4. ✅ Add repository description and topics
5. ✅ Share with collaborators or community
6. ✅ Start working on the next tasks from `tasks.md`

## Getting Help

- **GitHub Docs**: https://docs.github.com/
- **Git Documentation**: https://git-scm.com/doc
- **GitHub Community**: https://github.community/

---

**Repository URL**: https://github.com/bano99/james-useful-home-robot

**Clone Command**: 
```bash
git clone https://github.com/bano99/james-useful-home-robot.git
```
