# MCP Quick Start - GitHub CI/CD Monitoring

This guide will get you up and running with GitHub MCP server in 5 minutes so Kiro can read your CI/CD logs.

## Quick Setup (3 Steps)

### Step 1: Get GitHub Token

1. Go to: https://github.com/settings/tokens
2. Click **"Generate new token (classic)"**
3. Name: `Kiro MCP Server`
4. Select scopes: ✅ `repo` and ✅ `workflow`
5. Click **"Generate token"**
6. **Copy the token** (you won't see it again!)

### Step 2: Set Environment Variable

**Windows (PowerShell):**
```powershell
setx GITHUB_TOKEN "paste_your_token_here"
```

**Linux/Mac:**
```bash
echo 'export GITHUB_TOKEN="paste_your_token_here"' >> ~/.bashrc
source ~/.bashrc
```

### Step 3: Install UV and Restart Kiro

**Windows:**
```powershell
powershell -c "irm https://astral.sh/uv/install.ps1 | iex"
```

**Linux/Mac:**
```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

Then **completely close and restart Kiro**.

## Verify It Works

After restarting Kiro, ask:

> "Show me the logs from the latest CI/CD workflow run"

or

> "What's the status of the GitHub Actions workflows?"

If Kiro can access the information, you're all set! 🎉

## Automated Setup

Alternatively, run the setup script:

**Windows:**
```cmd
scripts\setup-mcp.bat
```

**Linux/Mac:**
```bash
chmod +x scripts/setup-mcp.sh
./scripts/setup-mcp.sh
```

## Troubleshooting

**MCP server not connecting?**
1. Verify token is set: `echo $env:GITHUB_TOKEN` (PowerShell) or `echo $GITHUB_TOKEN` (bash)
2. Check UV is installed: `uv --version`
3. Restart Kiro completely (not just reload)
4. Check MCP Server view in Kiro for errors

**Unicode/Encoding errors on Windows?**
The configuration has been updated to fix this. If you still see errors:
```powershell
[System.Environment]::SetEnvironmentVariable('PYTHONIOENCODING', 'utf-8', 'User')
[System.Environment]::SetEnvironmentVariable('PYTHONUTF8', '1', 'User')
```
Then restart Kiro.

**Still not working?**
See the detailed guide: [docs/mcp_setup.md](docs/mcp_setup.md)

## What You Can Do Now

With GitHub MCP server configured, you can ask Kiro to:

- ✅ Read CI/CD workflow logs
- ✅ Check workflow run status
- ✅ List repository issues
- ✅ View pull requests
- ✅ Read file contents from GitHub
- ✅ Search repositories
- ✅ Create issues and PRs

## Example Questions

```
"Show me the error logs from the failed CI/CD run"
"What workflows are configured in this repository?"
"List all open issues"
"What's in the latest commit?"
"Show me the contents of the CI workflow file"
```

---

**Ready to fix those CI/CD failures?** Ask Kiro to check the logs! 🚀
