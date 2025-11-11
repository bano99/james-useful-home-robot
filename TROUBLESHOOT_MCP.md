# MCP Troubleshooting Guide

## Error: "Request timed out" for GitHub MCP Server

This error means the GitHub MCP server is taking too long to connect or initialize.

### Quick Diagnosis

Run the diagnostic script:
```cmd
scripts\diagnose-mcp.bat
```

This will check:
- UV installation
- GITHUB_TOKEN environment variable
- Python UTF-8 settings
- MCP configuration file
- GitHub MCP server startup

### Common Causes and Solutions

#### 1. GITHUB_TOKEN Not Set or Invalid

**Check if token is set:**
```cmd
echo %GITHUB_TOKEN%
```

If it shows `%GITHUB_TOKEN%` (literally), the variable is not set.

**Solution:**
```cmd
REM Get a new token from https://github.com/settings/tokens
setx GITHUB_TOKEN "ghp_your_token_here"

REM Close this terminal and open a new one
REM Then restart Kiro
```

**Verify token works:**
```cmd
scripts\test-github-token.bat
```

#### 2. Token Lacks Required Scopes

Your token needs these scopes:
- ✅ `repo` - Full control of private repositories
- ✅ `workflow` - Update GitHub Action workflows

**Solution:**
1. Go to https://github.com/settings/tokens
2. Click on your token
3. Verify scopes are selected
4. If not, regenerate the token with correct scopes
5. Update the environment variable

#### 3. Network/Firewall Issues

The MCP server needs to connect to GitHub API.

**Test GitHub API access:**
```cmd
curl -H "Authorization: token %GITHUB_TOKEN%" https://api.github.com/user
```

If this fails:
- Check your internet connection
- Check if firewall is blocking GitHub API
- Try from a different network
- Check if corporate proxy is interfering

#### 4. UV Package Manager Issues

**Verify UV is installed:**
```cmd
uv --version
```

If not found:
```cmd
powershell -c "irm https://astral.sh/uv/install.ps1 | iex"
```

**Test MCP server manually:**
```cmd
set GITHUB_PERSONAL_ACCESS_TOKEN=%GITHUB_TOKEN%
uvx mcp-server-github
```

Press Ctrl+C after a few seconds. If you see errors, that's the issue.

#### 5. Environment Variables Not Loaded

Kiro needs to be restarted AFTER setting environment variables.

**Solution:**
1. Set all environment variables:
```cmd
setx GITHUB_TOKEN "your_token"
setx PYTHONIOENCODING "utf-8"
setx PYTHONUTF8 "1"
```

2. Close ALL terminal windows
3. Close Kiro COMPLETELY (not just the window)
4. Open a new terminal and verify:
```cmd
echo %GITHUB_TOKEN%
echo %PYTHONIOENCODING%
echo %PYTHONUTF8%
```

5. Start Kiro

#### 6. MCP Server Taking Too Long to Start

The GitHub MCP server might be slow on first run (downloading dependencies).

**Solution:**
Pre-install the MCP server:
```cmd
uvx mcp-server-github --help
```

This downloads and caches the server. Then restart Kiro.

### Alternative: Disable GitHub MCP Server Temporarily

If you need to work without GitHub MCP server, you can disable it:

Edit `.kiro/settings/mcp.json`:
```json
{
  "mcpServers": {
    "github": {
      "disabled": true,
      ...
    }
  }
}
```

Then restart Kiro. The filesystem MCP server should still work.

### Alternative: Use GitHub CLI Instead

If MCP server continues to fail, you can use GitHub CLI for CI/CD logs:

**Install GitHub CLI:**
```cmd
winget install --id GitHub.cli
```

**Authenticate:**
```cmd
gh auth login
```

**View workflow runs:**
```cmd
gh run list --repo bano99/james-useful-home-robot
```

**View logs:**
```cmd
gh run view <run-id> --log --repo bano99/james-useful-home-robot
```

### Step-by-Step Reset

If nothing works, try a complete reset:

1. **Remove MCP configuration:**
```cmd
del .kiro\settings\mcp.json
```

2. **Clear UV cache:**
```cmd
uv cache clean
```

3. **Unset environment variables:**
```cmd
reg delete "HKCU\Environment" /v GITHUB_TOKEN /f
reg delete "HKCU\Environment" /v PYTHONIOENCODING /f
reg delete "HKCU\Environment" /v PYTHONUTF8 /f
```

4. **Restart computer**

5. **Start fresh:**
```cmd
REM Get new token from https://github.com/settings/tokens
setx GITHUB_TOKEN "new_token_here"
setx PYTHONIOENCODING "utf-8"
setx PYTHONUTF8 "1"

REM Reinstall UV
powershell -c "irm https://astral.sh/uv/install.ps1 | iex"

REM Restore MCP config
copy .kiro\settings\mcp.json.backup .kiro\settings\mcp.json

REM Restart Kiro
```

### Getting More Debug Info

To see detailed MCP server logs:

1. Open Kiro
2. Go to View → Output
3. Select "Kiro - MCP Logs" from dropdown
4. Look for specific error messages

Common error patterns:
- `Authentication failed` → Token is invalid
- `Connection refused` → Network/firewall issue
- `Module not found` → UV/Python installation issue
- `Timeout` → Server taking too long (network or first-time download)

### Still Not Working?

If you've tried everything:

1. **Check Kiro version** - Ensure you're on the latest version
2. **Check system requirements** - Windows 10/11, Python 3.10+
3. **Try on different machine** - Rule out system-specific issues
4. **Report issue** - File a bug report with:
   - Kiro version
   - Windows version
   - UV version (`uv --version`)
   - MCP logs from Kiro
   - Output from `scripts\diagnose-mcp.bat`

### Workaround: Manual CI/CD Log Access

While troubleshooting MCP, you can manually check CI/CD logs:

1. Go to: https://github.com/bano99/james-useful-home-robot/actions
2. Click on the failed workflow run
3. Click on the failed job
4. View the logs directly in browser

You can copy/paste relevant error messages to me, and I can help fix them without needing MCP access.

---

**Need immediate help?** Run `scripts\diagnose-mcp.bat` and share the output!
