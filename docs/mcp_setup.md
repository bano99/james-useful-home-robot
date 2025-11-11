# MCP Server Setup Guide

This guide explains how to set up Model Context Protocol (MCP) servers for the James project, including the GitHub MCP server for accessing CI/CD logs and repository data.

## What is MCP?

Model Context Protocol (MCP) allows AI assistants like Kiro to interact with external services and tools. For this project, we use MCP servers to:
- Access GitHub repository data (issues, PRs, CI/CD logs)
- Read local filesystem
- Interact with Git repositories

## GitHub MCP Server Setup

The GitHub MCP server allows Kiro to:
- Read CI/CD workflow logs
- Check workflow run status
- Access repository issues and pull requests
- Read file contents from GitHub
- Search repositories

### Step 1: Create GitHub Personal Access Token

1. Go to https://github.com/settings/tokens
2. Click **"Generate new token"** → **"Generate new token (classic)"**
3. Give it a descriptive name: `Kiro MCP Server - James Robot`
4. Set expiration: Choose based on your preference (90 days recommended)
5. Select the following scopes:
   - ✅ `repo` - Full control of private repositories
     - This includes: `repo:status`, `repo_deployment`, `public_repo`, `repo:invite`
   - ✅ `workflow` - Update GitHub Action workflows
   - ✅ `read:org` - Read org and team membership (if using organizations)
6. Click **"Generate token"**
7. **IMPORTANT**: Copy the token immediately (you won't see it again!)

### Step 2: Set Environment Variable

The MCP server needs your GitHub token as an environment variable.

**On Windows (PowerShell):**

```powershell
# Set for current session
$env:GITHUB_TOKEN = "your_github_token_here"

# Set permanently (User level)
[System.Environment]::SetEnvironmentVariable('GITHUB_TOKEN', 'your_github_token_here', 'User')

# Verify
echo $env:GITHUB_TOKEN
```

**On Windows (Command Prompt):**

```cmd
# Set for current session
set GITHUB_TOKEN=your_github_token_here

# Set permanently
setx GITHUB_TOKEN "your_github_token_here"

# Verify (in new command prompt)
echo %GITHUB_TOKEN%
```

**On Linux/Mac:**

```bash
# Add to ~/.bashrc or ~/.zshrc
echo 'export GITHUB_TOKEN="your_github_token_here"' >> ~/.bashrc
source ~/.bashrc

# Or for current session only
export GITHUB_TOKEN="your_github_token_here"

# Verify
echo $GITHUB_TOKEN
```

### Step 3: Install UV Package Manager

The MCP servers use `uvx` (part of the `uv` package manager) to run.

**On Windows (PowerShell):**

```powershell
# Install uv
powershell -c "irm https://astral.sh/uv/install.ps1 | iex"

# Verify installation
uv --version
```

**On Linux/Mac:**

```bash
# Install uv
curl -LsSf https://astral.sh/uv/install.sh | sh

# Add to PATH (if not already)
echo 'export PATH="$HOME/.cargo/bin:$PATH"' >> ~/.bashrc
source ~/.bashrc

# Verify installation
uv --version
```

### Step 4: Restart Kiro

After setting the environment variable and installing UV:

1. **Completely close Kiro** (not just the window, but exit the application)
2. **Restart Kiro**
3. The MCP servers will automatically connect on startup

### Step 5: Verify MCP Server Connection

In Kiro, you should see the MCP servers connected in the MCP Server view (in the Kiro feature panel).

You can also verify by asking Kiro to:
- "List the recent workflow runs for my repository"
- "Show me the logs from the latest CI/CD run"
- "What issues are open in my repository?"

## MCP Configuration File

The MCP configuration is stored in `.kiro/settings/mcp.json`:

```json
{
  "mcpServers": {
    "github": {
      "command": "uvx",
      "args": ["mcp-server-github"],
      "env": {
        "GITHUB_PERSONAL_ACCESS_TOKEN": "${GITHUB_TOKEN}"
      },
      "disabled": false,
      "autoApprove": [
        "list_commits",
        "get_file_contents",
        "search_repositories",
        "get_issue",
        "list_issues"
      ]
    },
    "filesystem": {
      "command": "uvx",
      "args": ["mcp-server-filesystem", "C:\\Users\\denis\\Documents\\James_Useful_Home_Robot"],
      "env": {
        "FASTMCP_LOG_LEVEL": "ERROR"
      },
      "disabled": false,
      "autoApprove": [
        "read_file",
        "list_directory",
        "search_files"
      ]
    }
  }
}
```

## Available MCP Tools

Once configured, Kiro can use these GitHub MCP tools:

### Repository Operations
- `get_file_contents` - Read file contents from GitHub
- `search_repositories` - Search for repositories
- `create_repository` - Create new repositories
- `fork_repository` - Fork repositories

### Issues and Pull Requests
- `create_issue` - Create new issues
- `get_issue` - Get issue details
- `list_issues` - List repository issues
- `update_issue` - Update existing issues
- `create_pull_request` - Create pull requests
- `get_pull_request` - Get PR details
- `list_pull_requests` - List repository PRs

### Commits and Branches
- `list_commits` - List repository commits
- `create_branch` - Create new branches
- `list_branches` - List repository branches

### Workflows and Actions
- `list_workflows` - List GitHub Actions workflows
- `get_workflow_run` - Get workflow run details
- `list_workflow_runs` - List workflow runs
- `get_workflow_run_logs` - **Get CI/CD logs** ⭐

## Troubleshooting

### Issue: "MCP server not connecting"

**Solution:**
1. Verify `uv` is installed: `uv --version`
2. Check environment variable is set: `echo $env:GITHUB_TOKEN` (PowerShell) or `echo %GITHUB_TOKEN%` (CMD)
3. Restart Kiro completely
4. Check MCP Server view in Kiro for error messages

### Issue: "UnicodeEncodeError: 'charmap' codec can't encode character"

This is a Windows-specific encoding issue when MCP servers try to output Unicode characters.

**Solution:**
The MCP configuration has been updated to force UTF-8 encoding. If you still see this error:

1. Verify your `.kiro/settings/mcp.json` includes these environment variables:
```json
"env": {
  "PYTHONIOENCODING": "utf-8",
  "PYTHONUTF8": "1"
}
```

2. Set system-wide UTF-8 support (Windows 10/11):
   - Open Settings → Time & Language → Language → Administrative language settings
   - Click "Change system locale"
   - Check "Beta: Use Unicode UTF-8 for worldwide language support"
   - Restart your computer

3. Or set environment variables globally:
```powershell
[System.Environment]::SetEnvironmentVariable('PYTHONIOENCODING', 'utf-8', 'User')
[System.Environment]::SetEnvironmentVariable('PYTHONUTF8', '1', 'User')
```

4. Restart Kiro after making changes

### Issue: "Authentication failed"

**Solution:**
1. Verify your GitHub token is valid: https://github.com/settings/tokens
2. Ensure token has correct scopes (`repo`, `workflow`)
3. Check token hasn't expired
4. Regenerate token if needed and update environment variable

### Issue: "uvx command not found"

**Solution:**
1. Install UV package manager (see Step 3 above)
2. Ensure UV is in your PATH
3. Restart your terminal/PowerShell
4. Restart Kiro

### Issue: "Permission denied accessing repository"

**Solution:**
1. Verify token has `repo` scope
2. Check you have access to the repository
3. For private repos, ensure token has private repo access

### Issue: "Rate limit exceeded"

**Solution:**
- GitHub API has rate limits (5000 requests/hour for authenticated users)
- Wait for the rate limit to reset
- Check rate limit status: https://api.github.com/rate_limit

## Security Best Practices

1. **Never commit tokens to Git**
   - The `.gitignore` already excludes `.env` files
   - Always use environment variables

2. **Use minimal scopes**
   - Only grant necessary permissions
   - Review token scopes periodically

3. **Rotate tokens regularly**
   - Set expiration dates
   - Regenerate tokens every 90 days

4. **Revoke unused tokens**
   - Go to https://github.com/settings/tokens
   - Delete tokens you no longer need

5. **Use fine-grained tokens (optional)**
   - GitHub offers fine-grained personal access tokens
   - More granular control over permissions
   - Repository-specific access

## Alternative: Using .env File

If you prefer not to set system environment variables, you can use a `.env` file:

1. Create `.env` in project root:
```bash
GITHUB_TOKEN=your_github_token_here
```

2. Update MCP config to load from .env:
```json
{
  "mcpServers": {
    "github": {
      "command": "uvx",
      "args": ["mcp-server-github"],
      "env": {
        "GITHUB_PERSONAL_ACCESS_TOKEN": "${GITHUB_TOKEN}"
      }
    }
  }
}
```

3. Ensure `.env` is in `.gitignore` (already included)

**Note**: This method requires Kiro to support loading environment variables from `.env` files.

## Testing MCP Server

After setup, test the GitHub MCP server by asking Kiro:

```
"Can you check the status of the latest CI/CD workflow run?"
```

```
"Show me the logs from the failed CI/CD job"
```

```
"List all open issues in the repository"
```

If these work, your MCP server is properly configured! 🎉

## Additional MCP Servers

You can add more MCP servers to enhance Kiro's capabilities:

### Git MCP Server
```json
"git": {
  "command": "uvx",
  "args": ["mcp-server-git"],
  "disabled": false
}
```

### Brave Search MCP Server
```json
"brave-search": {
  "command": "uvx",
  "args": ["mcp-server-brave-search"],
  "env": {
    "BRAVE_API_KEY": "${BRAVE_API_KEY}"
  },
  "disabled": false
}
```

### PostgreSQL MCP Server
```json
"postgres": {
  "command": "uvx",
  "args": ["mcp-server-postgres"],
  "env": {
    "DATABASE_URL": "${DATABASE_URL}"
  },
  "disabled": false
}
```

## Resources

- **MCP Documentation**: https://modelcontextprotocol.io/
- **GitHub MCP Server**: https://github.com/modelcontextprotocol/servers/tree/main/src/github
- **UV Package Manager**: https://docs.astral.sh/uv/
- **GitHub API**: https://docs.github.com/en/rest

---

**Need help?** Check the Kiro documentation or ask in the Kiro community!
