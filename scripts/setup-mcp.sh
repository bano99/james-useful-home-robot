#!/bin/bash

# Script to set up MCP servers for Kiro
# This script helps configure GitHub MCP server

set -e

echo "=========================================="
echo "MCP Server Setup for Kiro"
echo "=========================================="
echo ""

# Check if uv is installed
if ! command -v uv &> /dev/null; then
    echo "[!] UV package manager not found"
    echo ""
    echo "Installing UV..."
    curl -LsSf https://astral.sh/uv/install.sh | sh
    
    # Add to PATH for current session
    export PATH="$HOME/.cargo/bin:$PATH"
    
    if command -v uv &> /dev/null; then
        echo "[OK] UV installed successfully"
        echo ""
        echo "Please run this script again or source your shell config:"
        echo "  source ~/.bashrc  # or ~/.zshrc"
        exit 0
    else
        echo "[X] Failed to install UV"
        echo "Please install manually from: https://docs.astral.sh/uv/"
        exit 1
    fi
fi

echo "[OK] UV package manager found"
uv --version
echo ""

# Check if GITHUB_TOKEN is set
if [ -n "$GITHUB_TOKEN" ]; then
    echo "[OK] GITHUB_TOKEN environment variable is set"
    echo "Token: ${GITHUB_TOKEN:0:4}****${GITHUB_TOKEN: -4}"
    echo ""
else
    echo "[!] GITHUB_TOKEN environment variable not set"
    echo ""
    echo "To set up GitHub MCP server, you need a Personal Access Token:"
    echo ""
    echo "1. Go to: https://github.com/settings/tokens"
    echo "2. Click 'Generate new token (classic)'"
    echo "3. Name: 'Kiro MCP Server - James Robot'"
    echo "4. Select scopes: repo, workflow"
    echo "5. Generate and copy the token"
    echo ""
    read -p "Paste your GitHub token here (or press Enter to skip): " TOKEN
    
    if [ -n "$TOKEN" ]; then
        echo ""
        echo "Setting GITHUB_TOKEN environment variable..."
        
        # Determine shell config file
        if [ -f "$HOME/.zshrc" ]; then
            SHELL_CONFIG="$HOME/.zshrc"
        elif [ -f "$HOME/.bashrc" ]; then
            SHELL_CONFIG="$HOME/.bashrc"
        else
            SHELL_CONFIG="$HOME/.profile"
        fi
        
        # Add to shell config if not already present
        if ! grep -q "GITHUB_TOKEN" "$SHELL_CONFIG"; then
            echo "" >> "$SHELL_CONFIG"
            echo "# GitHub Personal Access Token for MCP" >> "$SHELL_CONFIG"
            echo "export GITHUB_TOKEN=\"$TOKEN\"" >> "$SHELL_CONFIG"
            echo "[OK] GITHUB_TOKEN added to $SHELL_CONFIG"
        else
            echo "[!] GITHUB_TOKEN already exists in $SHELL_CONFIG"
            echo "Please update it manually if needed"
        fi
        
        # Set for current session
        export GITHUB_TOKEN="$TOKEN"
        
        echo ""
        echo "IMPORTANT: Run 'source $SHELL_CONFIG' or restart your terminal"
    else
        echo ""
        echo "Skipped token setup. You can set it later with:"
        echo "  echo 'export GITHUB_TOKEN=\"your_token_here\"' >> ~/.bashrc"
        echo "  source ~/.bashrc"
    fi
    echo ""
fi

echo "=========================================="
echo "MCP Configuration Status"
echo "=========================================="
echo ""

if [ -f ".kiro/settings/mcp.json" ]; then
    echo "[OK] MCP configuration file exists"
    echo "Location: .kiro/settings/mcp.json"
else
    echo "[!] MCP configuration file not found"
    echo "Expected location: .kiro/settings/mcp.json"
fi
echo ""

echo "=========================================="
echo "Next Steps"
echo "=========================================="
echo ""
echo "1. Ensure GITHUB_TOKEN is set (see above)"
echo "2. Restart Kiro completely"
echo "3. Check MCP Server view in Kiro"
echo "4. Test by asking: 'Show me the latest CI/CD logs'"
echo ""
echo "For detailed instructions, see: docs/mcp_setup.md"
echo ""
