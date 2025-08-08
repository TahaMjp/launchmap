#!/usr/bin/env bash
set -e

echo "🚀 Starting environment setup..."

# Install Python
if ! command -v python3 &>/dev/null; then
    echo "⚠️ Python3 not found. Please install Python 3.8+ manually."
    exit 1
fi
echo "✅ Python version: $(python3 --version)"

# Install Node.js + npm
if ! command -v npm &>/dev/null; then
    if [[ "$OSTYPE" == "linux-gnu"* ]]; then
        echo "⚠️ npm not found. Installing Node.js..."
        curl -fsSL https://deb.nodesource.com/setup_20.x | sudo -E bash -
        sudo apt-get install -y nodejs
    elif [[ "$OSTYPE" == "darwin"* ]]; then
        brew install node
    else
        echo "❌ Unsupported OS for automatic Node.js install"
        exit 1
    fi
fi
echo "✅ Node.js version: $(node -v)"
echo "✅ npm version: $(npm -v)"

# Create virtual environment
if [ ! -d ".venv" ]; then
    python3 -m venv .venv
    echo "✅ Created virtual environment in .venv"
fi

# Activate virtual environment
source .venv/bin/activate
echo "✅ Activated virtual environment"

# Install Python Dependencies
if [ -f "requirements.txt" ]; then
    pip install --upgrade pip
    pip install -r requirements.txt
    echo "✅ Installed Python dependencies"
else
    echo "⚠️ No requirements.txt found, skipping Python deps install"
fi

# Install npm Dependencies
if [ -f "package.json" ]; then
    npm install
    echo "✅ Installed npm dependencies"
else
    echo "⚠️ No package.json found, skipping npm deps install"
fi

# Install Playwright & browser dependencies
echo "📦 Installing Playwright..."
npm install -D playwright
npx playwright install --with-deps
echo "✅ Playwright installed with dependencies"

echo "🎉 Setup complete!"