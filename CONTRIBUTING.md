# 🤝 Contributing to LaunchMap

Thank you for your interest in contributing to LaunchMap!  
Contributions of all kinds, features, bug fixes, tests, docs, and feedback are welcome!

---

## 🛠️ Project Structure
```txt
launchmap/
├── parser/            # Python-based static launch file parser
├── src/               # VS Code extension (TypeScript)
├── webview/           # Webview frontend (HTML/JS)
```

---

## ⚙️ Setup Instructions

### Installation

```bash
# Clone the repo
git clone https://github.com/Kodo-Robotics/launchmap
cd launchmap

# Python setup
python3 -m venv .venv
source .venv/bin/activate
pip install -r parser/requirements.txt

# Node setup
npm install
```

### Running

Here is how you can get the extension running and see the output:

1. Compile the code
```bash
npm run compile
```
This converts the TypeScript code into JavaScript so VS Code can run it.

2. Start the extension in a debug VS Code window
  - Open the file `src/extension.ts`.
  - Press F5 (or go to Run and Debug → Launch Extension).

This launches a new VS Code window with the extension loaded in "dev mode" so you can test your changes live.

3. Open a sample launch file
  - In the new debug VS Code window, navigate to:
`parser/tests/real_cases/launch_files/`
  - Open any `.launch.py` file (these are real world test cases).

4. Run the LaunchMap visualizer
  - Press `Cmd + Shift + P` (or `Ctrl + Shift + P` on Windows/Linux).
  - Search for `LaunchMap: Open Launch Visualizer` and run it.

You should now see the visualization output.

---

## 🧹 Linting & Code Style

We use:
- `ruff` for Python (ultra-fast linter + formatter)
- `eslint` for TypeScript/JavaScript

### 🔍 Run Python Lint (parser)

```bash
# Test
npm run lint:py 

# Fix
npm run lint:py:fix
```

### 🔍 Run JS/TS Lint (extension + webview)

```bash
# Test
npm run lint:js 

# Fix
npm run lint:js:fix
```

---

## 🧪 Running Tests

```bash
# Parser Tests
npm run test:parser

# Webview Visual Tests
npm run test:webview

# Extension Integration Tests
npm run test:src
```

### ⚠️ Notes on Snapshot Testing
- Visual snapshots are stored under `webview/tests/__screenshots__/`
- Snapshots may differ between macOS, Windows, and Linux
- Snapshots are only validated on PRs in CI (Linux)
- If you are on macOS/Windows, avoid committing updated snapshots unless necessary

---

## 💡 Tips
- Prefer small, focused PRs
- Follow lint rules strictly (ruff and eslint)
- Playwright tests are validated automatically on PRs
- Use `npx playwright test --update-snapshots` only on Linux for visual diffs

---

## 📬 Questions?
- Join our [Discord Server](https://discord.gg/EK3pHZxrKy)
- Ask on GitHub

Thanks again for contributing! 🚀