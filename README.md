# 🚀 LaunchMap – Visualize Your ROS 2 Launch Files

<p align="center">
  <img src="assets/launchmap-logo.png" width="150" alt="LaunchMap Logo">
</p>

[![VSCode Marketplace](https://img.shields.io/visual-studio-marketplace/v/KodoRobotics.launchmap?label=VSCode%20Marketplace)](https://marketplace.visualstudio.com/items?itemName=KodoRobotics.launchmap)
[![License](https://img.shields.io/github/license/Kodo-Robotics/launchmap?color=blue)](./LICENSE)

**LaunchMap** is a Visual Studio Code extension that lets you visualize the structure of ROS 2 launch files as interactive graphs directly inside VSCode.

Whether you are debugging a complex `launch.py`, exploring a new package, or onboarding to a robotics stack, LaunchMap helps you **see what is really happening** in your launch files.

![LaunchMap Demo](assets/launchmap-demo.gif)

---

## ✨ Features

- Visualizes ROS 2 launch files as interactive graphs
- Supports core launch constructs like:
  - `Node(...)`
  - `IncludeLaunchDescription(...)`
  - `GroupAction(...)`
  - `DeclareLaunchArgument(...)`
  - `LaunchConfiguration(...)`
- Traces argument usage and include relationships
- Opens the graph in a **new tab** within VSCode
- Displays a warning banner for unsupported or skipped components

---

## 📦 Installation

Install from the [Visual Studio Code Marketplace](https://marketplace.visualstudio.com/items?itemName=KodoRobotics.launchmap), or use the CLI:

```bash
code --install-extension KodoRobotics.launchmap
```

---

## ▶️ How to Use
1. Open a `.launch.py` file in VSCode.
2. Open the Command Palette (`Ctrl+Shift+P` or `Cmd+Shift+P`).
3. Run: LaunchMap: Visualize Launch File
4. The graph will open in a new editor tab.

---

## 🛠 Supported Constructs

| Construct                    | Status             |
|-----------------------------|--------------------|
| `Node(...)`                 | ✅ Supported        |
| `IncludeLaunchDescription(...)` | ✅ Supported    |
| `GroupAction(...)`          | ✅ Supported        |
| `DeclareLaunchArgument(...)`| ✅ Supported        |
| `LaunchConfiguration(...)`  | ✅ Supported        |
| `SetParameter(...)`         | ⚠️ Not yet parsed   |
| `ComposableNode(...)`       | ⚠️ Not yet parsed   |
| `IfCondition(...)`          | ⚠️ Not yet parsed  |

> A banner at the top of the graph view shows which components were skipped.

---

## 🐛 Known Limitations

- Some advanced constructs like ComposableNode are skipped.
- Launch files are parsed statically (not executed).
- Layout may be suboptimal for large or deeply nested graphs.

---

## 📜 License

Licensed under the Apache 2.0 License.
Copyright © 2025 Kodo Robotics.

---

## 🤝 Contributing

Contributions are welcome!
Please open an issue, suggest a feature, or submit a pull request.