// Copyright (c) 2025 Kodo Robotics
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

import * as vscode from 'vscode';
import * as cp from 'child_process';
import * as path from 'path';
import * as which from 'which';

let lastParsedData: any = null;

export function activate(context: vscode.ExtensionContext) {
    context.subscriptions.push(
        vscode.commands.registerCommand('launchmap.openVisualizer', async () => {
            const editor = vscode.window.activeTextEditor;
            if (!editor) {
                vscode.window.showErrorMessage("No active editor with a launch file.");
                return;
            }

            const filePath = editor.document.fileName;
            const result = await runPythonParser(filePath);

            lastParsedData = JSON.parse(result);

            const panel = vscode.window.createWebviewPanel(
                'launchmap',
                'ROS2 LaunchMap Visualizer',
                vscode.ViewColumn.One,
                {
                    enableScripts: true,
                    retainContextWhenHidden: true
                }
            );

            panel.webview.html = getWebviewHtml(panel.webview, context.extensionUri);

            // Initial send
            panel.webview.postMessage({ type: 'launchmap-data', data: lastParsedData });

            // Resend on tab refocus
            panel.onDidChangeViewState((e) => {
                if (e.webviewPanel.visible && lastParsedData) {
                    panel.webview.postMessage({ type: 'launchmap-data', data: lastParsedData });
                }
            });
        })
    );
}

async function runPythonParser(filePath: string): Promise<string> {
    return new Promise((resolve, reject) => {
        const pythonCmd = detectPythonCommand();

        if (!pythonCmd) {
            vscode.window.showErrorMessage(
                "Python iterpreter not found. Please install Python 3 and make sure it is available in your PATH."  
            );
            return reject(new Error("No Python interpreter found."));
        }

        const scriptPath = path.join(__dirname, '..', 'parse.py');
        const cmd = `"${pythonCmd}" "${scriptPath}" "${filePath}"`;

        cp.exec(cmd, (err, stdout, stderr) => {
            if (err) {
                vscode.window.showErrorMessage("Parser error: " + stderr);
                return reject(err);
            }
            resolve(stdout);
        });
    });
}

function detectPythonCommand(): string | null {
    const candidates = ['python3', 'python', 'py'];
    for (const cmd of candidates) {
        try {
            which.sync(cmd);
            return cmd;
        } catch (_) {
            continue;
        }
    }
    return null;
}

function getWebviewHtml(webview: vscode.Webview, extensionUri: vscode.Uri): string {
    const scriptUri = webview.asWebviewUri(vscode.Uri.joinPath(extensionUri, 'webview', 'script.js'));
    const styleUri = webview.asWebviewUri(vscode.Uri.joinPath(extensionUri, 'webview', 'style.css'));

    return `
        <!DOCTYPE html>
        <html>
        <head>
            <meta charset="UTF-8">
            <meta http-equiv="Content-Security-Policy" content="default-src 'none'; script-src 'unsafe-inline' 'unsafe-eval' ${webview.cspSource}; style-src ${webview.cspSource};">
            <link href="${styleUri}" rel="stylesheet">
        </head>
        <body>
            <div id="mvp-banner">
                🛠️ Some components like <code>ComposableNode</code>, <code>IfCondition</code> aren’t visualized yet. This is an early version — more coming soon!
            </div>
            <div id="editor">
            </div>
            <script src="${scriptUri}" type="module"></script>
        </body>
        </html>
    `
}