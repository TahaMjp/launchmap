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

            // Export as Json button
            panel.webview.onDidReceiveMessage(message => {
                if (message.type === 'export-json') {
                    vscode.commands.executeCommand('launchmap.exportAsJson');
                }
            });

            // Resend on tab refocus
            panel.onDidChangeViewState((e) => {
                if (e.webviewPanel.visible && lastParsedData) {
                    panel.webview.postMessage({ type: 'launchmap-data', data: lastParsedData });
                }
            });
        })
    );

    context.subscriptions.push(
        vscode.commands.registerCommand('launchmap.exportAsJson', async () => {
            let graphDataToExport = null;

            const editor = vscode.window.activeTextEditor;

            if (editor) {
                const filePath = editor.document.fileName;
                try {
                    const result = await runPythonParser(filePath);
                    graphDataToExport = JSON.parse(result);
                } catch (error) {
                    vscode.window.showErrorMessage("Failed to parse the launch file.");
                    return;
                }
            } else if (lastParsedData) {
                graphDataToExport = lastParsedData;
            } else {
                vscode.window.showErrorMessage("No active editor and no graph data available to export.");
                return;
            }

            const uri = await vscode.window.showSaveDialog({
                filters: { 'JSON': ['json'] },
                defaultUri: vscode.Uri.file('launch_graph.json'),
                saveLabel: 'Export Launch Graph'
            });

            if (!uri) return;

            const jsonString = JSON.stringify(graphDataToExport, null, 2);
            try {
                await vscode.workspace.fs.writeFile(uri, Buffer.from(jsonString, 'utf8'));
                vscode.window.showInformationMessage(`Launch graph exported to ${uri.fsPath}`);
            } catch (error) {
                vscode.window.showErrorMessage("Failed to save JSON: " + (error as Error).message);
            }
        })
    );

    context.subscriptions.push(
        vscode.commands.registerCommand('launchmap.importJson', async () => {
            const fileUris = await vscode.window.showOpenDialog({
                canSelectMany: false,
                filters: { 'JSON': ['json'] },
                openLabel: 'Import Launch Graph JSON'
            });

            if (!fileUris || fileUris.length === 0) return;

            const fileUri = fileUris[0];

            try {
                const contentBytes = await vscode.workspace.fs.readFile(fileUri);
                const content = Buffer.from(contentBytes).toString('utf8');
                const parsed = JSON.parse(content);
                lastParsedData = parsed;

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

                // Export as Json button
                panel.webview.onDidReceiveMessage(message => {
                    if (message.type === 'export-json') {
                        vscode.commands.executeCommand('launchmap.exportAsJson');
                    }
                });

                // Resend on tab refocus
                panel.onDidChangeViewState((e) => {
                    if (e.webviewPanel.visible && lastParsedData) {
                        panel.webview.postMessage({ type: 'launchmap-data', data: lastParsedData });
                    }
                });

            } catch (error) {
                vscode.window.showErrorMessage("Failed to load or parse the JSON file.");
            }
        })
    )
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
            <button id="export-btn">💾 Export JSON</button>
            <div id="editor">
            </div>
            <script src="${scriptUri}" type="module"></script>
        </body>
        </html>
    `
}