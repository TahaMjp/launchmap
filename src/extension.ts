import * as vscode from 'vscode';
import * as cp from 'child_process';
import * as path from 'path';

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
        const scriptPath = path.join(__dirname, '..', 'parse.py');
        cp.exec(`python3 "${scriptPath}" "${filePath}"`, (err, stdout, stderr) => {
            if (err) {
                vscode.window.showErrorMessage("Parser error: " + stderr);
                reject(err);
            } else {
                resolve(stdout);
            }
        });
    });
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
            <div id="editor"></div>
            <script src="${scriptUri}" type="module"></script>
        </body>
        </html>
    `
}