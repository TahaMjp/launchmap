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
import { getWebviewHtml } from './getWebviewHtml';

export function createVisualizerPanel(
  context: vscode.ExtensionContext,
  data: null,
  titleHint?: string
): vscode.WebviewPanel {
  const panelTitle = titleHint
    ? `${titleHint} : ROS2 LaunchMap Visualizer`
    : 'ROS2 LaunchMap Visualizer';

  const panel = vscode.window.createWebviewPanel(
    'launchmap',
    panelTitle,
    vscode.ViewColumn.One,
    {
      enableScripts: true,
      retainContextWhenHidden: true
    }
  );

  panel.webview.html = getWebviewHtml(panel.webview, context.extensionUri);
  panel.webview.postMessage({ type: 'launchmap-data', data });

  panel.webview.onDidReceiveMessage(async (message) => {
    if (message.type === 'export-json') {
      const result = await vscode.commands.executeCommand('launchmap.exportAsJson');
      if (result) {
        panel.webview.postMessage({ type: 'export-complete', path: result });
      }
    }
  });

  panel.onDidChangeViewState((e) => {
    if (e.webviewPanel.visible) {
      panel.webview.postMessage({ type: 'launchmap-data', data });
    }
  });

  return panel;
}
