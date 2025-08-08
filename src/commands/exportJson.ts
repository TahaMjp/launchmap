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
import { runPythonParser } from '../python/runParser';
import { getLastParsedData, setLastParsedData } from './openVisualizer';

export function registerExportJson(context: vscode.ExtensionContext) {
  context.subscriptions.push(
    vscode.commands.registerCommand('launchmap.exportAsJson', async (): Promise<string | null> => {
      let graphDataToExport = null;

      const editor = vscode.window.activeTextEditor;

      if (editor) {
        const filePath = editor.document.fileName;
        try {
          const result = await runPythonParser(filePath);
          graphDataToExport = JSON.parse(result);
          setLastParsedData(graphDataToExport);
        } catch {
          vscode.window.showErrorMessage('Failed to parse the launch file.');
          return null;
        }
      } else {
        graphDataToExport = getLastParsedData();
        if (!graphDataToExport) {
          vscode.window.showErrorMessage('No active editor and no graph data available to export.');
          return null;
        }
      }

      const uri = await vscode.window.showSaveDialog({
        filters: { 'JSON': ['json'] },
        defaultUri: vscode.Uri.file('launch_graph.json'),
        saveLabel: 'Export Launch Graph'
      });

      if (!uri) return null;

      const jsonString = JSON.stringify(graphDataToExport, null, 2);
      try {
        await vscode.workspace.fs.writeFile(uri, Buffer.from(jsonString, 'utf8'));
        vscode.window.showInformationMessage(`Launch graph exported to ${uri.fsPath}`);
        return uri.fsPath;
      } catch (error) {
        vscode.window.showErrorMessage('Failed to save JSON: ' + (error as Error).message);
        return null;
      }
    })
  );
}
