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
import * as path from 'path';
import { createVisualizerPanel } from '../panel/createVisualizerPanel';
import { setLastParsedData } from './openVisualizer';

export function registerImportJson(context: vscode.ExtensionContext) {
  context.subscriptions.push(
    vscode.commands.registerCommand('launchmap.importJson', async () => {
      const fileUris = await vscode.window.showOpenDialog({
        canSelectMany: false,
        filters: { 'JSON': ['json'] },
        openLabel: 'Import Launch Graph JSON'
      });

      if (!fileUris || fileUris.length === 0) return;

      try {
        const importedName = path.basename(fileUris[0].fsPath);

        const contentBytes = await vscode.workspace.fs.readFile(fileUris[0]);
        const content = Buffer.from(contentBytes).toString('utf8');
        const parsed = JSON.parse(content);

        setLastParsedData(parsed);
        createVisualizerPanel(context, parsed, importedName);
      } catch {
        vscode.window.showErrorMessage('Failed to load or parse the JSON file.');
      }
    })
  );
}
