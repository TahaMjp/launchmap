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
import * as fs from 'fs/promises';

const CONFIG_FILENAME = '.launchmap';

export async function getPluginDir(): Promise<string | null> {
    const workspaceFolder = vscode.workspace.workspaceFolders?.[0];
    if (!workspaceFolder) return null;

    const configPath = path.join(workspaceFolder.uri.fsPath, CONFIG_FILENAME);

    try {
        const content = await fs.readFile(configPath, 'utf-8');
        const json = JSON.parse(content);
        return json.pluginDir || null;
    } catch {
        return null;
    }
}

export async function setPluginDir(pluginDir: string): Promise<void> {
    const workspaceFolder = vscode.workspace.workspaceFolders?.[0];
    if (!workspaceFolder) {
        vscode.window.showErrorMessage("No workspace folder found to save plugin directory.");
        return;
    }

    const configPath = path.join(workspaceFolder.uri.fsPath, CONFIG_FILENAME);
    const config = { pluginDir };

    await fs.writeFile(configPath, JSON.stringify(config, null, 2), 'utf-8');
    vscode.window.showInformationMessage(`Plugin directory saved to ${CONFIG_FILENAME}`);
}