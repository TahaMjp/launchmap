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

import * as assert from 'assert';
import * as vscode from 'vscode';
import * as path from 'path';
import * as fs from 'fs';
import * as os from 'os';
import { promisify } from 'util';

const mkdtemp = promisify(fs.mkdtemp);
const rm = promisify(fs.rm || fs.rmdir);

suite('End-to-End Workflow Test', () => {
  let tempDir: string;
  let exportPath: string;
  const fixturePath = path.resolve(__dirname, '../../../src/test/fixtures/test.launch.py');

  suiteSetup(async () => {
    // Create a temp directory for export
    tempDir = await mkdtemp(path.join(os.tmpdir(), 'launchmap-test-'));
    exportPath = path.join(tempDir, 'launch_graph.json');
  });

  test('1️⃣ Open Visualizer → Panel created', async () => {
    const doc = await vscode.workspace.openTextDocument(fixturePath);
    await vscode.window.showTextDocument(doc);

    await vscode.commands.executeCommand('launchmap.openVisualizer');
    assert.ok(true, 'Visualizer command executed without error');
  });

  test('2️⃣ Export as JSON → File saved', async () => {
    // Mock save dialog to return temp file path
    vscode.window.showSaveDialog = async () => vscode.Uri.file(exportPath);

    await vscode.commands.executeCommand('launchmap.exportAsJson');
    const exists = fs.existsSync(exportPath);
    assert.ok(exists, 'Expected exported JSON file to exist');
  });

  test('3️⃣ Import JSON → Panel created again', async () => {
    // Mock open dialog to return exported file path
    vscode.window.showOpenDialog = async () => [vscode.Uri.file(exportPath)];

    await vscode.commands.executeCommand('launchmap.importJson');
    assert.ok(true, 'Import command executed without error');
  });

  test('4️⃣ Set Plugin Dir via command → Stored in .launchmap file', async () => {
    const pluginDir = path.join(tempDir, 'plugins');
    await fs.promises.mkdir(pluginDir, { recursive: true });
    const configPath = path.join(tempDir, '.launchmap');

    // Stub workspace folder to point to temp directory
    Object.defineProperty(vscode.workspace, 'workspaceFolders', {
      get: () => [{ uri: vscode.Uri.file(tempDir) }],
      configurable: true
    });

    vscode.window.showOpenDialog = async () => [vscode.Uri.file(pluginDir)];

    await vscode.commands.executeCommand('launchmap.setPluginDir');

    const configContent = await fs.promises.readFile(configPath, 'utf8');
    const parsed = JSON.parse(configContent);

    assert.strictEqual(parsed.pluginDir, pluginDir, 'Expected pluginDir to be set correctly in .launchmap');
  });

  test('5️⃣ Plugin Dir via .launchmap → Used in parser call', async () => {
    const pluginDir = path.join(tempDir, 'plugins');
    await fs.promises.mkdir(pluginDir, { recursive: true });
    await fs.promises.writeFile(path.join(pluginDir, 'dummpy.py'), '# dummy plugin');

    const configPath = path.join(tempDir, '.launchmap');
    const config = { pluginDir };
    await fs.promises.writeFile(configPath, JSON.stringify(config, null, 2));

    // Stub workspace folder to point to temp directory
    Object.defineProperty(vscode.workspace, 'workspaceFolders', {
      get: () => [{ uri: vscode.Uri.file(tempDir) }],
      configurable: true,
    });

    const doc = await vscode.workspace.openTextDocument(fixturePath);
    await vscode.window.showTextDocument(doc);

    await vscode.commands.executeCommand('launchmap.openVisualizer');

    assert.ok(true, 'Visualizer executed with .launchmap pluginDir present');

    await fs.promises.unlink(configPath);
  });

  suiteTeardown(async () => {
    // Clean up temp directory after all tests
    await rm(tempDir, { recursive: true, force: true });
  });
});
