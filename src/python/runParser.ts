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
import { getPluginDir } from '../utils/launchmapConfig';

export async function runPythonParser(filePath: string): Promise<string> {
  // eslint-disable-next-line no-async-promise-executor
  return new Promise(async (resolve, reject) => {
    const pythonCmd = detectPythonCommand();

    if (!pythonCmd) {
      vscode.window.showErrorMessage(
        'Python iterpreter not found. Please install Python 3 and make sure it is available in your PATH.'
      );
      return reject(new Error('No Python interpreter found.'));
    }

    const scriptPath = path.join(__dirname, '..', '..', 'parse.py');
    const cmdParts = [`"${pythonCmd}"`, `"${scriptPath}"`, `"${filePath}"`];

    const pluginDir = await getPluginDir();
    if (pluginDir) {
      cmdParts.push('--plugin-dir', `"${pluginDir}"`);
    }

    const cmd = cmdParts.join(' ');
    cp.exec(cmd, (err, stdout, stderr) => {
      if (err) {
        vscode.window.showErrorMessage('Parser error: ' + stderr);
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
    } catch {
      continue;
    }
  }
  return null;
}
