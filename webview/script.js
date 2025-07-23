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

import './components/setupRenderers.js';
import { renderAll } from './core/renderAll.js';

window.addEventListener('message', (event) => {
    const message = event.data;
    if (message.type == 'launchmap-data') {
        renderAll(message.data);
    }
});

const vscode = acquireVsCodeApi();
document.getElementById('export-btn')?.addEventListener('click', () => {
    vscode.postMessage({ type: 'export-json' });
});