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

import { renderComponent } from './renderComponent.js';
import './render/setupRenderers.js';
import { renderEdges } from './render/renderEdges.js';
import { enableZoomAndPan } from './zoomPanController.js';
import { getRegisteredRenderKeys } from './renderRegistry.js';

const argumentRegistry = {};
const blockRegistry = {};
const portRegistry = {};

window.addEventListener('message', (event) => {
    const message = event.data;
    if (message.type == 'launchmap-data') {
        renderAll(message.data);
    }
});

function renderAll(data) {
    const editor = document.getElementById("editor");
    editor.innerHTML = "";

    // Create zoom layer
    const zoomLayer = document.createElement("div");
    zoomLayer.id = "zoom-layer";
    editor.appendChild(zoomLayer);

    // Edges
    const edgeLayer = document.createElementNS("http://www.w3.org/2000/svg", "svg");
    edgeLayer.setAttribute("id", "edge-layer");
    edgeLayer.classList.add("edge-layer");
    zoomLayer.appendChild(edgeLayer);

    // Render
    const layoutCtx = { x: 100, y: 100 };
    const context = {
        parsedData: data,
        blockRegistry,
        argumentRegistry,
        portRegistry,
        renderEdges
    };

    const renderKeys = getRegisteredRenderKeys();

    for (const key of Object.keys(data)) {
        if (renderKeys.includes(key)) {
            const value = data[key];
            const typeHint = Array.isArray(value) ? key : value.type || key;
            renderComponent({ value: value, type: typeHint }, zoomLayer, layoutCtx, context);
            layoutCtx.y += 100;
        }
    }

    renderEdges(data, portRegistry);
    enableZoomAndPan(editor, zoomLayer, () => renderEdges(data, portRegistry));
}