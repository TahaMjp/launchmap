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

import { registrySystem } from "./registry.js";
import { renderComponent } from "../components/renderComponent.js";
import { enableZoomAndPan } from "./zoomPanController.js";
import { renderEdges } from "./renderEdges.js";
import { getRegisteredRenderKeys } from "./dispatcher.js";
import { LayoutManager } from "./layoutManager.js";

export function renderAll(data) {
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
    const layoutManager = new LayoutManager();
    const context = {
        parsedData: data,
        portRegistry: registrySystem.portRegistry,
        renderEdges,
        renderBlock: (block, columnType) => { 
            requestAnimationFrame(() => {
                layoutManager.placeBlock(block, columnType);
            }); 
        }
    };

    const renderKeys = getRegisteredRenderKeys();
    for (const key of Object.keys(data)) {
        if (renderKeys.includes(key)) {
            const value = data[key];
            const typeHint = Array.isArray(value) ? key : value.type || key;
            renderComponent({ value: value, type: typeHint }, zoomLayer, context);
        }
    }

    renderEdges(data, registrySystem.portRegistry);
    enableZoomAndPan(editor, zoomLayer, () => renderEdges(data, registrySystem.portRegistry));
}