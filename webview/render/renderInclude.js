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

import { makeDraggable } from '../drag.js';
import { renderSection } from './renderSection.js';

export function renderIncludesGroup(container, includes, namespace, layoutCtx, options={}) {
    includes.forEach((include, idx) => {
        const path = options.pathPrefix ? `${options.pathPrefix}[${idx}]` : `includes[${idx}]`;
        const block = renderInclude(include, namespace, layoutCtx, { ...options, path });
        container.appendChild(block);
        layoutCtx.y += 100;
    });

    layoutCtx.x += 250;
    layoutCtx.y = 100;
}

function renderInclude(include, namespace, layoutCtx, options) {
    const block = document.createElement("div");
    block.className = "include-block";
    block.style.left = `${layoutCtx.x}px`;
    block.style.top = `${layoutCtx.y}px`;
    block.style.position = "absolute";

    // Title
    const title = document.createElement("div");
    title.className = "node-title";
    title.innerText = `${namespace}/Include`;
    block.appendChild(title);

    // Path
    const renderOptions = { includeLeftPort: true, portIdPrefix: options.path, portRegistry: options.portRegistry };
    const path = include.launch_description_source || "<unresolved>";
    block.appendChild(renderSection("launch_description_source", "📂", "Path", path, renderOptions));
    
    // Launch arguments
    const args = include.launch_arguments || {};
    block.appendChild(renderSection("launch_arguments", "📥", "Args", args, renderOptions))

    if (options.path) {
        block.dataset.path = options.path;
        block.dataset.type = "include";
        if (options.blockRegistry) {
            options.blockRegistry[options.path] = block;
        }
    }

    makeDraggable(block, {
        ...options,
        onDrag: () => {
            if (options.renderEdges && options.parsedData && options.argumentRegistry && options.blockRegistry) {
                options.renderEdges(options.parsedData, options.portRegistry);
            }
        }
    });
    return block;
}