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

import { createBaseBlock } from '../utils/baseBlock.js';
import { renderAutoResizableBody } from './renderAutoResizableBody.js';
import { renderComponent } from './renderComponent.js';
import { renderSection } from './renderSection.js';

export function renderComposableContainerGroup(container, groups, namespace, layoutCtx, options = {}) {
    groups.forEach((group, idx) => {
        const path = options.pathPrefix ? `${options.pathPrefix}.composable_nodes[${idx}]` : `composable_nodes[${idx}]`;
        renderComposableContainer(group, container, layoutCtx, { ...options, path });
    });

    layoutCtx.x += 350;
    layoutCtx.y = 100;
}

export function renderComposableContainer(group, container, layoutCtx, options = {}) {
    const composableContainer = createBaseBlock({
        type: "composable-container",
        layoutCtx,
        options: {
            ...options,
            path: options.path
        }
    });

    // Header
    const header = document.createElement("div");
    header.className = "composable-container-header";

    // Render additional sections
    const metaSections = [
        { key: "container", icon: "📛", label: "Container", value: group.target_container },
        { key: "package", icon: "📦", label: "Package", value: group.package },
        { key: "executable", icon: "▶️", label: "Executable", value: group.executable },
        { key: "output", icon: "🖥️", label: "Output", value: group.output}
    ];

    metaSections.forEach(( {key, icon, label, value }) => {
        if (value) {
            const section = renderSection(key, icon, label, value, {
                includeLeftPort: true,
                portIdPrefix: options.path,
                portRegistry: options.portRegistry
            });
            header.appendChild(section);
        }
    });

    composableContainer.append(header);

    // Body
    const body = document.createElement("div");
    body.className = "composable-container-body";
    composableContainer.appendChild(body);

    const innerLayout = { x: 20, y: 40 };
    const childOptions = { 
        ...options,
        stopPropagation: true, 
        constrainToParent: true,
        pathPrefix: `${options.path}.composable_nodes`
    };

    // Render composable nodes
    renderComponent(
        { type: "composable_nodes", value: group.composable_nodes, namespace: group.namespace }, 
        body, innerLayout, childOptions
    );
    container.appendChild(composableContainer);

    renderAutoResizableBody(composableContainer, "block", [".composable-container-header"]);

    layoutCtx.x += 350;
}