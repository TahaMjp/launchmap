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

export function renderGroupGroup(container, groups, namespace, layoutCtx, options = {}) {
    groups.forEach((group, idx) => {
        const path = options.pathPrefix ? `${options.pathPrefix}[${idx}]` : `groups[${idx}]`;
        renderGroup(group, path, container, layoutCtx, { ...options, path });
    });

    layoutCtx.x += 350;
    layoutCtx.y = 100;
}

export function renderGroup(group, prefix, container, layoutCtx, options = {}) {
    const ns = group.namespace || prefix;

    const groupBox = createBaseBlock({
        type: "group",
        label: ns,
        layoutCtx,
        options: {
            ...options,
            path: prefix
        }
    });

    // Header
    const header = document.createElement("div");
    header.className = "group-header";

    // Title
    const title = document.createElement("div");
    title.className = "group-title";
    title.innerText = `📦 Group: ${ns}`;
    header.appendChild(title);

    // Namespace
    if (group.namespace) {
        const nsSection = renderSection("namespace", "🧭", "Namespace", group.namespace, 
            { includeLeftPort: true, portIdPrefix: options.path, portRegistry: options.portRegistry });
        nsSection.classList.add("namespace-section");
        header.appendChild(nsSection);
    }
    groupBox.appendChild(header);

    // Body
    const body = document.createElement("div");
    body.className = "group-body";
    groupBox.appendChild(body);

    const innerLayout = { x: 20, y: 40 };
    const childOptions = { 
        ...options,
        stopPropagation: true, 
        constrainToParent: true,
        pathPrefix: `${prefix}.actions`
    };

    const actions = group.actions || {};
    for (const [key, value] of Object.entries(actions)) {
        renderComponent({ type: key, value: value, namespace: ns }, body, innerLayout, childOptions);
    }
    container.appendChild(groupBox);

    renderAutoResizableBody(groupBox, "block");

    layoutCtx.x += 350;
}