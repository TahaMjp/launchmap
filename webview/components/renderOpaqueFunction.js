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

import { renderBaseBlock } from './renderBaseBlock.js';
import { renderAutoResizableBody } from './renderAutoResizableBody.js';
import { renderComponent } from './renderComponent.js';

export function renderOpaqueFunctionGroup(container, opaqueFcns, namespace, layoutCtx, options = {}) {
    opaqueFcns.forEach((opaqueFcn, idx) => {
        const path = options.pathPrefix ? `${options.pathPrefix}.opaque_functions[${idx}]` : `opaque_functions[${idx}]`;
        renderOpaqueFunction(opaqueFcn, namespace, path, container, layoutCtx, { ...options, path });
    });

    layoutCtx.x += 350;
    layoutCtx.y = 100;
}

export function renderOpaqueFunction(opaqueFcn, namespace, prefix, container, layoutCtx, options = {}) {
    const fcnBox = renderBaseBlock({
        type: "opaque-function",
        layoutCtx,
        options: {
            ...options,
            events: opaqueFcn.events
        }
    });

    // Header
    const header = document.createElement("div");
    header.className = "opaque-function-header";

    // Title
    const title = document.createElement("div");
    title.className = "opaque-function-title";
    title.innerText = `${opaqueFcn.name}`;
    header.appendChild(title);

    fcnBox.append(header);

    // Body
    const body = document.createElement("div");
    body.className = "opaque-function-body";
    fcnBox.appendChild(body);

    const innerLayout = { x: 20, y: 40 };
    const childOptions = { 
        ...options,
        stopPropagation: true, 
        constrainToParent: true,
        pathPrefix: `${prefix}.returns`
    };

    const returns = opaqueFcn.returns || {};
    for (const [key, value] of Object.entries(returns)) {
        renderComponent({ type: key, value: value, namespace: namespace }, body, innerLayout, childOptions);
    }
    container.appendChild(fcnBox);

    renderAutoResizableBody(fcnBox, "block", [".opaque-function-header"]);

    layoutCtx.x += 350;
}