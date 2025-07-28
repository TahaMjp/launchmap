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
import { renderSection } from './renderSection.js';

export function renderTimerActions(container, timerActions, namespace, layoutCtx, options = {}) {
    timerActions.forEach((group, idx) => {
        const path = options.pathPrefix ? `${options.pathPrefix}.timer_actions[${idx}]` : `timer_actions[${idx}]`;
        renderTimerAction(group, container, layoutCtx, { ...options, path });
    });

    layoutCtx.x += 350;
    layoutCtx.y = 100;
}

export function renderTimerAction(timerAction, container, layoutCtx, options = {}) {
    const timerActionBox = renderBaseBlock({
        type: "timer-action",
        layoutCtx,
        options
    });

    // Header
    const header = document.createElement("div");
    header.className = "timer-action-header";

    // Render additional sections
    const metaSections = [
        { key: "period", icon: "🧭", label: "Period", value: timerAction.period },
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

    timerActionBox.append(header);

    // Body
    const body = document.createElement("div");
    body.className = "timer-action-body";
    timerActionBox.appendChild(body);

    const innerLayout = { x: 20, y: 40 };
    const childOptions = { 
        ...options,
        stopPropagation: true, 
        constrainToParent: true,
        pathPrefix: `${options.path}.actions`
    };

    const actions = timerAction.actions || {};
    for (const [key, value] of Object.entries(actions)) {
        renderComponent({ type: key, value: value }, body, innerLayout, childOptions);
    }
    container.appendChild(timerActionBox);

    renderAutoResizableBody(timerActionBox, "block", [".timer-action-header"]);

    layoutCtx.x += 350;
}