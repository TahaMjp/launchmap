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

import { renderSection } from './renderSection.js';
import { renderBaseBlock } from './renderBaseBlock.js';

export function renderEventHandlerGroup(container, handlers, options={}) {
    handlers.forEach((handler, idx) => {
        const path = `events[${idx}]`;
        const block = renderEventHandler(handler, { ...options, path });

        container.appendChild(block);
        options.renderBlock(block, "event-handler");
    });
}

function renderEventHandler(handler, options) {
    const block = renderBaseBlock({
        type: "event-handler",
        options: {
            ...options,
            events: {
                triggered_by: handler.triggered_by,
                triggers: handler.triggers
            },
            eventLabels: handler.type === "OnProcessExit"
                ? { left: "← target_action", right: "on_exit →" }
                : handler.type === "OnProcessStart"
                    ? { left: "← target_action", right: "on_start →" }
                    : undefined
        }
    });

    // Main Section
    const type = handler.type || 'EventHandler';
    block.appendChild(renderSection("type", "📣", "Type", type, {
        includeLeftPort: false,
        includeRightPort: false
    }));

    return block;
}