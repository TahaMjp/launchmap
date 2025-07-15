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

import { makeDraggable } from "../utils/drag.js";
import { getTypeLabel } from "../utils/labels.js";
import { renderEventPortRow } from "./renderEventPortRow.js";

export function renderBaseBlock({ type, label, layoutCtx, options }) {
    const block = document.createElement("div");
    block.className = `block ${type}-block`;
    block.style.position = "absolute";
    block.style.left = `${layoutCtx.x}px`;
    block.style.top = `${layoutCtx.y}px`;

    // Header container
    const header = document.createElement("div");
    header.className = "block-header";

    // Top label
    const heading = document.createElement("div");
    heading.className = "block-title";
    heading.innerText = getTypeLabel(type);
    header.appendChild(heading);

    // Event ports
    const path = options?.path;
    const portRegistry = options?.portRegistry;
    if (options?.events?.triggered_by?.length || options?.events?.triggers?.length) {
        const leftLabel = options.eventLabels?.left || "← triggered by";
        const rightLabel = options.eventLabels?.right || "triggers →";

        const eventPortRow = renderEventPortRow(path, portRegistry, leftLabel, rightLabel);
        header.appendChild(eventPortRow);
    }

    block.appendChild(header);

    // Metadata
    if (options?.path) {
        block.dataset.path = options.path;
        block.dataset.type = type;
    }

    // Draggable
    makeDraggable(block, {
        ...options,
        onDrag: () => {
            if (options.renderEdges && options.parsedData) {
                options.renderEdges(options.parsedData, options.portRegistry);
            }
        }
    });

    return block;
}

