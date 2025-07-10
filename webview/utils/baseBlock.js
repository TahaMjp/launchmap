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

import { makeDraggable } from "./drag.js";
import { getTypeLabel } from "./labels.js";

export function createBaseBlock({ type, label, layoutCtx, options }) {
    const block = document.createElement("div");
    block.className = `block ${type}-block`;
    block.style.position = "absolute";
    block.style.left = `${layoutCtx.x}px`;
    block.style.top = `${layoutCtx.y}px`;

    // Top label
    const heading = document.createElement("div");
    heading.className = "block-title";
    heading.innerText = getTypeLabel(type);
    block.appendChild(heading);

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

