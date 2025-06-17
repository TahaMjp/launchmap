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

export function renderArguments(container, argumentsList, layoutCtx, options) {
    if (!argumentsList || argumentsList.length == 0) return;

    argumentsList.forEach((arg, idx) => {
        const block = document.createElement("div");
        block.className = "argument-block";
        block.style.left = `${layoutCtx.x}px`;
        block.style.top = `${layoutCtx.y + idx * 80}px`;
        block.style.position = "absolute";

        const argSection = renderSection("argument", "🚀", arg.name, `<code>${arg.default}</code>`, 
            { includeRightPort: true, portIdPrefix: `argument:${arg.name}`, portRegistry: options.portRegistry });
        block.appendChild(argSection);

        block.dataset.argument = arg.name;
        options.argumentRegistry[arg.name] = block;

        container.appendChild(block);
        makeDraggable(block, {
            stopPropagation: true,
            ...options,
            onDrag: () => {
                if (options.renderEdges && options.parsedData && options.argumentRegistry && options.blockRegistry) {
                    options.renderEdges(options.parsedData, options.portRegistry);
                }
            }
        });
    });

    layoutCtx.x += 250;
}