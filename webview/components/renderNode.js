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
import { createBaseBlock } from '../utils/baseBlock.js';

export function renderNodeGroup(container, nodes, namespace, layoutCtx, options={}) {
    nodes.forEach((node, idx) => {
        const path = options.pathPrefix ? `${options.pathPrefix}.nodes[${idx}]` : `nodes[${idx}]`;
        const block = renderNode(node, namespace, layoutCtx, { ...options, path });
        container.appendChild(block);
        layoutCtx.y += 100;
    });

    layoutCtx.x += 250;
    layoutCtx.y = 100;
}

function renderNode(node, namespace, layoutCtx, options) {
    const block = createBaseBlock({
        type: 'node',
        layoutCtx,
        options
    })

    // Node name
    const titleLabel = node.name || node.executable || "(anonymous)";
    const fullName = namespace ? `${namespace}/${titleLabel}` : titleLabel;

    // Sections
    const renderOptions = { includeLeftPort: true, portIdPrefix: options.path, portRegistry: options.portRegistry };
    block.appendChild(renderSection("name", "📛", "Name", fullName, renderOptions));
    block.appendChild(renderSection("package", "📦", "Package", node.package, renderOptions));
    block.appendChild(renderSection("executable", "▶️", "Executable", node.executable, renderOptions));
    block.appendChild(renderSection("output", "🖥️", "Output", node.output || "—", renderOptions));

    if (node.condition) {
        block.appendChild(renderSection("condition", "❓", "Condition", node.condition, renderOptions));
    }

    if (node.parameters?.length > 0) {
        block.appendChild(renderSection("parameters", "⚙️", "Params", node.parameters, renderOptions));
    }

    if (node.arguments?.length > 0) {
        block.appendChild(renderSection("arguments", "💬", "Args", node.arguments, renderOptions));
    }

    return block;
}