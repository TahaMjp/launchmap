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
import { renderSection } from './renderSection.js';

export function renderIncludesGroup(container, includes, options={}) {
    includes.forEach((include, idx) => {
        const path = options.pathPrefix ? `${options.pathPrefix}.includes[${idx}]` : `includes[${idx}]`;
        const block = renderInclude(include, { ...options, path });

        container.appendChild(block);
        options.renderBlock(block, "include");
    });
}

function renderInclude(include, options) {
    const block = renderBaseBlock({
        type: 'include',
        options: {
            ...options,
            events: include.events
        }
    });

    // Render
    const renderOptions = { includeLeftPort: true, portIdPrefix: options.path, portRegistry: options.portRegistry };
    const path = include.launch_description_source || "<unresolved>";
    block.appendChild(renderSection("launch_description_source", "📂", "Path", path, renderOptions));
    
    const args = include.launch_arguments || {};
    block.appendChild(renderSection("launch_arguments", "📥", "Args", args, renderOptions))

    if (include.condition) {
        block.appendChild(renderSection("condition", "❓", "Condition", include.condition, renderOptions));
    }

    return block;
}