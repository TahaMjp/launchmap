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

export function renderArguments(container, argumentsList, options) {
  if (!argumentsList || argumentsList.length === 0) return;

  argumentsList.forEach((arg, idx) => {
    const path = `${options.pathPrefix || 'arguments'}[${idx}]`;
    const block = renderArgument(arg, { ...options, path });

    container.appendChild(block);
    options.renderBlock(block, 'argument');
  });
}

export function renderArgument(arg, options) {
  const block = renderBaseBlock({ type: 'argument', options });

  const value = arg.default_value !== undefined ? arg.default_value : '';
  const argSection = renderSection('argument', '🚀', arg.name, value,
    { includeRightPort: true, portIdPrefix: `argument:${arg.name}`, portRegistry: options.portRegistry });
  block.appendChild(argSection);

  block.dataset.argument = arg.name;

  return block;
}
