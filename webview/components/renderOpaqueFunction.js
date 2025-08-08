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
import { LayoutManager } from '../core/layoutManager.js';

export function renderOpaqueFunctionGroup(container, opaqueFcns, options = {}) {
  opaqueFcns.forEach((opaqueFcn, idx) => {
    const path = options.pathPrefix ? `${options.pathPrefix}.opaque_functions[${idx}]` : `opaque_functions[${idx}]`;
    const block = renderOpaqueFunction(opaqueFcn, { ...options, path });

    container.appendChild(block);
    options.renderBlock(block, 'opaque-function');
  });
}

export function renderOpaqueFunction(opaqueFcn, options = {}) {
  const fcnBox = renderBaseBlock({
    type: 'opaque-function',
    options: {
      ...options,
      events: opaqueFcn.events
    }
  });

  // Header
  const header = document.createElement('div');
  header.className = 'opaque-function-header';

  // Title
  const title = document.createElement('div');
  title.className = 'opaque-function-title';
  title.innerText = `${opaqueFcn.name}`;
  header.appendChild(title);

  fcnBox.append(header);

  // Body
  const body = document.createElement('div');
  body.className = 'opaque-function-body';
  fcnBox.appendChild(body);

  const innerLayoutManager = new LayoutManager(20, 40, 80, 40);
  const childOptions = {
    ...options,
    stopPropagation: true,
    constrainToParent: true,
    pathPrefix: `${options.path}.returns`,
    renderBlock: (block, columnType) => {
      requestAnimationFrame(() => {
        innerLayoutManager.placeBlock(block, columnType);
      });
    }
  };

  const returns = opaqueFcn.returns || {};
  for (const [key, value] of Object.entries(returns)) {
    renderComponent({ type: key, value: value }, body, childOptions);
  }

  renderAutoResizableBody(fcnBox, 'block', ['.opaque-function-header']);

  return fcnBox;
}
