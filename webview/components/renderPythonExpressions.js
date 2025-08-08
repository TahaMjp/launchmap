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

export function renderPythonExpressions(container, expressionsList, options) {
  if (!expressionsList || expressionsList.length === 0) return;

  expressionsList.forEach((expr, idx) => {
    const path = `python_expressions[${idx}]`;
    const block = renderPythonExpression(expr, { ...options, path });

    container.appendChild(block);
    options.renderBlock(block, 'python-expression');
  });
}

export function renderPythonExpression(expr, options) {
  const block = renderBaseBlock({
    type: 'python-expression',
    options
  });

  // Code section
  const codeSection = document.createElement('pre');
  codeSection.className = 'python-code';
  codeSection.innerText = expr.body;
  block.appendChild(codeSection);

  // Variables section (right port active)
  if (expr.variables?.length > 0) {
    expr.variables.forEach((v) => {
      const varName = v.replace('${var:', '').replace('}', '');
      const varSection = renderSection('variable', '📤', varName, '', {
        includeRightPort: true,
        portIdPrefix: `variable:${varName}`,
        portRegistry: options.portRegistry
      });
      block.appendChild(varSection);
    });
  }

  return block;
}
