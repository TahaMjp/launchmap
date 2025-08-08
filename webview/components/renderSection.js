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

export function renderSection(fieldName, icon, label, contentValue, {
  includeLeftPort = false,
  includeRightPort = false,
  portIdPrefix = '',
  portRegistry = null
}) {
  const wrapper = document.createElement('div');
  wrapper.className = 'section';
  wrapper.dataset.field = fieldName;

  // Port ID
  const fullPath = portIdPrefix
    ? (fieldName ? `${portIdPrefix}.${fieldName}` : portIdPrefix)
    : fieldName;

  const content = document.createElement('div');
  content.className = 'content';

  // Try primitive
  if (typeof contentValue !== 'object' || contentValue === null) {
    if (contentValue === '') {
      content.innerHTML = `${icon} ${label}`;
    } else {
      content.innerHTML = `${icon} ${label}: <code>${escapeHtml(String(contentValue))}</code>`;
    }
    // Left Port
    if (includeLeftPort) {
      const leftPort = document.createElement('div');
      leftPort.className = 'port left';
      wrapper.appendChild(leftPort);

      if (portRegistry) portRegistry[fullPath] = leftPort;
    }

    wrapper.appendChild(content);

    // Right Port
    if (includeRightPort) {
      const rightPort = document.createElement('div');
      rightPort.className = 'port right';
      wrapper.appendChild(rightPort);

      if (portRegistry) portRegistry[fullPath] = rightPort;
    }

    return wrapper;
  }

  // Complex data type
  content.innerHTML = `${icon} ${label}`;
  wrapper.classList.add('has-nested');
  wrapper.appendChild(content);
  const nested = renderNestedValue(contentValue, fullPath, {
    includeLeftPort, includeRightPort, portRegistry
  });
  const nestedWrapper = document.createElement('div');
  nestedWrapper.className = 'section-nested';
  nestedWrapper.appendChild(nested);
  wrapper.appendChild(nestedWrapper);

  return wrapper;
}

function renderNestedValue(value, basePath, options) {
  const list = document.createElement('ul');
  list.style.margin = '4px 0';
  list.style.paddingLeft = '16px';

  if (Array.isArray(value)) {
    value.forEach((item, idx) => {
      const itemPath = `${basePath}[${idx}]`;
      const li = document.createElement('li');
      li.appendChild(renderSection('', '', '', item, {
        ...options,
        portIdPrefix: itemPath
      }));
      list.appendChild(li);
    });
  } else {
    Object.entries(value).forEach(([key, val]) => {
      const li = document.createElement('li');
      li.appendChild(renderSection(key, '', key, val, {
        ...options,
        portIdPrefix: basePath
      }));
      list.appendChild(li);
    });
  }

  return list;
}

function escapeHtml(text) {
  return text
    .replace(/&/g, '&amp;')
    .replace(/</g, '&lt;')
    .replace(/>/g, '&gt;')
    .replace(/"/g, '&quot;');
}
