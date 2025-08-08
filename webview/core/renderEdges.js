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

export function renderEdges(data, portRegistry) {
  document.getElementById('edge-layer').innerHTML = '';

  (data.launch_argument_usages || []).forEach(usage => {
    const fromPortId = `argument:${usage.argument}.argument`;
    const toPortId = `${usage.path}`;
    connectPorts(fromPortId, toPortId, portRegistry);
  });

  (data.environment_variable_usages || []).forEach(usage => {
    const fromPortId = `argument:${usage.argument}.argument`;
    const toPortId = `${usage.path}`;
    connectPorts(fromPortId, toPortId, portRegistry);
  });

  (data.python_expression_usages || []).forEach(usage => {
    const fromPortId = `variable:${usage.variable}.variable`;
    const toPortId = `${usage.path}`;
    connectPorts(fromPortId, toPortId, portRegistry);
  });

  (data.event_handlers || []).forEach((handler, index) => {
    const triggersPortId = `${handler.triggered_by[0]}`;
    const eventTriggeredPortId = `events[${index}].events.triggered_by`;
    connectPorts(triggersPortId, eventTriggeredPortId, portRegistry, 'event-handler');

    const eventTriggersPortId = `events[${index}].events.triggers`;
    const triggeredPortId = `${handler.triggers[0]}`;
    connectPorts(eventTriggersPortId, triggeredPortId, portRegistry, 'event-handler');
  });
}

function connectPorts(fromPortId, toPortId, portRegistry, className='') {
  const fromPort = portRegistry[fromPortId];
  const toPort = portRegistry[toPortId];

  if (!fromPort || !toPort) return;

  fromPort.classList.add('used-port');
  toPort.classList.add('used-port');

  const from = getCenter(fromPort);
  const to = getCenter(toPort);

  drawEdge(from, to, className);
}

function getCenter(el) {
  const zoomLayer = document.getElementById('zoom-layer');
  const zoomRect = zoomLayer.getBoundingClientRect();
  const rect = el.getBoundingClientRect();

  const zoom = window.zoomState?.scale || 1;

  return {
    x: (rect.left - zoomRect.left + rect.width / 2) / zoom,
    y: (rect.top - zoomRect.top + rect.height / 2) / zoom
  };
}

function drawEdge(from, to, className) {
  const svg = document.getElementById('edge-layer');
  const path = document.createElementNS('http://www.w3.org/2000/svg', 'path');

  const dx = Math.abs(to.x - from.x);
  const controlOffset = Math.min(100, dx / 2);

  const d = `M ${from.x},${from.y} C ${from.x + controlOffset},${from.y} ${to.x - controlOffset},${to.y} ${to.x},${to.y}`;

  path.setAttribute('d', d);
  path.setAttribute('class', `edge-line ${className}`.trim());

  svg.appendChild(path);
}
