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

export function renderEventPortRow(path, portRegistry,
  leftLabelText = '← triggered by', rightLabelText = 'triggers →') {
  const eventPortRow = document.createElement('div');
  eventPortRow.className = 'event-port-row';

  // Left port (triggered_by)
  const leftWrapper = document.createElement('div');
  leftWrapper.className = 'event-port-wrapper left';

  const leftLabel = document.createElement('span');
  leftLabel.className = 'event-label';
  leftLabel.innerText = leftLabelText;

  const leftPort = document.createElement('div');
  leftPort.className = 'port left';
  leftWrapper.appendChild(leftPort);
  leftWrapper.appendChild(leftLabel);

  eventPortRow.appendChild(leftWrapper);

  if (portRegistry) portRegistry[`${path}.events.triggered_by`] = leftPort;

  // Right port (triggers)
  const rightWrapper = document.createElement('div');
  rightWrapper.className = 'event-port-wrapper right';

  const rightLabel = document.createElement('span');
  rightLabel.className = 'event-label';
  rightLabel.innerText = rightLabelText;

  const rightPort = document.createElement('div');
  rightPort.className = 'port right';
  rightWrapper.appendChild(rightLabel);
  rightWrapper.appendChild(rightPort);

  eventPortRow.appendChild(rightWrapper);

  if (portRegistry) portRegistry[`${path}.events.triggers`] = rightPort;

  return eventPortRow;
}
