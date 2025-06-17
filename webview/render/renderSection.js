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

export function renderSection(fieldName, icon, label, contentHtml, {
    includeLeftPort = false,
    includeRightPort = false,
    portIdPrefix = "",
    portRegistry = null
}) {
    const wrapper = document.createElement("div");
    wrapper.className = "section";
    wrapper.dataset.field = fieldName;

    if (includeLeftPort) {
        const leftPort = document.createElement("div");
        leftPort.className = "port left";
        wrapper.appendChild(leftPort);

        const portId = `${portIdPrefix}:${fieldName}`;
        if (portRegistry) portRegistry[portId] = leftPort;
    }

    const content = document.createElement("div");
    content.className = "content";
    content.innerHTML = `${icon} ${label}: ${contentHtml}`;
    wrapper.appendChild(content);

    if (includeRightPort) {
        const rightPort = document.createElement("div");
        rightPort.className = "port right";
        wrapper.appendChild(rightPort);

        const portId = `${portIdPrefix}:${fieldName}`;
        if (portRegistry) portRegistry[portId] = rightPort;
    }

    return wrapper;
}