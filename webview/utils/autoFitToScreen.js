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

export function autoFitToScreen(editor, zoomLayer, margin = 40) {
    if (!window.zoomState) return;
    const zoomState = window.zoomState;

    const prevTransform = zoomLayer.style.transform;
    zoomLayer.style.transform = "none";

    const children = zoomLayer.querySelectorAll(".block");
    if (!children.length) {
        zoomLayer.style.transform = prevTransform;
        return;
    }

    let minX = Infinity, minY = Infinity, maxX = -Infinity, maxY = -Infinity;
    children.forEach(child => {
        const rect = child.getBoundingClientRect();
        const editorRect = editor.getBoundingClientRect();

        const relativeLeft = rect.left - editorRect.left;
        const relativeTop = rect.top - editorRect.top;

        minX = Math.min(minX, relativeLeft);
        minY = Math.min(minY, relativeTop);
        maxX = Math.max(maxX, relativeLeft + rect.width);
        maxY = Math.max(maxY, relativeTop + rect.height);
    });

    const contentWidth = maxX - minX;
    const contentHeight = maxY - minY;

    const editorRect = editor.getBoundingClientRect();
    const availableWidth = editorRect.width - margin * 2;
    const availableHeight = editorRect.height - margin * 2;

    let scale = Math.min(availableWidth / contentWidth, availableHeight / contentHeight);
    scale = Math.max(0.2, Math.min(3, scale));

    zoomState.scale = scale;
    zoomState.offsetX = -minX * scale + (availableWidth - contentWidth * scale) / 2 + margin;
    zoomState.offsetY = -minY * scale + (availableHeight - contentHeight * scale) / 2 + margin;

    zoomLayer.style.transform = prevTransform;
    const applyTransform = () => {
        zoomLayer.style.transform = `translate(${zoomState.offsetX}px, ${zoomState.offsetY}px) scale(${zoomState.scale})`;
    }
    applyTransform();
}