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

export function enableZoomAndPan(editor, zoomLayer, renderEdgesCallback) {
  const zoomState = {
    scale: 1,
    offsetX: 0,
    offsetY: 0
  };
  window.zoomState = zoomState;

  let isPanning = false;
  let startX, startY;

  const ZOOM_MIN = 0.2;
  const ZOOM_MAX = 3;

  function applyTransform() {
    zoomLayer.style.transform = `translate(${zoomState.offsetX}px, ${zoomState.offsetY}px) scale(${zoomState.scale})`;
    if (renderEdgesCallback) renderEdgesCallback();
  }

  // Shift + Drag to pan
  editor.addEventListener('mousedown', (e) => {
    if (e.shiftKey) {
      isPanning = true;
      startX = e.clientX;
      startY = e.clientY;
      e.preventDefault();
    }
  });

  window.addEventListener('mousemove', (e) => {
    if (!isPanning) return;
    zoomState.offsetX += e.clientX - startX;
    zoomState.offsetY += e.clientY - startY;
    startX = e.clientX;
    startY = e.clientY;
    applyTransform();
  });

  window.addEventListener('mouseup', () => isPanning = false);

  // Scroll to zoom
  editor.addEventListener('wheel', (e) => {
    if (!e.ctrlKey && !e.shiftKey) {
      e.preventDefault();
      const zoomFactor = 1 - e.deltaY * 0.001;
      const newZoom = Math.min(ZOOM_MAX, Math.max(ZOOM_MIN, zoomState.scale * zoomFactor));

      const rect = zoomLayer.getBoundingClientRect();
      const dx = (e.clientX - rect.left - zoomState.offsetX) / zoomState.scale;
      const dy = (e.clientY - rect.top - zoomState.offsetY) / zoomState.scale;

      zoomState.offsetX -= dx * (newZoom - zoomState.scale);
      zoomState.offsetY -= dy * (newZoom - zoomState.scale);
      zoomState.scale = newZoom;
      applyTransform();
    }
  }, { passive: false });

  applyTransform();
}
