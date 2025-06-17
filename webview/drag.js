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

export function makeDraggable(element, options={}) {
    let offsetX = 0, offsetY = 0, startX = 0, startY = 0;

    // Ensure element has initial position
    if (!element.style.left) element.style.left = "0px";
    if (!element.style.top) element.style.top = "0px";

    element.onmousedown = dragMouseDown;

    function dragMouseDown(e) {
        if (options.stopPropagation) e.stopPropagation();
        e.preventDefault();
        startX = e.clientX;
        startY = e.clientY;

        document.onmousemove = elementDrag;
        document.onmouseup = stopDrag;
    }

    function elementDrag(e) {
        e.preventDefault();

        const zoomScale = window.zoomState?.scale || 1;

        offsetX = (e.clientX - startX) / zoomScale;
        offsetY = (e.clientY - startY) / zoomScale;
        startX = e.clientX;
        startY = e.clientY;

        const currentLeft = parseFloat(element.style.left || "0");
        const currentTop = parseFloat(element.style.top || "0");

        let newLeft = currentLeft + offsetX;
        let newTop = currentTop + offsetY;

        if (options.constrainToParent) {
            const parent = element.parentElement;
            const parentWidth = parent.clientWidth;
            const parentHeight = parent.clientHeight;
            const elementWidth = element.offsetWidth;
            const elementHeight = element.offsetHeight;

            const maxLeft = parentWidth - elementWidth;
            const maxTop = parentHeight - elementHeight;

            newLeft = Math.max(0, Math.min(newLeft, maxLeft));
            newTop = Math.max(0, Math.min(newTop, maxTop));
        }

        element.style.left = `${newLeft}px`;
        element.style.top = `${newTop}px`;

        if (typeof options.onDrag === "function") {
            requestAnimationFrame(() => options.onDrag());
        }
    }

    function stopDrag() {
        document.onmouseup = null;
        document.onmousemove = null;
    }
}