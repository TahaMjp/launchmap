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

export function renderAutoResizableBody(container, type = "block", fixedSelectors = []) {
    requestAnimationFrame(() => {
        let offsetY = 0;

        if (fixedSelectors.length > 0) {
            fixedSelectors.forEach(sel => {
                const header = container.querySelector(sel);
                if (header) {
                    const height = header.getBoundingClientRect().height;
                    offsetY = Math.max(offsetY, height + 45);
                }
            });
        }

        const blockSelector = `[class$="-${type}"]`
        const allSelectors = [blockSelector, ...fixedSelectors].join(", ");
        const children = container.querySelectorAll(allSelectors);
        
        let maxRight = 0;
        let maxBottom = 0;

        children.forEach(child => {
            const rect = child.getBoundingClientRect();
            const parentRect = container.getBoundingClientRect();
            const right = rect.right - parentRect.left;
            const bottom = rect.bottom - parentRect.top;

            if (right > maxRight) maxRight = right;
            if (bottom > maxBottom) maxBottom = bottom;
        });

        container.style.width = `${maxRight + 20}px`;
        container.style.height = `${maxBottom + 20}px`;

        const body = container.querySelector(`[class$="-body"]`);
        if (body) {
            body.style.top = `${offsetY}px`;
        }
    });
}