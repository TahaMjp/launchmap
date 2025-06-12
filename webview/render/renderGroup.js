import { makeDraggable } from '../drag.js';
import { renderNodeGroup } from './renderNode.js';
import { renderIncludesGroup } from './renderInclude.js';

export function renderGroup(group, index, container, layoutCtx) {
    const ns = group.namespace || `group_${index}`;
    const groupBox = document.createElement("div");
    groupBox.className = "group-box";
    groupBox.style.left = `${layoutCtx.x}px`;
    groupBox.style.top = `${layoutCtx.y}px`;
    groupBox.style.position = "absolute";

    const title = document.createElement("div");
    title.className = "group-title";
    title.innerText = `📦 Group: ${ns}`;
    groupBox.appendChild(title);

    const innerLayout = { x: 20, y: 40 };
    const options = { stopPropagation: true, constrainToParent: true};
    renderNodeGroup(groupBox, group.nodes || [], ns, innerLayout, options);
    renderIncludesGroup(groupBox, group.includes || [], ns, innerLayout, options);
    container.appendChild(groupBox);
    makeDraggable(groupBox);

    requestAnimationFrame(() => {
        const children = groupBox.querySelectorAll(".node-block, .include-block");
        let maxRight = 0;
        let maxBottom = 0;

        children.forEach(child => {
            const rect = child.getBoundingClientRect();
            const parentRect = groupBox.getBoundingClientRect();
            const right = rect.right - parentRect.left;
            const bottom = rect.bottom - parentRect.top;

            if (right > maxRight) maxRight = right;
            if (bottom > maxBottom) maxBottom = bottom;
        });

        groupBox.style.width = `${maxRight + 20}px`;
        groupBox.style.height = `${maxBottom + 20}px`;
    });

    layoutCtx.x += 350;
}