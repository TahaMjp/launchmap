import { makeDraggable } from '../drag.js';
import { renderNodeGroup } from './renderNode.js';
import { renderIncludesGroup } from './renderInclude.js';

export function renderGroup(group, prefix, container, layoutCtx, options = {}) {
    const ns = group.namespace || prefix;
    const groupBox = document.createElement("div");
    groupBox.className = "group-box";
    groupBox.style.left = `${layoutCtx.x}px`;
    groupBox.style.top = `${layoutCtx.y}px`;
    groupBox.style.position = "absolute";

    const title = document.createElement("div");
    title.className = "group-title";
    title.innerText = `📦 Group: ${ns}`;
    groupBox.appendChild(title);

    groupBox.dataset.path = prefix;
    groupBox.dataset.type = "group";
    if (options.blockRegistry) {
        options.blockRegistry[prefix] = groupBox;
    }

    const innerLayout = { x: 20, y: 40 };
    const childOptions = { 
        ...options,
        stopPropagation: true, 
        constrainToParent: true,
        pathPrefix: `${prefix}.nodes`
    };

    renderNodeGroup(groupBox, group.nodes || [], ns, innerLayout, childOptions);
    renderIncludesGroup(groupBox, group.includes || [], ns, innerLayout, {
        ...childOptions,
        pathPrefix: `${prefix}.includes`
    });

    container.appendChild(groupBox);
    makeDraggable(groupBox, {
        ...childOptions,
        onDrag: () => {
            if (options.renderEdges && options.parsedData && options.argumentRegistry && options.blockRegistry) {
                options.renderEdges(options.parsedData, options.argumentRegistry, options.blockRegistry);
            }
        }
    });

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