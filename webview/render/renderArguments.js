import { makeDraggable } from '../drag.js';

export function renderArguments(container, argumentsList, layoutCtx, options) {
    if (!argumentsList || argumentsList.length == 0) return;

    argumentsList.forEach((arg, idx) => {
        const block = document.createElement("div");
        block.className = "argument-block";

        block.innerHTML = `
            <div class="arg-title">🚀 ${arg.name}</div>
            <div>Default: <code>${arg.default}</code></div>
        `;

        block.style.left = `${layoutCtx.x}px`;
        block.style.top = `${layoutCtx.y + idx * 80}px`;
        block.style.position = "absolute";

        block.dataset.argument = arg.name;
        options.argumentRegistry[arg.name] = block;

        container.appendChild(block);
        makeDraggable(block, {
            stopPropagation: true,
            ...options,
            onDrag: () => {
                if (options.renderEdges && options.parsedData && options.argumentRegistry && options.blockRegistry) {
                    options.renderEdges(options.parsedData, options.argumentRegistry, options.blockRegistry);
                }
            }
        });
    });

    layoutCtx.x += 250;
}