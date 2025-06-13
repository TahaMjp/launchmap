import { makeDraggable } from '../drag.js';
import { renderSection } from './renderSection.js';

export function renderArguments(container, argumentsList, layoutCtx, options) {
    if (!argumentsList || argumentsList.length == 0) return;

    argumentsList.forEach((arg, idx) => {
        const block = document.createElement("div");
        block.className = "argument-block";
        block.style.left = `${layoutCtx.x}px`;
        block.style.top = `${layoutCtx.y + idx * 80}px`;
        block.style.position = "absolute";

        const argSection = renderSection("argument", "🚀", arg.name, `<code>${arg.default}</code>`, 
            { includeRightPort: true, portIdPrefix: `argument:${arg.name}`, portRegistry: options.portRegistry });
        block.appendChild(argSection);

        block.dataset.argument = arg.name;
        options.argumentRegistry[arg.name] = block;

        container.appendChild(block);
        makeDraggable(block, {
            stopPropagation: true,
            ...options,
            onDrag: () => {
                if (options.renderEdges && options.parsedData && options.argumentRegistry && options.blockRegistry) {
                    options.renderEdges(options.parsedData, options.portRegistry);
                }
            }
        });
    });

    layoutCtx.x += 250;
}