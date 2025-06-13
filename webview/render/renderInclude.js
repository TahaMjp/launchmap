import { makeDraggable } from '../drag.js';

export function renderIncludesGroup(container, includes, namespace, layoutCtx, options={}) {
    includes.forEach((include, idx) => {
        const path = options.pathPrefix ? `${options.pathPrefix}[${idx}]` : "";
        const block = renderInclude(include, namespace, layoutCtx, { ...options, path });
        container.appendChild(block);
        layoutCtx.y += 100;
    });

    layoutCtx.x += 250;
    layoutCtx.y = 100;
}

function renderInclude(include, namespace, layoutCtx, options) {
    const block = document.createElement("div");
    block.className = "include-block";

    const path = include.path || "<unresolved>";
    const args = include.launch_arguments || {};

    let argsHTML = "";
    const entries = Object.entries(args);
    if (entries.length > 0) {
        argsHTML = "<div>📥 Args:</div><ul>";
        for (const [k, v] of entries) {
            argsHTML += `<li><code>${k}</code>: <code>${v}</code></li>`;
        }
        argsHTML += "</ul>";
    }

    block.innerHTML = `
    <div class="node-title">${namespace} / Include</div>
    <div>📂 Path: <code>${path}</code></div>
    ${argsHTML}
  `;

    block.style.left = `${layoutCtx.x}px`;
    block.style.top = `${layoutCtx.y}px`;
    block.style.position = "absolute";

    if (options.path) {
        block.dataset.path = options.path;
        block.dataset.type = "include";
        if (options.blockRegistry) {
            options.blockRegistry[options.path] = block;
        }
    }

    makeDraggable(block, {
        ...options,
        onDrag: () => {
            if (options.renderEdges && options.parsedData && options.argumentRegistry && options.blockRegistry) {
                options.renderEdges(options.parsedData, options.argumentRegistry, options.blockRegistry);
            }
        }
    });
    return block;
}