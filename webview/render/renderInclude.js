import { makeDraggable } from '../drag.js';
import { renderSection } from './renderSection.js';

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
    block.style.left = `${layoutCtx.x}px`;
    block.style.top = `${layoutCtx.y}px`;
    block.style.position = "absolute";

    // Title
    const title = document.createElement("div");
    title.className = "node-title";
    title.innerText = `${namespace}/Include`;
    block.appendChild(title);

    // Path
    const renderOptions = { includeLeftPort: true, portIdPrefix: options.path, portRegistry: options.portRegistry };
    const path = include.path || "<unresolved>";
    block.appendChild(renderSection("path", "📂", "Path", `<code>${path}</code>`, renderOptions));
    
    // Launch arguments
    const args = include.launch_arguments || {};
    if (Object.keys(args).length > 0) {
        let argsHTML = "<ul>";
        for (const [k, v] of Object.entries(args)) {
            argsHTML += `<li><code>${k}</code>: <code>${v}</code></li>`;
        }
        argsHTML += "</ul>";
        block.appendChild(renderSection("launch_arguments", "📥", "Args", argsHTML, renderOptions));
    }

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
                options.renderEdges(options.parsedData, options.portRegistry);
            }
        }
    });
    return block;
}