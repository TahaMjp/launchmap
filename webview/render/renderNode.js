import { makeDraggable } from '../drag.js';
import { renderSection } from './renderSection.js';

export function renderNodeGroup(container, nodes, namespace, layoutCtx, options={}) {
    nodes.forEach((node, idx) => {
        const path = options.pathPrefix ? `${options.pathPrefix}[${idx}]` : "";
        const block = renderNode(node, namespace, layoutCtx, { ...options, path });
        container.appendChild(block);
        layoutCtx.y += 100;
    });

    layoutCtx.x += 250;
    layoutCtx.y = 100;
}

function renderNode(node, namespace, layoutCtx, options) {
    const block = document.createElement("div");
    block.className = "node-block";
    block.style.left = `${layoutCtx.x}px`;
    block.style.top = `${layoutCtx.y}px`;
    block.style.position = "absolute";

    // Title bar
    const title = document.createElement("div");
    title.className = "node-title";
    title.innerText = `${namespace}/${node.name || node.executable}`;
    block.appendChild(title);

    // Fields
    const renderOptions = { includeLeftPort: true, portIdPrefix: options.path, portRegistry: options.portRegistry };
    block.appendChild(renderSection("package", "📦", "Package", `<code>${node.package}</code>`, renderOptions));
    block.appendChild(renderSection("executable", "▶️", "Executable", `<code>${node.executable}</code>`, renderOptions));
    block.appendChild(renderSection("output", "🖥️", "Output", node.output || "—", renderOptions));

    if (node.parameters?.length > 0) {
        let paramHtml = "<ul>";
        for (const p of node.parameters) {
            for (const [k, v] of Object.entries(p)) {
                paramHtml += `<li><code>${k}</code>: <code>${v}</code></li>`;
            }
        }
        paramHtml += "</ul>";
        block.appendChild(renderSection("parameters", "⚙️", "Params", paramHtml, renderOptions));
    }
    

    if (options.path) {
        block.dataset.path = options.path;
        block.dataset.type = "node";
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