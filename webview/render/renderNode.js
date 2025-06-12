import { makeDraggable } from '../drag.js';

export function renderNodeGroup(container, nodes, namespace, layoutCtx, options={}) {
    nodes.forEach((node, idx) => {
        const block = renderNode(node, namespace, layoutCtx, options);
        container.appendChild(block);
        layoutCtx.y += 100;
    });

    layoutCtx.x += 250;
    layoutCtx.y = 100;
}

function renderNode(node, namespace, layoutCtx, options) {
    const block = document.createElement("div");
    block.className = "node-block";

    block.innerHTML = `
    <div class="node-title">${namespace} /${node.name || node.executable}</div>
    <div>📦 <code>${node.package}</code></div>
    <div>▶️ <code>${node.executable}</code></div>
    <div>🖥️ Output: ${node.output || "—"}</div>
    ${renderParameters(node.parameters)}
  `;

    block.style.left = `${layoutCtx.x}px`;
    block.style.top = `${layoutCtx.y}px`;
    block.style.position = "absolute";

    makeDraggable(block, options);
    return block;
}

function renderParameters(params) {
    if (!params || params.length === 0) return "";

    let html = "<div>⚙️ Params:</div><ul>";
    for (const p of params) {
        for (const p of params) {
            for (const [k, v] of Object.entries(p)) {
                html += `<li><code>${k}</code>: <code>${v}</code></li>`;
            }
        }
    }
    html += "</ul>";
    return html;
}