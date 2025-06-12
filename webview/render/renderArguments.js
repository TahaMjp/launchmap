import { makeDraggable } from '../drag.js';

export function renderArguments(container, argumentsList, layoutCtx) {
    if (!argumentsList || argumentsList.length == 0) return;

    const block = document.createElement("div");
    block.className = "arguments-block";

    block.innerHTML = `<div class="node-title">🚀 Launch Arguments</div><ul>`;
    argumentsList.forEach(arg => {
        block.innerHTML += `<li><code>${arg.name}</code> = <code>${arg.default}</code></li>`;
    });
    block.innerHTML += `</ul>`;

    block.style.left = `${layoutCtx.x}px`;
    block.style.top = `${layoutCtx.y}px`;
    block.style.position = "absolute";

    container.appendChild(block);
    makeDraggable(block, { stopPropagation: true });

    layoutCtx.y += 120;
}