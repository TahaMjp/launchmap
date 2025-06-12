import { renderArguments } from './render/renderArguments.js';
import { renderNodeGroup } from './render/renderNode.js';
import { renderIncludesGroup } from './render/renderInclude.js';
import { renderGroup } from './render/renderGroup.js';

window.addEventListener('message', (event) => {
    const message = event.data;
    if (message.type == 'launchmap-data') {
        renderAll(message.data);
    }
});

function renderAll(data) {
    const editor = document.getElementById("editor");
    editor.innerHTML = "";

    const layoutCtx = { x: 100, y: 100 };

    // Top Level nodes and includes
    renderArguments(editor, data.arguments || [], layoutCtx);
    renderNodeGroup(editor, data.nodes || [], "", layoutCtx);
    renderIncludesGroup(editor, data.includes || [], "", layoutCtx);

    // Recursively render groups
    (data.groups || []).forEach((group, idx) => {
        renderGroup(group, idx, editor, layoutCtx);
    });
}