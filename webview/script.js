import { renderArguments } from './render/renderArguments.js';
import { renderNodeGroup } from './render/renderNode.js';
import { renderIncludesGroup } from './render/renderInclude.js';
import { renderGroup } from './render/renderGroup.js';
import { renderEdges } from './render/renderEdges.js';

const argumentRegistry = {};
const blockRegistry = {};
const portRegistry = {};

window.addEventListener('message', (event) => {
    const message = event.data;
    if (message.type == 'launchmap-data') {
        renderAll(message.data);
    }
});

function renderAll(data) {
    const editor = document.getElementById("editor");
    editor.innerHTML = "";

    // Edges
    const edgeLayer = document.createElementNS("http://www.w3.org/2000/svg", "svg");
    edgeLayer.setAttribute("id", "edge-layer");
    edgeLayer.classList.add("edge-layer");
    editor.appendChild(edgeLayer);

    // Graph Nodes
    const layoutCtx = { x: 100, y: 100 };

    // Top Level nodes and includes
    renderArguments(editor, data.arguments || [], layoutCtx, 
        { argumentRegistry, blockRegistry, portRegistry, parsedData: data, renderEdges });
    renderNodeGroup(editor, data.nodes || [], "", layoutCtx, 
        { pathPrefix: "nodes", blockRegistry, argumentRegistry, portRegistry, parsedData: data, renderEdges});
    renderIncludesGroup(editor, data.includes || [], "", layoutCtx, 
        { pathPrefix: "includes", blockRegistry, argumentRegistry, portRegistry, parsedData: data, renderEdges });

    // Recursively render groups
    (data.groups || []).forEach((group, idx) => {
        renderGroup(group, `groups[${idx}]`, editor, layoutCtx, 
            { path: `groups[${idx}]`, blockRegistry, argumentRegistry, portRegistry, parsedData: data, renderEdges });
    });

    renderEdges(data, portRegistry);
}