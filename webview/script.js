import { renderArguments } from './render/renderArguments.js';
import { renderNodeGroup } from './render/renderNode.js';
import { renderIncludesGroup } from './render/renderInclude.js';
import { renderGroup } from './render/renderGroup.js';
import { renderEdges } from './render/renderEdges.js';
import { enableZoomAndPan } from './zoomPanController.js';

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

    // Create zoom layer
    const zoomLayer = document.createElement("div");
    zoomLayer.id = "zoom-layer";
    editor.appendChild(zoomLayer);

    // Edges
    const edgeLayer = document.createElementNS("http://www.w3.org/2000/svg", "svg");
    edgeLayer.setAttribute("id", "edge-layer");
    edgeLayer.classList.add("edge-layer");
    zoomLayer.appendChild(edgeLayer);

    // Graph Nodes
    const layoutCtx = { x: 100, y: 100 };

    // Top Level nodes and includes
    renderArguments(zoomLayer, data.arguments || [], layoutCtx, 
        { argumentRegistry, blockRegistry, portRegistry, parsedData: data, renderEdges });
    renderNodeGroup(zoomLayer, data.nodes || [], "", layoutCtx, 
        { pathPrefix: "nodes", blockRegistry, argumentRegistry, portRegistry, parsedData: data, renderEdges});
    renderIncludesGroup(zoomLayer, data.includes || [], "", layoutCtx, 
        { pathPrefix: "includes", blockRegistry, argumentRegistry, portRegistry, parsedData: data, renderEdges });

    // Recursively render groups
    (data.groups || []).forEach((group, idx) => {
        renderGroup(group, `groups[${idx}]`, zoomLayer, layoutCtx, 
            { path: `groups[${idx}]`, blockRegistry, argumentRegistry, portRegistry, parsedData: data, renderEdges });
    });

    renderEdges(data, portRegistry);
    enableZoomAndPan(editor, zoomLayer, () => renderEdges(data, portRegistry));
}