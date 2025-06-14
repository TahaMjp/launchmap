export function renderEdges(data, portRegistry) {
    const svg = document.getElementById("edge-layer");
    svg.innerHTML = "";

    (data.launch_argument_usages || []).forEach(usage => {
        const fromPortId = `argument:${usage.argument}:argument`;
        const toPortId = `${usage.path}:${usage.field}`

        const fromPort = portRegistry[fromPortId];
        const toPort = portRegistry[toPortId];

        if (!fromPort || !toPort) return;

        fromPort.classList.add("used-port");
        toPort.classList.add("used-port");

        const from = getCenter(fromPort);
        const to = getCenter(toPort);

        drawEdge(svg, from, to, usage.field);
    });
}

function getCenter(el) {
    const rect = el.getBoundingClientRect();
    const parentSvg = document.getElementById("edge-layer");
    const svgRect = parentSvg.getBoundingClientRect();

    return {
        x: rect.left - svgRect.left + rect.width / 2 ,
        y: rect.top - svgRect.top + rect.height / 2
    };
}

function drawEdge(svg, from, to, field) {
    const path = document.createElementNS("http://www.w3.org/2000/svg", "path");

    const dx = Math.abs(to.x - from.x);
    const controlOffset = Math.min(100, dx / 2);

    const d = `M ${from.x},${from.y} C ${from.x + controlOffset},${from.y} ${to.x - controlOffset},${to.y} ${to.x},${to.y}`;

    path.setAttribute("d", d);
    path.setAttribute("class", "edge-line");
    path.setAttribute("stroke", getEdgeColor(field));

    svg.appendChild(path);
}

function getEdgeColor(field) {
    switch (field) {
        case "parameters": return "#6aff6a";
        case "launch_arguments": return "#ffa500";
        case "namespace": return "#66ccff";
        default: return "#ccc";
    }
}