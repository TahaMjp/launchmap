export function renderEdges(data, argumentRegistry, blockRegistry) {
    const svg = document.getElementById("edge-layer");
    svg.innerHTML = "";

    (data.launch_argument_usages || []).forEach(usage => {
        const fromBlock = argumentRegistry[usage.argument];
        const toBlock = blockRegistry[usage.path];

        if (!fromBlock || !toBlock) return;

        const from = getAnchorPoint(fromBlock, "argument");
        const to = getAnchorPoint(toBlock, usage.field);

        drawEdge(svg, from, to, usage.field);
    });
}

function getAnchorPoint(block, role) {
    const rect = block.getBoundingClientRect();
    const scrollX = window.scrollX;
    const scrollY = window.scrollY;

    const anchor = {
        x: rect.left + scrollX,
        y: rect.top + scrollY
    };

    switch (role) {
        case "argument":
            anchor.x += rect.width;
            anchor.y += rect.height / 2;
            break;
        case "parameters":
            anchor.x += 0;
            anchor.y += rect.height * 0.6;
            break;
        case "namespace":
            anchor.x += 0;
            anchor.y += rect.height * 0.3;
            break;
        case "launch_arguments":
            anchor.x += 0;
            anchor.y += rect.height * 0.45;
            break;
        default:
            anchor.x += 0;
            anchor.y += rect.height / 2;
    }

    return anchor;
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