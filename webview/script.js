window.addEventListener('message', (event) => {
    const message = event.data;
    if (message.type == 'launchmap-data') {
        renderNodes(message.data);
    }
});

function renderNodes(data) {
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

function renderNodeGroup(container, nodes, namespace, layoutCtx) {
    nodes.forEach((node, idx) => {
        const block = renderNode(node, namespace, layoutCtx);
        container.appendChild(block);
        layoutCtx.y += 100;
    });

    layoutCtx.x += 250;
    layoutCtx.y = 100;
}

function renderNode(node, namespace, layoutCtx) {
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

function renderIncludesGroup(container, includes, namespace, layoutCtx) {
    includes.forEach((include, idx) => {
        const block = renderInclude(include, namespace, layoutCtx);
        container.appendChild(block);
        layoutCtx.y += 100;
    });

    layoutCtx.x += 250;
    layoutCtx.y = 100;
}

function renderInclude(include, namespace, layoutCtx) {
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

    return block;
}

function renderArguments(container, argumentsList, layoutCtx) {
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

    layoutCtx.y += 120;
}

function renderGroup(group, index, container, layoutCtx) {
    const ns = group.namespace || `group_${index}`;
    const groupBox = document.createElement("div");
    groupBox.className = "group-box";
    groupBox.style.left = `${layoutCtx.x}px`;
    groupBox.style.top = `${layoutCtx.y}px`;

    const title = document.createElement("div");
    title.className = "group-title";
    title.innerText = `📦 Group: ${ns}`;
    groupBox.appendChild(title);

    const innerLayout = { x: 20, y: 40 };

    renderNodeGroup(groupBox, group.nodes || [], ns, innerLayout);
    renderIncludesGroup(groupBox, group.inclues || [], ns, innerLayout);

    groupBox.style.width = `300px`;
    groupBox.style.height = `${Math.max(innerLayout.y + 50, 150)}px`;

    container.appendChild(groupBox);

    layoutCtx.x += 350;
}