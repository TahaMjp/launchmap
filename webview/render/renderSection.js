export function renderSection(fieldName, icon, label, contentHtml, {
    includeLeftPort = false,
    includeRightPort = false,
    portIdPrefix = "",
    portRegistry = null
}) {
    const wrapper = document.createElement("div");
    wrapper.className = "section";
    wrapper.dataset.field = fieldName;

    if (includeLeftPort) {
        const leftPort = document.createElement("div");
        leftPort.className = "port left";
        wrapper.appendChild(leftPort);

        const portId = `${portIdPrefix}:${fieldName}`;
        if (portRegistry) portRegistry[portId] = leftPort;
    }

    const content = document.createElement("div");
    content.className = "content";
    content.innerHTML = `${icon} ${label}: ${contentHtml}`;
    wrapper.appendChild(content);

    if (includeRightPort) {
        const rightPort = document.createElement("div");
        rightPort.className = "port right";
        wrapper.appendChild(rightPort);

        const portId = `${portIdPrefix}:${fieldName}`;
        if (portRegistry) portRegistry[portId] = rightPort;
    }

    return wrapper;
}