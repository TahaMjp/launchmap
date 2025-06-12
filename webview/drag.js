export function makeDraggable(element, options={}) {
    let offsetX = 0, offsetY = 0, startX = 0, startY = 0;

    // Ensure element has initial position
    if (!element.style.left) element.style.left = "0px";
    if (!element.style.top) element.style.top = "0px";

    element.onmousedown = dragMouseDown;

    function dragMouseDown(e) {
        if (options.stopPropagation) e.stopPropagation();
        e.preventDefault();
        startX = e.clientX;
        startY = e.clientY;

        document.onmousemove = elementDrag;
        document.onmouseup = stopDrag;
    }

    function elementDrag(e) {
        e.preventDefault();
        offsetX = e.clientX - startX;
        offsetY = e.clientY - startY;
        startX = e.clientX;
        startY = e.clientY;

        const currentLeft = parseFloat(element.style.left || "0");
        const currentTop = parseFloat(element.style.top || "0");

        let newLeft = currentLeft + offsetX;
        let newTop = currentTop + offsetY;

        if (options.constrainToParent) {
            const parent = element.parentElement;
            const parentWidth = parent.clientWidth;
            const parentHeight = parent.clientHeight;
            const elementWidth = element.offsetWidth;
            const elementHeight = element.offsetHeight;

            const maxLeft = parentWidth - elementWidth;
            const maxTop = parentHeight - elementHeight;

            newLeft = Math.max(0, Math.min(newLeft, maxLeft));
            newTop = Math.max(0, Math.min(newTop, maxTop));
        }

        element.style.left = `${newLeft}px`;
        element.style.top = `${newTop}px`;
    }

    function stopDrag() {
        document.onmouseup = null;
        document.onmousemove = null;
    }
}