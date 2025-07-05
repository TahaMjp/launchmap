// Copyright (c) 2025 Kodo Robotics
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

import { getRenderer } from "../core/dispatcher.js";

export function renderComponent(obj, container, layoutCtx, options = {}) {
    if (!obj || typeof obj !== "object") return;

    const type = obj.type || detectType(obj);
    const renderer = getRenderer(type);

    if (!renderer) {
        console.warn(`No renderer for type: ${type}`, obj);
        return;
    }

    renderer(obj, container, layoutCtx, {
        ...options,
        renderComponent
    });
}

function detectType(obj) {
    if (obj.nodes || obj.groups || obj.composable_nodes) return "group";
    if (Array.isArray(obj.arguments)) return "arguments";
    if (Array.isArray(obj.includes)) return "includes";
    if (Array.isArray(obj.nodes)) return "nodes";
    if (Array.isArray(obj.opaque_functions)) return "opaque_functions";
    return null;
}