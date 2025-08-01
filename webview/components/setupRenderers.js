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

import { registerRenderer } from "../core/dispatcher.js";
import { renderArguments } from "./renderArguments.js";
import { renderIncludesGroup } from "./renderInclude.js";
import { renderNodeGroup } from "./renderNode.js";
import { renderGroupGroup } from "./renderGroup.js";
import { renderOpaqueFunctionGroup } from "./renderOpaqueFunction.js";
import { renderComposableContainerGroup } from "./renderComposableContainer.js";
import { renderComposableNodeGroup } from "./renderComposableNode.js";
import { renderEventHandlerGroup } from "./renderEventHandler.js";
import { renderPythonExpressions } from "./renderPythonExpressions.js";
import { renderEnvironmentVariables } from "./renderEnvironmentVariables.js";
import { renderTimerActions } from "./renderTimerActions.js";

registerRenderer("arguments", (obj, container, options) => {
    renderArguments(container, obj.value || [], options);
});

registerRenderer("environment_variables", (obj, container, options) => {
    renderEnvironmentVariables(container, obj.value || [], options);
});

registerRenderer("python_expressions", (obj, container, options) => {
    renderPythonExpressions(container, obj.value || [], options);
});

registerRenderer("event_handlers", (obj, container, options) => {
    renderEventHandlerGroup(container, obj.value || [], options);
});

registerRenderer("nodes", (obj, container, options) => {
    renderNodeGroup(container, obj.value || [], options);
});

registerRenderer("includes", (obj, container, options) => {
    renderIncludesGroup(container, obj.value || [], options);
});

registerRenderer("groups", (obj, container, options) => {
    renderGroupGroup(container, obj.value || [], options);
});

registerRenderer("timer_actions", (obj, container, options) => {
    renderTimerActions(container, obj.value || [], options);
});

registerRenderer("opaque_functions", (obj, container, options) => {
    renderOpaqueFunctionGroup(container, obj.value || [], options);
});

registerRenderer("composable_nodes_container", (obj, container, options) => {
    renderComposableContainerGroup(container, obj.value || [], options);
});

registerRenderer("composable_nodes", (obj, container, options) => {
    renderComposableNodeGroup(container, obj.value || [], options);
});