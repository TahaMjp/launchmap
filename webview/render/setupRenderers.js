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

import { registerRenderer } from "../renderRegistry.js";
import { renderArguments } from "./renderArguments.js";
import { renderIncludesGroup } from "./renderInclude.js";
import { renderNodeGroup } from "./renderNode.js";
import { renderGroupGroup } from "./renderGroup.js";

registerRenderer("arguments", (obj, container, layoutCtx, options) => {
    renderArguments(container, obj.value || [], layoutCtx, options);
});

registerRenderer("nodes", (obj, container, layoutCtx, options) => {
    renderNodeGroup(container, obj.value || [], obj.namespace || "", layoutCtx, options);
});

registerRenderer("includes", (obj, container, layoutCtx, options) => {
    renderIncludesGroup(container, obj.value || [], obj.namespace || "", layoutCtx, options);
});

registerRenderer("groups", (obj, container, layoutCtx, options) => {
    renderGroupGroup(container, obj.value || [], obj.namespace || "", layoutCtx, options);
});