# Copyright (c) 2025 Kodo Robotics
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import ast
from parser.context import ParseContext
from parser.parser.registry import register_handler
from parser.parser.utils import flatten_once
from parser.resolution.utils import resolve_call_signature

@register_handler("GroupAction", "launch.actions.GroupAction")
def handle_group_action(node: ast.Call, context: ParseContext) -> dict:
    """
    Handle GroupAction(actions=[...])
    - Resolves all actions recursively
    - Pushes/pops namespace scope if PushRosNamespace is found
    """
    args, kwargs = resolve_call_signature(node, context.engine)

    raw_actions = kwargs.get("actions", args[0] if args else [])
    parsed_actions = []

    temp_ns = None
    for raw in raw_actions:
        resolved = raw
        if isinstance(resolved, dict) and resolved.get("type") == "PushRosNamespace":
            temp_ns = resolved.get("namespace")
            context.push_namespace(temp_ns)
            continue

        if isinstance(resolved, list):
            parsed_actions.extend(flatten_once(resolved))
        else:
            parsed_actions.append(resolved)
    
    if temp_ns:
        context.pop_namespace()

    return {
        "type": "GroupAction",
        "namespace": temp_ns,
        "actions": parsed_actions
    }