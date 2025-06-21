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
from parser.utils import parse_value, resolve_starred_list

def handle_group_action(node: ast.Call, ctx: ParseContext) -> dict:
    if not isinstance(node, ast.Call) or node.func.id != "GroupAction":
        return None
    
    group_data = {}
    namespace = None
    children = []

    if node.args and isinstance(node.args[0], ast.List):
        children = resolve_starred_list(node.args[0].elts, ctx.visitor)
    else:
        for kw in node.keywords:
            if kw.arg == "actions" and isinstance(kw.value, ast.List):
                children = kw.value.elts

    # PushRosNamespace
    for child in children:
        if isinstance(child, ast.Call) and getattr(child.func, 'id', None) == "PushRosNamespace":
            ns_arg = child.args[0] if child.args else None
            if ns_arg:
                ns_ctx = ParseContext(visitor=ctx.visitor, field="namespace")
                namespace = parse_value(ns_arg, ns_ctx)
            else:
                namespace = "<unresolved>"
    
    if namespace:
        group_data["namespace"] = namespace
    
    # Delegate all children back to visitor
    for child in children:
        actual_node = child
        if isinstance(child, ast.Name) and child.id in ctx.visitor.assignments:
            actual_node = ctx.visitor.assignments[child.id]

        if isinstance(actual_node, ast.Call):
            ctx.visitor._handle_action(actual_node, into=group_data)

    return group_data if group_data else None