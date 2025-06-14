"""
GroupAction Handler for ROS 2 Launch File AST Parser

This module extracts and parses the 'GroupAction' structure from a ROS 2 launch file,
allowing for namespace scoping and hierarchical grouping of other actions like Node,
IncludeLaunchDescription, and nested groups.

✅ Supported GroupAction Features (Static Parsing):
- PushRosNamespace('robot_ns') → parsed as 'namespace'
- Child Node(...) actions
- Child IncludeLaunchDescription(...) actions
- Nested GroupAction(...) blocks
- Empty GroupAction blocks
- Multiple PushRosNamespace calls (last one is used)

⚠️ Partially Supported / Fallback as "<unresolved>":
- PushRosNamespace set from variable or complex expression
- Unsupported inner actions (e.g., LogInfo, SetEnvironmentVariable) are ignored
- Children passed as variables or constructed dynamically (e.g., `children = [...]`)
- Mixed content with unknown or unresolved structure

🛑 Not Supported by Static Parsing:
- GroupAction used conditionally (if/else)
- Children generated via loops or helper functions
- Runtime behavior like OpaqueFunction, OnProcessExit
- Actions requiring evaluated substitutions or lambdas

For unsupported or ambiguous cases, the handler inserts fallback values
or skips unresolved children, maintaining consistency for visualization
and further downstream processing.
"""

import ast
from parser.context import ParseContext
from parser.utils import parse_value, get_kwarg
from parser.handlers.node_handler import handle_node
from parser.handlers.include_handler import handle_include

def handle_group_action(node: ast.Call, ctx: ParseContext) -> dict:
    if not isinstance(node, ast.Call) or node.func.id != "GroupAction":
        return None
    
    group_data = {}
    namespace = None
    children = node.args[0].elts if node.args and isinstance(node.args[0], ast.List) else []

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