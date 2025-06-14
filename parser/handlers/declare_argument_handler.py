"""
DeclareLaunchArgument Handler for ROS 2 Launch File AST Parser

This module statically parses the 'DeclareLaunchArgument' action from a ROS 2
launch file, extracting metadata about launch-time arguments such as their
names, default values, and descriptions.

✅ Supported Fields (Static Parsing):
- name (str): 
    - Supports both positional and keyword usage
- default_value:
    - Constant values (e.g., strings, numbers, booleans)
    - Boolean strings ('true', 'false') → parsed as Python bools
    - LaunchConfiguration substitution (with or without default)
    - EnvironmentVariable substitution
    - PathJoinSubstitution (with inline TextSubstitution)
- description:
    - Constant string descriptions

⚠️ Partially Supported / Fallback as "<unresolved>":
- default_value assigned from a variable or function
- LaunchConfiguration with a variable name
- Substitutions with nested or computed expressions
- TextSubstitution or PathJoinSubstitution with dynamic arguments

🛑 Not Supported by Static Parsing:
- Arguments declared conditionally (e.g., inside if/else or loop)
- Arguments defined via helper/wrapper functions
- Arguments generated dynamically or via unpacking (*args, **kwargs)
- Substitutions requiring runtime context (e.g., Command, FindExecutable)

For unsupported or ambiguous expressions, the parser assigns safe fallback
placeholders such as "<unresolved>" or symbolic substitution strings
(e.g., "${LaunchConfiguration:foo}") to maintain consistency and clarity in
the resulting representation.
"""

import ast
from parser.context import ParseContext
from parser.utils import get_kwarg, parse_value

def handle_declare_argument(node: ast.Call, ctx: ParseContext) -> dict:
    if not isinstance(node, ast.Call):
        return None
    
    data = {}

    # positional `name`
    if node.args:
        data["name"] = parse_value(node.args[0])
    
    # keyword `name`
    name_kwarg = get_kwarg(node, "name")
    if name_kwarg:
        data["name"] = parse_value(name_kwarg)

    # default_value
    default = get_kwarg(node, "default_value")
    if default:
        data["default"] = parse_value(default)
    
    # description
    desc = get_kwarg(node, "description")
    if desc:
        data["description"] = parse_value(desc)

    if ctx:
        ctx.visitor.declared_arguments.add(data["name"])
    
    return data if "name" in data else None