"""
Substitution Parser for ROS 2 Launch File AST Parser

This module extracts and formats launch-time substitution expressions used in
ROS 2 launch files, such as LaunchConfiguration, EnvironmentVariable, and
PathJoinSubstitution. It is designed to work safely using static AST parsing,
without executing any Python code.

✅ Supported Substitutions (Static Parsing):
- LaunchConfiguration('name')
- LaunchConfiguration('name', default='value')
- EnvironmentVariable('VAR_NAME')
- PathJoinSubstitution([...TextSubstitution(text=...)])
- Boolean string literals ("true", "false") → parsed as bool

⚠️ Partially Supported / Fallback as "<unresolved>":
- Substitutions passed as variables
- LaunchConfiguration(default=EnvironmentVariable(...)) — no nested resolution
- PathJoinSubstitution with non-constant arguments
- Any substitution containing unknown or dynamic input

🛑 Not Supported by Static Parsing:
- Command([...]) or FindExecutable(...) — require subprocess/runtime environment
- Custom/unimported substitution types
- Nested or computed substitutions
- Default values derived from functions or variables

For unsupported or ambiguous substitutions, the parser returns symbolic
placeholders (e.g., "<unresolved>", "<unhandled:SubType>") to preserve structure
and enable graceful degradation in the visual output.

"""

import ast
from parser.context import ParseContext
from parser.utils import get_kwarg

def parse_substitution(node: ast.Call, ctx: ParseContext = None) -> str:
    if not isinstance(node, ast.Call) or not isinstance(node.func, ast.Name):
        return "<unresolved>"

    func = node.func.id

    if func == "LaunchConfiguration":
        return _parse_launch_configuration(node, ctx)
    
    elif func == "EnvironmentVariable":
        return _parse_environment_variable(node)
    
    elif func == "PathJoinSubstitution":
        return _parse_path_join(node)
    
    elif func == "TextSubstitution":
        return _parse_text_substitution(node)
    
    return f"<unhandled:{func}>"

def _parse_launch_configuration(node: ast.Call, ctx: ParseContext = None) -> str:
    name = None
    default = None

    if node.args:
        name = _resolve_const(node.args[0])
    
    for kw in node.keywords:
        if kw.arg == "name":
            name = _resolve_const(kw.value)
        elif kw.arg == "default":
            default = _resolve_const(kw.value)
    
    if name is None:
        return "<unresolved>"
    
    if ctx:
        ctx.track_launch_arg_usage(name)
        ctx.visitor.used_arguments.append(name)
    
    return f"${{LaunchConfiguration:{name}:default={default}}}" if default is not None else f"${{LaunchConfiguration:{name}}}"

def _parse_environment_variable(node: ast.Call) -> str:
    name = _resolve_const(node.args[0]) if node.args else "<unknown>"
    return f"${{EnvironmentVariable:{name}}}"

def _parse_path_join(node: ast.Call) -> str:
    if not node.args or not isinstance(node.args[0], ast.List):
        return "<unresolved>"
    
    parts = []
    for elt in node.args[0].elts:
        part = parse_literal_or_substitution(elt)
        parts.append(str(part))
    
    return f"${{PathJoinSubstitution:{'/'.join(parts)}}}"

def _parse_text_substitution(node: ast.Call) -> str:
    text = get_kwarg(node, "text")
    return _resolve_const(text) if text else "<unresolved>"

def _resolve_const(val):
    if isinstance(val, ast.Constant):
        if isinstance(val.value, str) and val.value.lower() in ("true", "false"):
            return val.value.lower() == "true"
        return val.value
    return "<unresolved>"

def parse_literal_or_substitution(node):
    if isinstance(node, ast.Constant):
        return _resolve_const(node)
    elif isinstance(node, ast.Call):
        return parse_substitution(node)
    elif isinstance(node, ast.Name):
        return "<unresolved>"
    return "<unresolved>"