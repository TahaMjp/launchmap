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
from parser.utils import get_kwarg

def parse_substitution(node: ast.Call) -> str:
    if not isinstance(node, ast.Call) or not isinstance(node.func, ast.Name):
        return "<unresolved>"

    func = node.func.id

    if func == "LaunchConfiguration":
        name = node.args[0].value if node.args else "<unnamed>"
        default = None
        for kw in node.keywords:
            if kw.arg == "name":
                name = _resolve_const(kw.value)
            elif kw.arg == "default":
                default = _resolve_const(kw.value)
        return f"${{LaunchConfiguration:{name}:default={default}}}" if default is not None else f"${{LaunchConfiguration:{name}}}"

    elif func == "EnvironmentVariable":
        name = node.args[0].value if node.args else "<unknown>"
        return f"${{EnvironmentVariable:{name}}}"

    elif func == "PathJoinSubstitution":
        joined = []
        for elt in node.args[0].elts:
            if isinstance(elt, ast.Call) and elt.func.id == "TextSubstitution":
                text = get_kwarg(elt, "text")
                joined.append(_resolve_const(text) if text else "<unresolved>")
        return f"${{PathJoinSubstitution:{'/'.join(joined)}}}"
    
    return f"<unhandled:{func}>"

def _resolve_const(val):
    if isinstance(val, ast.Constant):
        if isinstance(val.value, str) and val.value.lower() in ("true", "false"):
            return val.value.lower() == "true"
        return val.value
    return "<unresolved>"