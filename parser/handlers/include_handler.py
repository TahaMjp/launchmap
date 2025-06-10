"""
IncludeLaunchDescription Handler for ROS 2 Launch File AST Parser

This module extracts and parses the 'IncludeLaunchDescription' action from a ROS 2
launch file. It statically analyzes how other launch files are included and how
arguments are passed to them, without executing any code or following includes recursively.

✅ Supported Include Patterns (Static Parsing):
- PythonLaunchDescriptionSource with:
    - Direct string path
    - os.path.join(...) with constant strings
    - get_package_share_directory(...) → <pkg:package_name>
    - Combination of get_package_share_directory + os.path.join
    - LaunchConfiguration(...) → ${LaunchConfiguration:name}
    - EnvironmentVariable(...) → ${EnvironmentVariable:VAR}
- launch_arguments passed via:
    - Inline dict with `.items()`

⚠️ Partially Supported / Fallback as "<unresolved>":
- launch_arguments passed via variable
- Include path set via a variable
- Nested or complex substitutions
- Arguments or paths returned from functions
- Includes inside GroupAction or other structures (only parsed if statically reachable)

🛑 Not Supported by Static Parsing:
- Conditional includes (e.g., inside if/else branches)
- Includes inside loops or lambdas
- Includes returned from helper/factory functions
- Deep recursion or evaluation of included files
- Any dynamic logic requiring runtime or execution context

For unsupported or ambiguous cases, the handler inserts fallback values
such as "<unresolved>" or symbolic placeholders to maintain safe,
consistent, and meaningful output for visualization and further inspection.
"""

import ast
from parser.utils import get_kwarg, parse_value, parse_dict

def handle_include(node: ast.Call) -> dict:
    include_data = {}

    # Extract source argument: PythonLaunchDescriptionSource(...)
    for arg in node.args:
        if isinstance(arg, ast.Call) and getattr(arg.func, 'id', None) == "PythonLaunchDescriptionSource":
            source_arg = arg.args[0] if arg.args else None
            include_data["path"] = parse_value(source_arg) if source_arg else "<unresolved>"
    
    # Handle launch arguments={...}.items()
    for kw in node.keywords:
        if kw.arg == "launch_arguments" and isinstance(kw.value, ast.Call):
            call = kw.value
            if isinstance(call.func, ast.Attribute) and call.func.attr == "items":
                dict_node = call.func.value
                if isinstance(dict_node, ast.Dict):
                    include_data["launch_arguments"] = parse_dict(dict_node)
    
    return include_data if "path" in include_data else None