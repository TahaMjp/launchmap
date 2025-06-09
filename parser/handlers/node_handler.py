"""
Node Handler for ROS 2 Launch File AST Parser

This module extracts and parses the 'Node' action from a ROS 2 launch file,
handling common fields and launch-time configurations.

✅ Supported Node Fields (Static Parsing):
- package (str)
- executable (str)
- name (str, optional)
- namespace (str, optional)
- output (e.g., "screen", optional)
- parameters:
    - List of inline dictionaries
    - YAML file paths as strings
    - LaunchConfiguration with/without default
    - Direct dictionary (not in a list)
- remappings:
    - List of 2-element tuples

⚠️ Partially Supported / Fallback as "<unresolved>":
- Parameters set via variable reference
- LaunchConfiguration combined with other substitutions
- Fields assigned from variables
- Mixed types or nested launch expressions

🛑 Not Supported by Static Parsing:
- Node constructed conditionally (e.g., in if/else or loops)
- Node returned from factory/helper functions
- Parameters returned from functions (e.g., load_params())
- Any dynamic logic requiring runtime evaluation

For unsupported or ambiguous cases, the handler inserts a fallback value
(e.g., "<unresolved>" or symbolic LaunchConfiguration placeholder)
to keep output consistent and informative.

"""

import ast
from parser.utils import get_kwarg, parse_value, parse_dict

def handle_node(node: ast.Call) -> dict:
    if not isinstance(node, ast.Call):
        return None
    
    fields = ["package", "executable", "name", "namespace", "output"]
    data = {}

    for field in fields:
        value = get_kwarg(node, field)
        if value:
            data[field] = parse_value(value)
    
    # Parameters
    params = get_kwarg(node, "parameters")
    parsed_params = []

    if isinstance(params, ast.List):
        for elt in params.elts:
            if isinstance(elt, ast.Constant):
                parsed_params.append(elt.value)
            elif isinstance(elt, ast.Dict):
                parsed_params.append(parse_dict(elt))
            else:
                parsed_params.append("<unresolved>")
    elif isinstance(params, ast.Dict):
        parsed_params.append(parse_dict(params))
    elif isinstance(params, ast.Name):
        parsed_params = "<unresolved>"
    
    if parsed_params:
        data["parameters"] = parsed_params

    # Remappings
    remaps = get_kwarg(node, "remappings")
    if remaps and isinstance(remaps, ast.List):
        pairs = []
        for elt in remaps.elts:
            if isinstance(elt, ast.Tuple) and len(elt.elts) == 2:
                pairs.append([parse_value(elt.elts[0]), parse_value(elt.elts[1])])
        data["remappings"] = pairs
    
    return data
