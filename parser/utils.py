import ast

def get_kwarg(node, name):
    for kw in node.keywords:
        if kw.arg == name:
            return kw.value
    return None

def parse_dict(dict_node):
    parsed = {}
    for k, v in zip(dict_node.keys, dict_node.values):
        parsed[parse_value(k)] = parse_value(v)
    return parsed

def parse_value(val):
    from parser.substitutions import parse_substitution

    if isinstance(val, ast.Constant):
        if isinstance(val.value, str) and val.value.lower() in ("true", "false"):
            return val.value.lower() == "true"
        return val.value
    elif isinstance(val, ast.Name):
        return "<unresolved>"
    elif isinstance(val, ast.Call):
        return parse_substitution(val)

    return "<unresolved>"