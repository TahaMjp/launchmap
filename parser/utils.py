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
        if isinstance(val.func, ast.Name) and val.func.id in {
            "LaunchConfiguration", "EnvironmentVariable", "PathJoinSubstitution"
        }:
            return parse_substitution(val)
        else:
            return parse_python_expression(val)

    return "<unresolved>"

def parse_python_expression(val):
    # Handle os.path.join
    if isinstance(val, ast.Call):
        if isinstance(val.func, ast.Attribute) and val.func.attr == "join":
            path_attr = val.func.value
            if isinstance(path_attr, ast.Attribute) and path_attr.attr == "path":
                parts = []
                for arg in val.args:
                    part = parse_value(arg)
                    parts.append(part if isinstance(part, str) else "<unresolved>")
                return "/".join(parts)
            
        # Handle os.path.join(package_share_directory(...), ...)
        if isinstance(val.func, ast.Attribute) and val.func.attr == "join":
            first_arg = val.args[0]
            if (
                isinstance(first_arg, ast.Call)
                and isinstance(first_arg.func, ast.Name)
                and first_arg.func.id == "get_package_share_directory"
            ):
                pkg_name = parse_value(first_arg.args[0])
                rest = [parse_value(arg) for arg in val.args[1:]]
                return f"<pkg:{pkg_name}>/" + "/".join(rest)
        
        # Direct get_package_share_directory(...)
        if isinstance(val.func, ast.Name) and val.func.id == "get_package_share_directory":
            pkg_name = parse_value(val.args[0]) if val.args else "<unknown>"
            return f"<pkg:{pkg_name}>"
    
    return "<unresolved>"