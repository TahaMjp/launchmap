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

def get_kwarg(node, name):
    for kw in node.keywords:
        if kw.arg == name:
            return kw.value
    return None

def parse_dict(dict_node, ctx: ParseContext = None):
    parsed = {}
    for k, v in zip(dict_node.keys, dict_node.values):
        parsed[parse_value(k, ctx)] = parse_value(v, ctx)
    return parsed

def parse_value(val, ctx: ParseContext = None):
    from parser.substitutions import parse_substitution

    if isinstance(val, ast.Constant):
        if isinstance(val.value, str) and val.value.lower() in ("true", "false"):
            return val.value.lower() == "true"
        return val.value
    
    elif isinstance(val, ast.Name):
        if ctx is not None:
            resolved = ctx.visitor.assignments.get(val.id)
            if resolved:
                return parse_value(resolved, ctx)
        return f"${{{val.id}}}"

    elif isinstance(val, ast.List):
        return [parse_value(elt, ctx) for elt in val.elts]
    
    elif isinstance(val, ast.Dict):
        return parse_dict(val, ctx)
    
    elif isinstance(val, ast.Call):
        if isinstance(val.func, ast.Name) and val.func.id in {
            "LaunchConfiguration", "EnvironmentVariable", "PathJoinSubstitution"
        }:
            return parse_substitution(val, ctx)
        
        else:
            return parse_python_expression(val, ctx)

    return "<unresolved>"

def parse_python_expression(val, ctx: ParseContext = None):
    if not isinstance(val, ast.Call):
        return "<unresolved>"
    
    func = val.func

    # Handle get_package_share_directory
    if isinstance(func, ast.Name) and func.id == "get_package_share_directory":
        if val.args:
            pkg_name = parse_value(val.args[0], ctx)
            return f"<pkg:{pkg_name}>"
        return "<pkg:?>"

    # Handle os.path.join
    if isinstance(func, ast.Attribute) and func.attr == "join":
        is_os_path_join = (
            isinstance(func.value, ast.Attribute)
            and getattr(func.value.value, "id", None) == "os"
            and func.value.attr == "path"
        )
        if is_os_path_join:
            parts = [parse_value(arg, ctx) for arg in val.args]
            parts_str = [p if isinstance(p, str) else "<unresolved>" for p in parts]

            # first arg is get_package_share_directory
            if (
                isinstance(val.args[0], ast.Call)
                and isinstance(val.args[0].func, ast.Name)
                and val.args[0].func.id == "get_package_share_directory"
            ):
                pkg_part = parse_value(val.args[0], ctx)
                rest = parts_str[1:]
                return f"{pkg_part}/{'/'.join(rest)}"
            
            return "/".join(parts_str)
        
    # Fallback
    func_name = (
        func.id if isinstance(func, ast.Name)
        else func.attr if isinstance(func, ast.Attribute)
        else "unknown"
    )
    arg_values = [parse_value(arg, ctx) for arg in val.args]
    return f"${{Call:{func_name}({', '.join(map(str, arg_values))})}}"