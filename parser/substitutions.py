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