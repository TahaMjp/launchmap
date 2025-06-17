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
from parser.utils import get_kwarg, parse_value, parse_dict

def handle_include(node: ast.Call, ctx: ParseContext) -> dict:
    include_data = {}

    # Extract source argument: PythonLaunchDescriptionSource(...)
    for arg in node.args:
        if isinstance(arg, ast.Call) and getattr(arg.func, 'id', None) == "PythonLaunchDescriptionSource":
            source_arg = arg.args[0] if arg.args else None
            if source_arg:
                path_ctx = ParseContext(visitor=ctx.visitor, field="path")
                include_data["path"] = parse_value(source_arg, path_ctx)
            else:
                include_data["path"] = "<unresolved>"
    
    # Handle launch arguments={...}.items()
    for kw in node.keywords:
        if kw.arg == "launch_arguments" and isinstance(kw.value, ast.Call):
            call = kw.value
            if isinstance(call.func, ast.Attribute) and call.func.attr == "items":
                dict_node = call.func.value
                if isinstance(dict_node, ast.Dict):
                    args_ctx = ParseContext(visitor=ctx.visitor, field="launch_arguments")
                    include_data["launch_arguments"] = parse_dict(dict_node, args_ctx)
    
    return include_data if "path" in include_data else None