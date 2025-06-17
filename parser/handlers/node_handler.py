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

def handle_node(node: ast.Call, ctx: ParseContext) -> dict:
    if not isinstance(node, ast.Call):
        return None
    
    fields = ["package", "executable", "name", "namespace", "output"]
    data = {}

    for field in fields:
        value = get_kwarg(node, field)
        if value:
            field_ctx = ParseContext(visitor=ctx.visitor, field=field)
            data[field] = parse_value(value, field_ctx)
    
    # Parameters
    params = get_kwarg(node, "parameters")
    param_ctx = ParseContext(visitor=ctx.visitor, field="parameters")
    parsed_params = []

    if isinstance(params, ast.List):
        for elt in params.elts:
            parsed = parse_value(elt, param_ctx)
            parsed_params.append(parsed)

    elif isinstance(params, ast.Dict):
        parsed = parse_value(params, param_ctx)
        parsed_params.append(parsed)

    elif params is not None:
        parsed = parse_value(params, param_ctx)
        if isinstance(parsed, list):
            parsed_params.extend(parsed)
        else:
            parsed_params.append(parsed)
    
    if parsed_params:
        data["parameters"] = parsed_params

    # Remappings
    remaps = get_kwarg(node, "remappings")
    if remaps and isinstance(remaps, ast.List):
        pairs = []
        for elt in remaps.elts:
            if isinstance(elt, ast.Tuple) and len(elt.elts) == 2:
                remap_ctx = ParseContext(visitor=ctx.visitor, field="remappings")
                lhs = parse_value(elt.elts[0], remap_ctx)
                rhs = parse_value(elt.elts[1], remap_ctx)
                pairs.append([lhs, rhs])
        data["remappings"] = pairs
    
    # Arguments
    arguments = get_kwarg(node, "arguments")
    if arguments and isinstance(arguments, ast.List):
        arg_ctx = ParseContext(visitor=ctx.visitor, field="arguments")
        data["arguments"] = [parse_value(arg, arg_ctx) for arg in arguments.elts]
    
    return data
