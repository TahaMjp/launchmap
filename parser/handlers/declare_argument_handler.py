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
from parser.utils import get_kwarg, parse_value

def handle_declare_argument(node: ast.Call, ctx: ParseContext) -> dict:
    if not isinstance(node, ast.Call):
        return None
    
    data = {}

    # positional `name`
    if node.args:
        data["name"] = parse_value(node.args[0], ctx)
    
    # keyword `name`
    name_kwarg = get_kwarg(node, "name")
    if name_kwarg:
        data["name"] = parse_value(name_kwarg, ctx)

    # default_value
    default = get_kwarg(node, "default_value")
    if default:
        data["default"] = parse_value(default, ctx)
    
    # description
    desc = get_kwarg(node, "description")
    if desc:
        data["description"] = parse_value(desc, ctx)

    if ctx:
        ctx.visitor.declared_arguments.add(data["name"])
    
    return data if "name" in data else None