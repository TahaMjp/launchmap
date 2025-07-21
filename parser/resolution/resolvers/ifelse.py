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
from parser.resolution.utils import collect_assigned_variable_names
from parser.resolution.resolution_registry import register_resolver

@register_resolver(ast.If)
def resolve_if_expression(node: ast.If, engine):
    """
    Symbolically track `if`/`elif`/`else` expressions and assigned variables.
    """
    context = engine.context

    # Extract code as string
    code_str = ast.unparse(node)
    
    # Collect all variables assigned in any branch
    assigned_vars = set()

    def collect_from_block(block):
        for stmt in block:
            assigned_vars.update(collect_assigned_variable_names(stmt))

    collect_from_block(node.body)
    orelse = node.orelse
    while orelse:
        if len(orelse) == 1 and isinstance(orelse[0], ast.If):
            collect_from_block(orelse[0].body)
            orelse = orelse[0].orelse
        else:
            collect_from_block(orelse)
            break
    
    # Track variables
    symbolic_vars = [f"${{var:{v}}}" for v in sorted(assigned_vars)]
    context.introspection.track_python_expression(code_str, symbolic_vars)

    return None