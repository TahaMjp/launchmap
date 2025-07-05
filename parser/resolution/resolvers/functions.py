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

from parser.resolution.resolution_engine import ResolutionEngine
from parser.resolution.resolution_registry import register_resolver
import copy
import ast

from parser.resolution.utils import bind_function_args

@register_resolver(ast.Call, priority=5)
def resolve_user_function(node: ast.Call, engine):
    # Resolve function name
    func_name = engine.context.get_func_name(node.func)

    # Check for user defined functions
    if engine.context.has_function(func_name):
        fn_def = engine.context.lookup_function(func_name)
    else:
        return None
        
    # Bind parameters
    resolved_args = [engine.resolve(arg) for arg in node.args]
    resolved_kwargs = {kw.arg: engine.resolve(kw.value) for kw in node.keywords}
    arg_map = bind_function_args(fn_def, resolved_args, resolved_kwargs)

    # Clone context to avoid polluting global scope
    sub_context = engine.context
    for name, value in arg_map.items():
        sub_context.define_variable(name, value)

    # Local engine
    sub_engine = ResolutionEngine(sub_context)
    sub_context.engine = sub_engine

    for stmt in fn_def.body:
        if isinstance(stmt, ast.Return):
            return sub_engine.resolve(stmt.value)
        else:
            sub_engine.resolve(stmt.value)
    
    return "Handled"