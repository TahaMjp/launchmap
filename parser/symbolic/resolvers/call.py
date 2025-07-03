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
import warnings
from parser.parser.dispatcher import dispatch_call
from parser.parser.postprocessing import simplify_launch_configurations
from parser.symbolic.symbolic_registry import register_symbolic_resolver

@register_symbolic_resolver(ast.Call)
def resolve_call(node: ast.Call, engine):
    try:
        # Try launching launch_ros contruct
        output = dispatch_call(node, engine.context)
        return simplify_launch_configurations(output)
    except ValueError:
        func_name = engine.context.get_func_name(node.func)

        # Check if it is a launch_ros construct (heuristic: capitalized name)
        if func_name and func_name[0].isupper():
            warnings.warn(f"No handler registered for launch construct: '{func_name}'")
            return None

        # Symbolic fallback for unresolved method/function calls
        try:
            base = engine.resolve(node.func.value)
            method = node.func.attr
            args = [engine.resolve(arg) for arg in node.args]
            kwargs = {kw.arg: engine.resolve(kw.value) for kw in node.keywords}

            # Build symbolic string representation
            arg_strs = [repr(a) for a in args] + [f"{k}={repr(v)}" for k, v in kwargs.items()]
            return f"{base}.{method}({', '.join(arg_strs)})"
        
        except Exception as e:
            raise ValueError(f"Unable to symbolically resolve call: {ast.dump(node)} -> {e}")