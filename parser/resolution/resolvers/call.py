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

from parser.resolution.resolution_registry import register_resolver
from parser.parser.dispatcher import dispatch_call
import ast

@register_resolver(ast.Call)
def resolve_call(node: ast.Call, engine):
    try:
        # Try launching launch_ros contruct
        return dispatch_call(node, engine.context)
    except ValueError:
        func_name = engine.context.get_func_name(node.func)

        # Check if it is a launch_ros construct (heuristic: capitalized name)
        if func_name and func_name[0].isupper():
            raise NotImplementedError(f"No handler registered for launch construct: '{func_name}'")

        # Otherwise Fallback: Python function or method call (e.g. .items())
        func = engine.resolve(node.func)
        args = [engine.resolve(arg) for arg in node.args]
        kwargs = {kw.arg: engine.resolve(kw.value) for kw in node.keywords}

        if callable(func):
            result = func(*args, **kwargs)
            if isinstance(result, type({}.items())):
                result = dict(result)
            return result
    
        raise ValueError(f"Cannot resolve non-callable: {func}")