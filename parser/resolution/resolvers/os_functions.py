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
from parser.parser.postprocessing import simplify_launch_configurations
from parser.resolution.resolution_registry import register_resolver

@register_resolver(ast.Call, priority=10)
def resolve_os_functions(node: ast.Call, engine):
    func_name = engine.context.get_func_name(node.func)

    # Support any os.* or os.path.* function
    if not func_name.startswith("os.") and not func_name.startswith("os.path."):
        return None
    
    args = [engine.resolve(arg) for arg in node.args]
    simplified_args = simplify_launch_configurations(args)
    return f"${{{func_name}:{simplified_args}}}"