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
from warnings import warn
import ast

@register_resolver(ast.Name)
def resolve_name(node: ast.Name, engine):
    name = node.id

    # Case 1: Defined variable
    if engine.context.has_variable(name):
        return engine.context.lookup_variable(name)
    
    # Case 2: Defined function
    if engine.context.has_function(name):
        return engine.context.lookup_function(name)
    
    # Case 3: Unknown: Fallback to string
    warn(f"Name '{name}' not found in variables or functions. Assuming literal name.")
    return name