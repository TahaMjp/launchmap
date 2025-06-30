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
import ast

@register_resolver(ast.BoolOp)
def resolve_boolop(node: ast.BoolOp, engine):
    values = [engine.resolve(v) for v in node.values]
    if isinstance(node.op, ast.And):
        return all(values)
    elif isinstance(node.op, ast.Or):
        return any(values)
    raise NotImplementedError(f"Unsupported boolean operator: {type(node.op).__name__}")
