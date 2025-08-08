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

from parser.resolution.resolution_registry import register_resolver


@register_resolver(ast.JoinedStr)
def resolve_function_def(node: ast.FunctionDef, engine):
    parts = []
    for value in node.values:
        if isinstance(value, ast.FormattedValue):
            inner = engine.resolve(value.value)
            parts.append(str(inner))
        elif isinstance(value, ast.Constant):
            parts.append(str(value.value))
        else:
            parts.append(f"<unresolved:{type(value).__name__}")

    return "".join(parts)
