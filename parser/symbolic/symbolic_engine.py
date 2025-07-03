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
from parser.symbolic.loader import register_builtin_resolvers
from parser.symbolic.symbolic_registry import get_symbolic_resolvers

class SymbolicEngine:
    def __init__(self, context):
        register_builtin_resolvers()
        self.context = context
        self.trace = {}

    def resolve(self, node: ast.AST):
        for resolver in get_symbolic_resolvers(type(node)):
            result = resolver(node, self)
            if result is not None:
                return result
        return None
