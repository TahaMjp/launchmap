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

from typing import Callable, Dict, List, Type
import ast

_RESOLVERS: Dict[Type[ast.AST], List[Callable]] = {}

def register_symbolic_resolver(node_type: Type[ast.AST]):
    def decorator(func: Callable):
        _RESOLVERS.setdefault(node_type, []).append(func)
        return func
    return decorator

def get_symbolic_resolvers(node_type: Type[ast.AST]) -> List[Callable]:
    return _RESOLVERS.get(node_type, [])