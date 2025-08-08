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
from typing import Callable, Dict, List, Tuple, Type

from parser.resolution.resolution_engine import ResolutionEngine

# (priority, resolver_function)
_RESOLVERS: Dict[Type[ast.AST], List[Tuple[int, Callable]]] = {}


def register_resolver(node_type: Type[ast.AST], *, priority: int = 0):
    """
    Register a resolver for a given AST node type with an optional priority.
    Higher priority resolvers are tried first.
    """

    def decorator(func: Callable[[ast.AST, "ResolutionEngine"], any]):
        _RESOLVERS.setdefault(node_type, []).append((priority, func))
        _RESOLVERS[node_type].sort(key=lambda pair: -pair[0])
        return func

    return decorator


def get_resolvers(node_type: Type[ast.AST]) -> List[Callable]:
    """
    Return resolvers sorted by priority (high to low)
    """
    return [f for _, f in _RESOLVERS.get(node_type, [])]
