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

from typing import Callable, Dict, Optional, Any
from parser.resolution.resolution_engine import ResolutionEngine
import ast

_RESOLVERS: Dict[type, Callable[[ast.AST, 'ResolutionEngine'], Any]] = {}

def register_resolver(node_type: type):
    def decorator(func: Callable[[ast.AST, 'ResolutionEngine'], Any]):
        _RESOLVERS[node_type] = func
        return func
    return decorator

def get_resolver(node_type: type) -> Optional[Callable[[ast.AST, 'ResolutionEngine'], Any]]:
    return _RESOLVERS.get(node_type)