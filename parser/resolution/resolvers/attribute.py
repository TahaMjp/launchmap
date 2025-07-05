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

import re
from parser.resolution.resolution_registry import register_resolver
import ast

@register_resolver(ast.Attribute)
def resolve_attribute(node: ast.Attribute, engine):
    if engine.context.strategy == "symbolic":
        return _resolve_symbolic_attribute(node, engine)
    else:
        return _resolve_normal_attribute(node, engine)

def _resolve_symbolic_attribute(node: ast.Attribute, engine):
    base = engine.resolve(node.value)

    if isinstance(base, str) and re.search(r"\$\{[^}]+\}", base):
        return f"{base}.{node.attr}"
    
    try:
        return getattr(base, node.attr)
    except AttributeError:
        raise ValueError(f"Object of type {type(base).__name__} has no attribute '{node.attr}'")

def _resolve_normal_attribute(node: ast.Attribute, engine):
    base_object = engine.resolve(node.value)

    if base_object is None:
        raise ValueError(f"Cannot access attribute '{node.attr}' on None")
    
    try:
        return getattr(base_object, node.attr)
    except AttributeError:
        raise ValueError(f"Object of type {type(base_object).__name__} has no attribute '{node.attr}'")


def get_attr_chain(attr_node: ast.Attribute) -> str:
    parts = []
    while isinstance(attr_node, ast.Attribute):
        parts.insert(0, attr_node.attr)
        attr_node = attr_node.value
    if isinstance(attr_node, ast.Name):
        parts.insert(0, attr_node.id)
    return ".".join(parts)
    