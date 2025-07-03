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

# Mapping from AST operator to symbolic string representation
COMPARE_OPS = {
    ast.Eq: "==",
    ast.NotEq: "!=",
    ast.Lt: "<",
    ast.LtE: "<=",
    ast.Gt: ">",
    ast.GtE: ">=",
    ast.In: ".in",
    ast.NotIn: ".not_in",
    ast.Is: "is",
    ast.IsNot: "is not",
}

@register_resolver(ast.Compare)
def resolve_compare(node: ast.Compare, engine):
    left = engine.resolve(node.left)

    comparisons = []
    for op, comparator in zip(node.ops, node.comparators):
        op_type = type(op)
        right = engine.resolve(comparator)

        if op_type in (ast.In, ast.NotIn):
            op_str = COMPARE_OPS[op_type]
            expr = f"{left}{op_str}({right})"
        elif op_type in COMPARE_OPS:
            op_str = COMPARE_OPS[op_type]
            expr = f"{left} {op_str} {right}"
        else:
            expr = f"{left} ? {right}"

        comparisons.append(expr)
        left = right

    return " and ".join(comparisons) if len(comparisons) > 1 else comparisons[0]