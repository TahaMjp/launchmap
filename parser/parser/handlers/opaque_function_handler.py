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
from parser.context import ParseContext, SymbolicContext
from parser.parser.registry import register_handler
from parser.parser.utils.common import group_entities_by_type
from parser.resolution.utils import resolve_call_signature
from parser.parser.utils.ast_utils import extract_opaque_function
from parser.symbolic.symbolic_engine import SymbolicEngine

@register_handler("OpaqueFunction", "launch.actions.OpaqueFunction")
def handle_opaque_function(node: ast.Call, context: ParseContext) -> dict:
    args, kwargs = resolve_call_signature(node, context.engine)

    fn_candidate = kwargs.get("function") or (args[0] if args else None)

    if isinstance(fn_candidate, ast.FunctionDef):
        function_name = fn_candidate.name
        fn_def = fn_candidate
    elif isinstance(fn_candidate, str):
        function_name = fn_candidate
        fn_def = context.lookup_function(function_name)
        if not fn_def:
            raise ValueError(f"Function '{function_name}' not found in parsed context.")
    else:
        raise ValueError("OpaqueFunction 'function' must be a string or FunctionDef.")
    
    # Build symbolic context + engine
    symbolic_context = SymbolicContext(function_name, context.introspection)
    symbolic_engine = SymbolicEngine(symbolic_context)
    symbolic_context.engine = symbolic_engine

    # Extract returned data and symbolic usage from function body
    result = extract_opaque_function(fn_def, context, symbolic_engine)
    grouped = group_entities_by_type(result)

    return {
        "type": "OpaqueFunction",
        "name": function_name,
        "returns": grouped
    }