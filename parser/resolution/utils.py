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
import warnings


def resolve_python_expression(left, op, right):
    if isinstance(op, ast.Add):
        return left + right
    elif isinstance(op, ast.Sub):
        return left - right
    elif isinstance(op, ast.Mult):
        return left * right
    elif isinstance(op, ast.Div):
        return left / right
    elif isinstance(op, ast.Mod):
        return left % right
    elif isinstance(op, ast.Pow):
        return left**right
    elif isinstance(op, ast.LShift):
        return left << right
    elif isinstance(op, ast.RShift):
        return left >> right
    elif isinstance(op, ast.BitOr):
        return left | right
    elif isinstance(op, ast.BitAnd):
        return left & right
    elif isinstance(op, ast.BitXor):
        return left ^ right
    else:
        raise NotImplementedError(f"Unsupported op: {type(op).__name__}")


def get_func_name(func_node: ast.expr) -> str:
    """
    Reconstructs the fully qualified name from an AST function call,
    launch.actions.DeclareLaunchArgument -> 'launch.actions.DeclareLaunchArgument'
    """
    if isinstance(func_node, ast.Name):
        return func_node.id
    elif isinstance(func_node, ast.Attribute):
        parts = []
        while isinstance(func_node, ast.Attribute):
            parts.insert(0, func_node.attr)
            func_node = func_node.value
        if isinstance(func_node, ast.Name):
            parts.insert(0, func_node.id)
        return ".".join(parts)
    raise TypeError(f"Unsupported function node type: {type(func_node).__name__}")


def resolve_call_kwargs(node: ast.Call, engine):
    return {kw.arg: engine.resolve(kw.value) for kw in node.keywords}


def resolve_call_signature(node: ast.Call, engine):
    args = [engine.resolve(arg) for arg in node.args]
    kwargs = {kw.arg: engine.resolve(kw.value) for kw in node.keywords}
    return args, kwargs


def try_all_resolvers(node: ast.AST, engine) -> object:
    """
    Attempts to resolve the given AST node using all registered resolvers
    for its type. Returns the first non-None result. Raises error if
    the node is unrecognized and no resolver handles it.
    """
    from parser.resolution.resolution_registry import get_resolvers

    resolvers = get_resolvers(type(node))
    for resolver in resolvers:
        result = resolver(node, engine)
        if result is not None:
            return result

    if not resolvers:
        source = getattr(node, "lineno", "?")
        node_type = type(node).__name__
        warnings.warn(f"Unhandled AST node ({node_type}) at line {source}: {ast.dump(node)}")

    return None


def bind_function_args(
    fn_def: ast.FunctionDef, args: list, kwargs: dict, exclude_first: bool = False
) -> dict:
    """
    Bind args and kwargs to the function definition's parameters.
    Returns a mapping from parameter names to actual passed value.
    """
    param_names = [arg.arg for arg in fn_def.args.args]
    if exclude_first:
        param_names = param_names[1:]

    vararg_name = fn_def.args.vararg.arg if fn_def.args.vararg else None
    kwarg_name = fn_def.args.kwarg.arg if fn_def.args.kwarg else None

    binding = {}

    # Position bindings
    for name, value in zip(param_names, args):
        binding[name] = value

    # Remaining positional args (*args)
    remaining_args = args[len(param_names) :]
    if vararg_name:
        binding[vararg_name] = remaining_args

    # Named kwargs that match a param name
    for name in param_names[len(args) :]:
        if name in kwargs:
            binding[name] = kwargs[name]

    # Remaining **kwargs
    if kwarg_name:
        all_named = set(binding.keys())
        extra_kwargs = {k: v for k, v in kwargs.items() if k not in all_named}
        binding[kwarg_name] = extra_kwargs

    return binding


def collect_assigned_variable_names(stmt: ast.stmt) -> set[str]:
    """
    Collect all variable names that are assigned in a statement.
    """
    assigned = set()

    class AssignVisitor(ast.NodeVisitor):
        def visit_Assign(self, node: ast.Assign):
            for target in node.targets:
                if isinstance(target, ast.Name):
                    assigned.add(target.id)
            self.generic_visit(node)

        def visit_AnnAssign(self, node: ast.AnnAssign):
            if isinstance(node.target, ast.Name):
                assigned.add(node.target.id)
            self.generic_visit(node)

    AssignVisitor().visit(stmt)
    return assigned
