import ast
from parser.context import ParseContext
from parser.handlers.node_handler import handle_node
from parser.handlers.declare_argument_handler import handle_declare_argument
from parser.handlers.include_handler import handle_include
from parser.handlers.group_handler import handle_group_action

class LaunchFileVisitor(ast.NodeVisitor):
    def __init__(self):
        self.result = {
            "nodes": [],
            "arguments": [],
            "includes": [],
            "groups": [],
            "launch_argument_usages": []    
        }

        self.launch_arguments = set()
        self.path_stack = []
    
    def visit_Call(self, node: ast.Call):
        # Detect LaunchDescription([...])
        if isinstance(node.func, ast.Name) and node.func.id == "LaunchDescription":
            for arg in node.args:
                if isinstance(arg, ast.List):
                    for elt in arg.elts:
                        self._handle_action(elt)
        self.generic_visit(node)

    def track_launch_arg_usage(self, arg_name, field):
        usage = {
            "argument": arg_name,
            "field": field,
            "path": ".".join(self.path_stack) if self.path_stack else []
        }
        self.result.setdefault("launch_argument_usages", []).append(usage)

    def with_path(self, label: str, handler_fn, node) -> dict:
        self.path_stack.append(label)
        ctx = ParseContext(visitor = self)
        result = handler_fn(node, ctx)
        self.path_stack.pop()
        return result

    def _handle_action(self, node: ast.Call, into=None):
        target = into if into is not None else self.result
        func_id = getattr(node.func, 'id', None)

        def next_index(key):
            return len(target.get(key, []))

        if func_id == "Node":
            node_index = next_index("nodes")
            node_data = self.with_path(f"nodes[{node_index}]", handle_node, node)
            if node_data:
                target.setdefault("nodes", []).append(node_data)
        
        elif func_id == "DeclareLaunchArgument":
            arg_data = handle_declare_argument(node)
            if arg_data:
                target.setdefault("arguments", []).append(arg_data)
        
        elif func_id == "IncludeLaunchDescription":
            include_index = next_index("includes")
            include_data = self.with_path(f"includes[{include_index}]", handle_include, node)
            if include_data:
                target.setdefault("includes", []).append(include_data)
        
        elif func_id == "GroupAction":
            group_index = next_index("groups")
            group_data = self.with_path(f"groups[{group_index}]", handle_group_action, node)
            if group_data:
                target.setdefault("groups", []).append(group_data)
        