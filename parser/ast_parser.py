import ast
from parser.handlers.node_handler import handle_node
from parser.handlers.declare_argument_handler import handle_declare_argument
from parser.handlers.include_handler import handle_include
from parser.handlers.group_handler import handle_group_action

class LaunchFileVisitor(ast.NodeVisitor):
    def __init__(self):
        self.result = {
            "nodes": [],
            "arguments": [],    
        }
    
    def visit_Call(self, node: ast.Call):
        # Detect LaunchDescription([...])
        if isinstance(node.func, ast.Name) and node.func.id == "LaunchDescription":
            for arg in node.args:
                if isinstance(arg, ast.List):
                    for elt in arg.elts:
                        self._handle_action(elt)
        self.generic_visit(node)

    def _handle_action(self, node: ast.Call, into=None):
        target = into if into is not None else self.result

        func_id = getattr(node.func, 'id', None)

        if func_id == "Node":
            node_data = handle_node(node)
            if node_data:
                target.setdefault("nodes", []).append(node_data)
        
        elif func_id == "DeclareLaunchArgument":
            arg_data = handle_declare_argument(node)
            if arg_data:
                target.setdefault("arguments", []).append(arg_data)
        
        elif func_id == "IncludeLaunchDescription":
            include_data = handle_include(node)
            if include_data:
                target.setdefault("includes", []).append(include_data)
        
        elif func_id == "GroupAction":
            group_data = handle_group_action(node, self)
            if group_data:
                target.setdefault("groups", []).append(group_data)
        