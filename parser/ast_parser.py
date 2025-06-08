import ast
from parser.handlers.node_handler import handle_node

class LaunchFileVisitor(ast.NodeVisitor):
    def __init__(self):
        self.result = {"nodes": []}
    
    def visit_Call(self, node: ast.Call):
        # Detect LaunchDescription([...])
        if isinstance(node.func, ast.Name) and node.func.id == "LaunchDescription":
            for arg in node.args:
                if isinstance(arg, ast.List):
                    for elt in arg.elts:
                        self._handle_action(elt)
        self.generic_visit(node)

    def _handle_action(self, node: ast.Call):
        if isinstance(node.func, ast.Name) and node.func.id == "Node":
            node_data = handle_node(node)
            if node_data:
                self.result["nodes"].append(node_data)