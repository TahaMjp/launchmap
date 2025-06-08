import ast
from parser.ast_parser import LaunchFileVisitor

def parse_launch_string(code: str) -> dict:
    tree = ast.parse(code)
    visitor = LaunchFileVisitor()
    visitor.visit(tree)
    return visitor.result
