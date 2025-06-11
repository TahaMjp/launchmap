import ast
import json
from parser.ast_parser import LaunchFileVisitor

def parse_launch_string(code: str) -> dict:
    tree = ast.parse(code)
    visitor = LaunchFileVisitor()
    visitor.visit(tree)
    return visitor.result

def export_launch_to_json(code: str, output_file = "launch_graph.json"):
    parsed = parse_launch_string(code)
    with open(output_file, "w") as f:
        json.dump(parsed, f, indent=2)
    print(f"Launch structure exported to {output_file}")