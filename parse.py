import sys
import json
from parser.parser import parse_launch_string


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python parse.py <launch_file.py>", file=sys.stderr)
        sys.exit(1)
    
    filepath = sys.argv[1]
    with open(filepath, "r") as f:
        code = f.read()
    
    result = parse_launch_string(code)
    print(json.dumps(result))