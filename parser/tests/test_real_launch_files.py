import os
import pytest
import json
from parser.parser import parse_launch_string

REAL_CASES_DIR = os.path.join(os.path.dirname(__file__), "real_cases")

def load_real_launch_files():
    cases = []
    for file in os.listdir(REAL_CASES_DIR):
        if file.endswith(".py"):
            path = os.path.join(REAL_CASES_DIR, file)
            with open(path, "r") as f:
                code = f.read()
            cases.append((file, code))
    return cases

@pytest.mark.parametrize("filename,code", load_real_launch_files())
def test_parse_real_launch_files(filename, code):
    result = parse_launch_string(code)

    print(f"\nParsed output for {filename}:\n", json.dumps(result, indent=2))

    assert not isinstance(result, dict), f"{filename} did not return a dict"
    assert any(k in result for k in ("nodes", "includes", "groups")), f"{filename} returned empyt or incomplete structure."