import yaml
import pytest
from parser.parser import parse_launch_string

def load_yaml_tests(file_path):
    with open(file_path, 'r') as f:
        data = yaml.safe_load(f)
        return [
            pytest.param(test["input"], test["expected"], id=test["name"])
            for test in data["tests"]
        ]

@pytest.mark.parametrize("code,expected", load_yaml_tests("test_cases/node_tests.yaml"))
def test_node_parsing(code, expected):
    result = parse_launch_string(code)
    assert result == expected

@pytest.mark.parametrize("code,expected", load_yaml_tests("test_cases/node_params_edge.yaml"))
def test_node_parsing(code, expected):
    result = parse_launch_string(code)
    assert result == expected