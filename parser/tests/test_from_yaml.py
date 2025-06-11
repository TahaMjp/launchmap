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
    assert result.get("nodes") == expected.get("nodes")

@pytest.mark.parametrize("code,expected", load_yaml_tests("test_cases/node_params_edge.yaml"))
def test_node_params_edge_parsing(code, expected):
    result = parse_launch_string(code)
    assert result.get("nodes") == expected.get("nodes")

@pytest.mark.parametrize("code,expected", load_yaml_tests("test_cases/declare_argument_tests.yaml"))
def test_declare_argument_parsing(code, expected):
    result = parse_launch_string(code)
    assert result.get("arguments") == expected.get("arguments")

@pytest.mark.parametrize("code,expected", load_yaml_tests("test_cases/include_launch_tests.yaml"))
def test_include_launch_parsing(code, expected):
    result = parse_launch_string(code)
    assert result.get("includes") == expected.get("includes")

@pytest.mark.parametrize("code,expected", load_yaml_tests("test_cases/group_action_tests.yaml"))
def test_group_action_parsing(code, expected):
    result = parse_launch_string(code)
    assert result.get("groups") == expected.get("groups")