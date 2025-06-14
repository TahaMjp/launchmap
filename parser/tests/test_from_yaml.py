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

@pytest.mark.parametrize("code,expected", load_yaml_tests("test_cases/launch_argument_usage_tests.yaml"))
def test_launch_argument_usage_tracking(code, expected):
    result = parse_launch_string(code)
    assert result.get("arguments", []) == expected.get("arguments", [])
    assert result.get("groups", []) == expected.get("groups", [])
    assert result.get("includes", []) == expected.get("includes", [])
    print(result.get("launch_argument_usages", []))
    print(expected.get("launch_argument_usages", []))
    assert result.get("launch_argument_usages", []) == expected.get("launch_argument_usages", [])

@pytest.mark.parametrize("code,expected", load_yaml_tests("test_cases/variable_assignment_tests.yaml"))
def test_variable_assignment(code, expected):
    result = parse_launch_string(code)
    assert result.get("arguments", []) == expected.get("arguments", [])
    assert result.get("groups", []) == expected.get("groups", [])
    assert result.get("includes", []) == expected.get("includes", [])

@pytest.mark.parametrize("code,expected", load_yaml_tests("test_cases/undeclared_arguments_tests.yaml"))
def test_undeclared_arguments(code, expected):
    result = parse_launch_string(code)
    assert result.get("arguments", []) == expected.get("arguments", [])
    assert result.get("nodes", []) == expected.get("nodes", [])
    print(result.get("launch_argument_usages"))
    print(expected.get("launch_argument_usages"))
    assert result.get("launch_argument_usages", []) == expected.get("launch_argument_usages", [])
    assert result.get("undeclared_launch_configurations", []) == expected.get("undeclared_launch_configurations", [])