import ast
from parser.substitutions import parse_substitution, _resolve_const

def _parse_call(code):
    node = ast.parse(code).body[0].value
    return parse_substitution(node)

def test_launch_config_basic():
    assert _parse_call("LaunchConfiguration('foo')") == "${LaunchConfiguration:foo}"

def test_launch_config_with_default():
    assert _parse_call("LaunchConfiguration('bar', default='baz')") == "${LaunchConfiguration:bar:default=baz}"

def test_env_var():
    assert _parse_call("EnvironmentVariable('ROBOT')") == "${EnvironmentVariable:ROBOT}"

def test_path_join_sub():
    code = """PathJoinSubstitution([
        TextSubstitution(text='/a'),
        TextSubstitution(text='b'),
        TextSubstitution(text='c')
    ])"""
    assert _parse_call(code) == "${PathJoinSubstitution:/a/b/c}"

def test_string_boolean_parsing():
    assert _resolve_const(ast.Constant("true")) is True
    assert _resolve_const(ast.Constant("false")) is False
    assert _resolve_const(ast.Constant("TRUE")) is True