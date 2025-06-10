from parser.utils import parse_python_expression
import ast

def test_os_path_join_basic():
    code = "os.path.join('a', 'b', 'c')"
    node = ast.parse(code).body[0].value
    assert parse_python_expression(node) == "a/b/c"

def test_get_package_share_path():
    code = "os.path.join(get_package_share_directory('my_robot'), 'launch', 'bringup.py')"
    node = ast.parse(code).body[0].value
    assert parse_python_expression(node) == "<pkg:my_robot>/launch/bringup.py"