# Copyright (c) 2025 Kodo Robotics
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

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