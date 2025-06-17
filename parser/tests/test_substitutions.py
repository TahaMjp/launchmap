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

    code = """PathJoinSubstitution(['/a', 'b', 'c'])"""
    assert _parse_call(code) == "${PathJoinSubstitution:/a/b/c}"

def test_string_boolean_parsing():
    assert _resolve_const(ast.Constant("true")) is True
    assert _resolve_const(ast.Constant("false")) is False
    assert _resolve_const(ast.Constant("TRUE")) is True