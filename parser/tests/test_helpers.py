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

import tempfile
import yaml
import pytest
from parser.entrypoint.parser_runner import parse_launch_file
from parser.entrypoint.user_interface import parse_and_format_launch_file

def load_yaml_tests(file_path):
    with open(file_path, 'r') as f:
        data = yaml.safe_load(f)
        return [
            pytest.param(test["input"], test["expected"], id=test["name"])
            for test in data["tests"]
        ]

def parse_launch_string(code: str) -> dict:
    with tempfile.NamedTemporaryFile("w", suffix=".py", delete=False) as tmp:
        tmp.write(code)
        tmp.flush()
        return parse_and_format_launch_file(tmp.name)