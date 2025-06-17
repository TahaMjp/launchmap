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

    assert isinstance(result, dict), f"{filename} did not return a dict"
    assert any(k in result for k in ("nodes", "includes", "groups")), f"{filename} returned empyt or incomplete structure."