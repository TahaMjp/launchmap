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
import warnings

from parser.context import ParseContext
from parser.entrypoint.parser_runner import parse_launch_file
from parser.resolution.resolution_engine import ResolutionEngine


def resolve_included_launch_file(
    filename: str, parent_context: ParseContext, passed_arguments: list
) -> dict:
    """
    Parses the included launch file, applying passed arguments and merging introspection.
    """
    if not os.path.isfile(filename):
        warnings.warn(f"Included launch file not found: {filename}")
        return {}

    child_context = ParseContext()
    child_context.introspection = parent_context.introspection
    child_context.current_file = filename

    engine = ResolutionEngine(child_context)
    child_context.engine = engine

    for pair in passed_arguments:
        if isinstance(pair, (list, tuple)) and len(pair) == 2:
            name, value = pair
            child_context.define_variable(name, value)

    return parse_launch_file(filename, child_context)
