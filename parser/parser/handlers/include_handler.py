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
from parser.context import ParseContext
from parser.parser.registry import register_handler
from parser.resolution.utils import resolve_call_signature

@register_handler("IncludeLaunchDescription", "launch.actions.IncludeLaunchDescription")
def handle_include(node: ast.Call, context: ParseContext) -> dict:
    """
    Handle IncludeLaunchDescription (launch_description_source=..., launch_arguments=...)
    - Resolves the included launch file via Include Resolver
    - Recursively parses the include file and merge results
    """
    args, kwargs = resolve_call_signature(node, context.engine)

    launch_source = kwargs.get("launch_description_source")
    if not launch_source and args:
        launch_source = args[0]

    launch_args = kwargs.get("launch_arguments", [])
    if not launch_args and len(args) > 1:
        launch_args = args[1]

    if not isinstance(launch_source, dict) or "filename" not in launch_source:
        raise ValueError("Could not resolve include path from launch_description_source")
    
    # Load and parse included file
    from parser.includes.resolver import resolve_included_launch_file
    included_output = resolve_included_launch_file(
        filename = launch_source["filename"],
        parent_context = context,
        passed_arguments = launch_args
    )

    return {
        "type": "IncludeLaunchDescription",
        "launch_description_source": launch_source["filename"],
        "launch_arguments": launch_args,
        "included": included_output
    }