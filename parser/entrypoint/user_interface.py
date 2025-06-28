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

from parser.entrypoint.parser_runner import parse_launch_file
from parser.parser.introspection_utils import collect_launch_config_usages
from parser.parser.postprocessing import simplify_launch_configurations
from parser.parser.type_mapping import TYPE_KEY_MAP

def parse_and_format_launch_file(filepath: str) -> dict:
    """
    Parses a ROS2 launch file and return output in the standardized grouped format.
    """
    raw = parse_launch_file(filepath)

    grouped = {
        "nodes": [],
        "arguments": [],
        "includes": [],
        "groups": [],
        "composable_nodes": [],
        "parameters": []
    }

    for item in raw["parsed"]:
        if isinstance(item, dict):
            type_key = item.get("type")
            if type_key:
                group_key = TYPE_KEY_MAP.get(type_key)
                if group_key and group_key in grouped:
                    clean_item = {k: v for k, v in item.items() if k != "type"}
                    grouped[group_key].append(clean_item)
    
    usage_paths = collect_launch_config_usages(grouped)

    grouped["launch_argument_usages"] = usage_paths
    grouped["undeclared_launch_configurations"] = raw.get("undeclared_launch_configurations", [])

    return simplify_launch_configurations(grouped)