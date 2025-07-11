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
from parser.parser.utils.common import group_entities_by_type

def parse_and_format_launch_file(filepath: str) -> dict:
    """
    Parses a ROS2 launch file and return output in the standardized grouped format.
    """
    raw = parse_launch_file(filepath)
    grouped = group_entities_by_type(raw["parsed"])

    if raw.get("composable_node_containers"):
        grouped["composable_nodes_container"] = raw["composable_node_containers"]
        
    grouped["launch_argument_usages"] = collect_launch_config_usages(grouped)
    grouped["undeclared_launch_configurations"] = raw.get("undeclared_launch_configurations", [])

    return simplify_launch_configurations(grouped)