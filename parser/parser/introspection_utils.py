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

from collections import defaultdict
from parser.parser.type_mapping import TYPE_KEY_MAP
import re

LAUNCH_CONFIG_REGEX = re.compile(r"\${LaunchConfiguration:([a-zA-Z0-9_]+)}")
EVENT_HANDLER_REGEX = re.compile(r"\${EventHandler\[(\d+)\]:(\w+)}")

def collect_launch_config_usages(grouped: dict) -> list[dict]:
    """
    Recursively walk the grouped data and return all LaunchConfiguration usages
    with field + JSON-style path, for traceability.
    """
    usages = []

    def walk(obj, path):
        if isinstance(obj, dict):
            for key, value in obj.items():
                if key == "type" and obj.get("type") == "LaunchConfiguration":
                    arg = obj.get("name")
                    usages.append({
                        "argument": arg,
                        "path": path
                    })
                else:
                    walk(value, f"{path}.{key}" if path else key)

        elif isinstance(obj, list):
            for idx, item in enumerate(obj):
                walk(item, f"{path}[{idx}]")
        
        elif isinstance(obj, str):
            for match in LAUNCH_CONFIG_REGEX.finditer(obj):
                usages.append({
                    "argument": match.group(1),
                    "path": path
                })
    
    for top_key in TYPE_KEY_MAP.values():
        for idx, entry in enumerate(grouped.get(top_key, [])):
            walk(entry, f"{top_key}[{idx}]")
    
    return usages

def collect_event_handler_usages(grouped: dict) -> list[dict]:
    """
    Recursively walk the grouped data and return all event handler usages
    with index, type and path in the structured output.
    """
    usage_map = defaultdict(lambda: {"type": None, "triggered_by": [], "triggers": []})

    def walk(obj, path):
        if isinstance(obj, dict):
            for key, value in obj.items():
                walk(value, f"{path}.{key}" if path else key)
        elif isinstance(obj, list):
            for idx, item in enumerate(obj):
                walk(item, f"{path}[{idx}]")
        elif isinstance(obj, str):
            for match in EVENT_HANDLER_REGEX.finditer(obj):
                idx = int(match.group(1))
                handler_type = match.group(2)

                usage_map[idx]["type"] = handler_type

                if ".triggers" in path:
                    usage_map[idx]["triggered_by"].append(path)
                elif ".triggered_by" in path:
                    usage_map[idx]["triggers"].append(path)
    
    for key in TYPE_KEY_MAP.values():
        for idx, item in enumerate(grouped.get(key, [])):
            walk(item, f"{key}[{idx}]")
    
    return [v for v in usage_map.values() if v["triggered_by"] or v["triggers"]]