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

def simplify_launch_configurations(obj):
    """
    Recursively walk the parsed output and convert
    LaunchConfiguration dicts to string form: "${LaunchConfiguration:name}"
    """
    if isinstance(obj, dict):
        if obj.get("type") == "LaunchConfiguration":
            name = obj.get("name")
            return f"${{LaunchConfiguration:{name}}}"
        elif obj.get("type") == "IfCondition":
            expression = simplify_launch_configurations(obj.get("expression"))
            return f"${{IfCondition:{expression}}}"
        
        return {k: simplify_launch_configurations(v) for k, v in obj.items()}

    elif isinstance(obj, (list, tuple)):
        return [simplify_launch_configurations(i) for i in obj]
    
    return obj