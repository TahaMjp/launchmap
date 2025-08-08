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
        type_key = obj.get("type")

        if type_key in simplifier_registry:
            return simplifier_registry[type_key](obj)

        # Otherwise recursively simplify dictionary
        return {k: simplify_launch_configurations(v) for k, v in obj.items()}

    elif isinstance(obj, (list, tuple)):
        return [simplify_launch_configurations(i) for i in obj]

    return obj


def format_symbolic_part(part):
    if isinstance(part, dict):
        return simplify_launch_configurations(part)
    elif isinstance(part, str):
        return f"'{part}'"
    else:
        return str(part)


def _simplify_launch_config(obj):
    name = obj.get("name")
    return f"${{LaunchConfiguration:{name}}}"


def _simplify_environment_variable(obj):
    name = obj.get("name")
    return f"${{EnvironmentVariable:{name}}}"


def _simplify_path_join(obj):
    parts = ", ".join(format_symbolic_part(p) for p in obj.get("parts"))
    return f"${{PathJoinSubstitution:[{parts}]}}"


def _simplify_find_package(obj):
    package = obj.get("package")
    return f"${{FindPackageShare:{package}}}"


def _simplify_find_executable(obj):
    name = obj.get("name")
    return f"${{FindExecutable:{name}}}"


def _simplify_command(obj):
    commands = ", ".join(format_symbolic_part(p) for p in obj.get("command"))
    return f"${{Command:[{commands}]}}"


def _simplify_this_launch_file_dir(obj):
    return "${ThisLaunchFileDir}"


def _simplify_custom_handler(obj):
    type_name = obj.get("type_name")
    kwarg_strs = [
        f"{k}={format_symbolic_part(v)}" for k, v in obj.items() if k not in {"type", "type_name"}
    ]
    kwargs = ", ".join(kwarg_strs)
    return f"${{CustomHandler:{type_name}({kwargs})}}"


# Dispatcher registry
simplifier_registry = {
    "LaunchConfiguration": _simplify_launch_config,
    "EnvironmentVariable": _simplify_environment_variable,
    "PathJoinSubstitution": _simplify_path_join,
    "FindPackageShare": _simplify_find_package,
    "Command": _simplify_command,
    "FindExecutable": _simplify_find_executable,
    "ThisLaunchFileDir": _simplify_this_launch_file_dir,
    "CustomHandler": _simplify_custom_handler,
}
