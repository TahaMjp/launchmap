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

class IntrospectionTracker:
    def __init__(self):
        # All declared arguments from DeclareLaunchArgument
        self.declared_launch_args: dict[str, dict] = {}

        # All names used in LaunchConfiguration(...)
        self.used_launch_configs: set[str] = set()

    def track_launch_arg_declaration(self, name: str, metadata: dict):
        """
        Called by DeclareLaunchArgument handler.
        Stores default_value, description, etc.
        """
        self.declared_launch_args[name] = metadata or {}

    def track_launch_config_usage(self, name: str):
        """
        Called by LaunchConfiguration handler.
        Adds the name to the used set.
        """
        self.used_launch_configs.add(name)
    
    def get_undeclared_launch_configs(self) -> set[str]:
        """
        Returns all LaunchConfigurations used but not declared.
        Useful for validation and diagnostics.
        """
        return self.used_launch_configs - set(self.declared_launch_args.keys())