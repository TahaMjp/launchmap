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

import importlib.util
import os

def load_user_handlers_from_directory(base_dir):
    if not os.path.isdir(base_dir):
        print(f"Plugin directory '{base_dir}' not found.")
        return
    
    for file in os.listdir(base_dir):
        if file.endswith('.py'):
            path = os.path.join(base_dir, file)
            spec = importlib.util.spec_from_file_location(f"user_plugin_{file}", path)
            if spec and spec.loader:
                module = importlib.util.module_from_spec(spec)
                try:
                    spec.loader.exec_module(module)
                    print(f"[LaunchMap] Loaded plugin: {file}")
                except Exception as e:
                    print(f"[LaunchMap] Failed to load plugin {file}: {e}")