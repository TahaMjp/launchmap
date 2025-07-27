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
import tempfile
import textwrap
from parser.plugin_loader import load_user_handlers_from_directory
from parser.entrypoint.user_interface import parse_and_format_launch_file

def test_load_user_handler_from_directory():
    with tempfile.TemporaryDirectory() as tmpdir:
        # Create custom_api.py
        with open(os.path.join(tmpdir, "custom_api.py"), "w") as f:
            f.write(textwrap.dedent("""
                from parser.parser.user_handler import register_user_handler
                
                @register_user_handler("MyCustomLaunchThing")
                def handle_custom(node, context):
                    return {
                        "type": "CustomHandler",
                        "type_name": "MyCustomLaunchThing",
                        "metadata": {
                            "info": "example"
                        }                
                    }
            """))

        # Write launch_file.py
        with open(os.path.join(tmpdir, "launch_file.launch"), "w") as f:
            f.write(textwrap.dedent("""
                from launch import LaunchDescription
                from custom_api import MyCustomLaunchThing
                                    
                def generate_launch_description():
                    return LaunchDescription([
                        MyCustomLaunchThing()                
                    ])
            """))

        # Load custom handler
        load_user_handlers_from_directory(tmpdir)

        # Run parser
        result = parse_and_format_launch_file(os.path.join(tmpdir, "launch_file.launch"))

        # Assert output
        custom = [c for c in result["custom_components"]]
        assert len(custom) == 1
        assert custom[0]["type_name"] == "MyCustomLaunchThing"
        assert custom[0]["metadata"] == {"info": "example"}