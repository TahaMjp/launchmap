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

import io
import os
import tempfile
from unittest.mock import patch, mock_open, MagicMock
from contextlib import contextmanager

@contextmanager
def mock_tempfile_and_fs(code: str, files: dict):
    """
    Mocks tempfile.NameTemporaryFile and open/os.path.exists for recursive launch testing.
    The entrypoint launch file with be named 'main.py'
    """

    # Fake in-memory file for main launch string
    fake_main = io.StringIO(code)
    fake_main.name = "main.py"

    def fake_named_tempfile(*args, **kwargs):
        return fake_main

    def fake_file_loader(path, *args, **kwargs):
        for name, content in files.items():
            if path.endswith(name):
                return io.StringIO(content)
        raise FileNotFoundError(f"No such file: {path}")

    with patch("tempfile.NamedTemporaryFile", side_effect=fake_named_tempfile), \
         patch("builtins.open", side_effect=fake_file_loader), \
         patch("os.path.exists", side_effect=lambda p: any(p.endswith(name) for name in files)):
        yield