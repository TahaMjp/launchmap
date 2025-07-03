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

from parser.introspection.tracker import IntrospectionTracker
from typing import Any
import ast

class ParseContext:
    def __init__(self):
        # Variable assignments within the launch file
        self.variables: dict[str, Any] = {}

        # Function definitions by name
        self.functions: dict[str, ast.FunctionDef] = {}

        # Current launch file being parsed (useful for trace/debug)
        self.current_file: str | None = None

        # Namespace tracking for PushRosNamespace
        self.namespace_stack: list[str] = []

        # Shared introspection tracker (for declarations, usages, etc.)
        self.introspection = IntrospectionTracker()

        # Resolution engine reference (set externally after creation)
        self.engine = None

    ## Variable Management

    def define_variable(self, name: str, value: Any):
        """ Define or update a variable in context. """
        self.variables[name] = value

    def has_variable(self, name: str) -> bool:
        """ Check variable exists in context """
        return name in self.variables

    def lookup_variable(self, name: str) -> Any:
        """ Resolve a variable by name. Raises if undefined. """
        if name not in self.variables:
            raise NameError(f"Variable '{name}' is not defined in context.")
        return self.variables[name]
    
    ## Function API

    def define_function(self, name: str, fn_def: ast.FunctionDef):
        """ Define or update a function in context. """
        self.functions[name] = fn_def

    def has_function(self, name: str) -> bool:
        """ Check function exists in context. """
        return name in self.functions

    def lookup_function(self, name: str) -> ast.FunctionDef | None:
        """ Return the AST of a function by name, or None if not found. """
        return self.functions.get(name)
    
    ## Namespace stack
    
    def push_namespace(self, ns: str):
        self.namespace_stack.append(ns)
    
    def pop_namespace(self):
        if self.namespace_stack:
            self.namespace_stack.pop()

    def current_namespace(self) -> str | None:
        return "/".join(self.namespace_stack) if self.namespace_stack else None
    
    ## Utility

    def get_func_name(self, func_node) -> str:
        """ Resolve a dotted name from an AST function call. """
        from parser.resolution.utils import get_func_name
        return get_func_name(func_node)
    

class SymbolicContext:
    def __init__(self, function_name: str = None, introspection: IntrospectionTracker = None):
        # Meta data
        self.function_name = function_name
        
        # Variable names
        self.variables = {}

        # Namespace tracking for PushRosNamespace
        self.namespace_stack: list[str] = []

        # Shared introspection tracker (for declarations, usages, etc.)
        self.introspection = introspection or IntrospectionTracker()

        # Resolution engine reference (set externally after creation)
        self.engine = None
    
    ## Variable Management

    def define_variable(self, name: str, value: Any):
        """ Define or update a variable in context. """
        self.variables[name] = value

    def lookup_variable(self, name: str) -> Any:
        """ Resolve a variable by name. Raises if undefined. """
        if name not in self.variables:
            raise NameError(f"Variable '{name}' is not defined in context.")
        return self.variables[name]
    
    ## Namespace stack
    
    def push_namespace(self, ns: str):
        self.namespace_stack.append(ns)
    
    def pop_namespace(self):
        if self.namespace_stack:
            self.namespace_stack.pop()

    def current_namespace(self) -> str | None:
        return "/".join(self.namespace_stack) if self.namespace_stack else None
    
    ## Utility
    def get_func_name(self, func_node) -> str:
        """ Resolve a dotted name from an AST function call. """
        from parser.resolution.utils import get_func_name
        return get_func_name(func_node)
    