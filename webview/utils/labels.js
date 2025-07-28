// Copyright (c) 2025 Kodo Robotics
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

export function getTypeLabel(type) {
    switch (type) {
        case 'node': return 'NODE';
        case 'include': return 'INCLUDE LAUNCH DESCRIPTION';
        case 'group': return 'GROUP ACTION';
        case 'timer-action': return 'TIMER ACTION';
        case 'argument': return 'DECLARE LAUNCH ARGUMENT';
        case 'environment-variable': return 'ENVIRONMENT VARIABLE';
        case 'opaque-function': return 'OPAQUE FUNCTION';
        case 'composable-node': return 'COMPOSABLE NODE';
        case 'composable-container': return 'COMPOSABLE NODE CONTAINER';
        case 'event-handler': return 'EVENT HANDLER';
        case 'python-expression': return 'PYTHON EXPRESSION';
        default: return `${type}`;
    }
}