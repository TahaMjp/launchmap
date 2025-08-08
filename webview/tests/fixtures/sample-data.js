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

export const SAMPLE_DATA = {
  arguments: [
    { name: 'robot_name', default_value: 'rrbot' }
  ],
  environment_variables: [
    { name: 'ROS_DISTRO', default_value: 'humble' }
  ],
  python_expressions: [
    {
      body: 'if ROS_DISTRO == \'humble\': tb3_param_dir = LaunchConfiguration(\'tb3_param_dir\')',
      variables: ['${var:tb3_param_dir}']
    }
  ],
  nodes: [
    { package: 'demo_nodes', executable: 'talker', name: 'talker_node', output: 'screen' }
  ],
  includes: [
    { launch_file: 'demo.launch.py', arguments: [] }
  ],
  groups: [
    { namespace: 'test_ns', actions: { nodes: [{ package: 'demo_pkg', executable: 'listener' }] } }
  ],
  timer_actions: [
    {
      period: '5.0',
      actions: {
        nodes: [{ package: 'demo_nodes', executable: 'delayed_talker' }]
      }
    }
  ],
  opaque_functions: [
    {
      name: 'launch_setup',
      returns: {
        nodes: [
          { package: 'robot_state_publisher', executable: 'rsp', name: 'rsp_node' }
        ]
      }
    }
  ],
  composable_nodes_container: [
    {
      target_container: 'my_container',
      package: 'rclcpp_components',
      executable: 'component_container_mt',
      composable_nodes: [
        { package: 'bar_pkg', plugin: 'bar_pkg/BarNode', name: 'bar' }
      ]
    }
  ],
  event_handlers: [
    {
      type: 'OnProcessExit',
      triggered_by: ['nodes[0].events.triggers'],
      triggers: ['nodes[0].events.triggered_by']
    }
  ]
};

export const COMPONENT_SELECTORS = [
  { type: 'argument', selector: '.argument-block' },
  { type: 'env-var', selector: '.environment-variable-block' },
  { type: 'node', selector: '.node-block' },
  { type: 'include', selector: '.include-block' },
  { type: 'group', selector: '.group-header' },
  { type: 'group-node', selector: '.group-block .node-block' },
  { type: 'opaque', selector: '.opaque-function-header' },
  { type: 'opaque-node', selector: '.opaque-function-block .node-block' },
  { type: 'composable-container', selector: '.composable-container-header' },
  { type: 'composable-node', selector: '.composable-node-block' },
  { type: 'python-expression', selector: '.python-expression-block' },
  { type: 'timer-action', selector: '.timer-action-header' },
  { type: 'timer-action-node', selector: '.timer-action-block .node-block' },
  { type: 'event-handler', selector: '.event-handler-block' }
];
