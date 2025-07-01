#! /usr/bin/python3
import argparse
import os.path
import sys
import yaml

from rclpy.parameter import PARAMETER_SEPARATOR_STRING
from ros2cli.node.direct import DirectNode
from ros2cli.node.strategy import add_arguments
from ros2cli.node.strategy import NodeStrategy
from ros2node.api import get_absolute_node_name
from ros2node.api import NodeNameCompleter
from ros2node.api import parse_node_name
from ros2node.api import wait_for_node

from ros2param.api import call_get_parameters
from ros2param.api import call_list_parameters
from ros2param.api import get_value
from ros2param.verb import VerbExtension


class NodeParamsGetter(VerbExtension):
    """Show all the parameters of a node in a YAML file format."""

    def add_arguments(self, parser, cli_name):
        add_arguments(parser)
        arg = parser.add_argument('node_name', help='Name of the ROS node')
        arg.completer = NodeNameCompleter(include_hidden_nodes_key='include_hidden_nodes')
        parser.add_argument('--include-hidden-nodes', action='store_true', help='Consider hidden nodes as well')
        parser.add_argument('--timeout', metavar='N', type=int, default=1, help='Wait for N seconds until node becomes available (default %(default)s sec)')

    def insert_dict(self, dictionary, key, value):
        split = key.split(PARAMETER_SEPARATOR_STRING, 1)
        if len(split) > 1:
            if not split[0] in dictionary:
                dictionary[split[0]] = {}
            self.insert_dict(dictionary[split[0]], split[1], value)
        else:
            dictionary[key] = value

    def get_node_params(self, node_name: str) -> dict:

        parser = argparse.ArgumentParser()
        self.add_arguments(parser, "None")
        args = parser.parse_args([node_name])

        absolute_node_name = get_absolute_node_name(args.node_name)
        with NodeStrategy(args) as node:
            if not wait_for_node(node, absolute_node_name, args.include_hidden_nodes, args.timeout):
                raise ValueError("node not found")

        node_name = parse_node_name(absolute_node_name)

        with DirectNode(args) as node:
            yaml_output = {node_name.full_name: {'ros__parameters': {}}}

            # retrieve parameter names
            print("call_list_parameters")
            list_parameters_response = call_list_parameters(node=node, node_name=absolute_node_name)
            if list_parameters_response is None:
                print(
                    'Wait for service timed out waiting for '
                    f'parameter services for node {node_name.full_name}', file=sys.stderr)
                raise RuntimeError("timed out waiting for parameter services")

            elif list_parameters_response.result() is None:
                e = list_parameters_response.exception()
                print(
                    'Exception while calling list_parameters service of node '
                    f"'{node_name.full_name}': {e}", file=sys.stderr)
                raise RuntimeError("exception while calling list_parameters service of node")

            parameter_names = sorted(list_parameters_response.result().result.names)

            # retrieve parameter values
            try:
                print("call_list_parameters")
                get_parameters_response = call_get_parameters(node=node, node_name=absolute_node_name, parameter_names=parameter_names)
            except RuntimeError as e:
                print(
                    'Exception while calling get_parameters service of node '
                    f"'{node_name.full_name}': {e}", file=sys.stderr)
                raise RuntimeError(f"exception while calling get_parameters service of node: {e}")

            if get_parameters_response.values is None:
                # pass through here, no parameters are available with this node.
                # since this is not failure, it proceeds to print the yaml as consistent behavior.
                pass

            parameter_values = [get_value(parameter_value=i) for i in get_parameters_response.values]

            # create dictionary with parameter names and values
            for param_name, pval in zip(parameter_names, parameter_values):
                self.insert_dict(yaml_output[node_name.full_name]['ros__parameters'], param_name, pval)

            return yaml_output


def main(args=None):

    d = NodeParamsGetter()
    params: dict = d.get_node_params(node_name="/local_costmap/local_costmap")
    print(yaml.dump(params, default_flow_style=False))
    file_path = os.path.expanduser("~/tmp/local_costmap_params.yaml")
    with open(file_path, 'w') as f:
        yaml.dump(params, f, default_flow_style=False)


if __name__ == '__main__':
    main()
