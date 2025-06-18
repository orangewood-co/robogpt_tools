import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters, GetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType


class ParameterWriter:
    def __init__(self, node=None):
        """Initialize the Robotiq gripper client."""
        
        if node is not None:
            self.node = node
        
    def set_remote_parameter(self, node_name, parameter_name, value, param_type):
        client = self.node.create_client(
            SetParameters,
            f'/{node_name}/set_parameters'
        )
        
        # Wait for service to be available
        if not client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().error(f'Service {node_name}/set_parameters not available')
            return False
            
        # Create parameter value based on type
        param_value = ParameterValue()
        
        if param_type == 'bool':
            param_value.type = ParameterType.PARAMETER_BOOL
            param_value.bool_value = value
        elif param_type == 'int':
            param_value.type = ParameterType.PARAMETER_INTEGER
            param_value.integer_value = value
        elif param_type == 'double':
            param_value.type = ParameterType.PARAMETER_DOUBLE
            param_value.double_value = value
        elif param_type == 'string':
            param_value.type = ParameterType.PARAMETER_STRING
            param_value.string_value = value
        
        # Create parameter
        parameter = Parameter()
        parameter.name = parameter_name
        parameter.value = param_value
        
        # Create request
        request = SetParameters.Request()
        request.parameters = [parameter]
        
        # Call service
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        
        # Process response
        if future.result() is not None:
            result = future.result().results[0]
            if result.successful:
                self.node.get_logger().info(f'Successfully set parameter {parameter_name}')
                return True
            else:
                self.node.get_logger().error(f'Failed to set parameter: {result.reason}')
                return False
        else:
            self.node.get_logger().error('Service call failed')
            return False

class ParameterReader:
    def __init__(self, node=None):
        """Initialize the parameter reader."""
        if node is not None:
            self.node = node
        
    def get_remote_parameter(self, node_name: str, param_name: str, param_type: str):
        """
        Get a single parameter value from a remote node.
        """
        client = self.node.create_client(
            GetParameters,
            f'/{node_name}/get_parameters'
        )
        
        # Wait for service to be available with longer timeout
        if not client.wait_for_service(timeout_sec=5.0):  # Increased timeout
            self.node.get_logger().error(f'Service {node_name}/get_parameters not available')
            return None
            
        # Create request for single parameter
        request = GetParameters.Request()
        request.names = [param_name]
        
        # Call service
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        
        # Process response
        if future.result() is not None:
            values = future.result().values
            if not values:  # Check if values list is empty
                self.node.get_logger().error(f'No values returned for parameter {param_name}')
                return None
                
            try:
                value = values[0]  # Now we know the list isn't empty
                
                # Convert value based on parameter type
                if param_type == 'bool':
                    return value.bool_value
                elif param_type == 'int':
                    return value.integer_value
                elif param_type == 'double':
                    return value.double_value
                elif param_type == 'string':
                    return value.string_value
                elif param_type == 'array':
                    return value.array_value
                else:
                    self.node.get_logger().error(f'Unsupported parameter type: {param_type}')
                    return None
            except AttributeError as e:
                self.node.get_logger().error(f'Parameter value does not have expected type {param_type}: {str(e)}')
                return None
        else:
            self.node.get_logger().error('Service call failed')
            return None


# def main(args=None):
#     rclpy.init(args=args)
#     node = Node('parameter_utility_node')
#     writer = ParameterWriter(node=node)
#     reader = ParameterReader(node=node)
    
#     # Example usage: set 'my_param' on 'other_node' to 'new_value'set_remote_parameter
#     success = writer.set_remote_parameter('configuration_setup', 'robot_name', 'ec63', 'string')
#     # bool_value = reader.get_remote_parameter('robogpt_agent', 'robot_name', 'string')

#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()