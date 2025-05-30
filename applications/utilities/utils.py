#!/usr/bin/python3

import os
import rclpy
import zipfile
from rclpy.executors import SingleThreadedExecutor
from rcl_interfaces.srv import GetParameters
from robogpt_startup.msg import Config
from rclpy.executors import SingleThreadedExecutor


def extract_xml_from_zip(zip_path, extract_to_folder):
    """
    Extracts XML files from a given zip file into a subfolder named after the zip file.
    Saves their paths in a list.

    :param zip_path: Path to the zip file.
    :param extract_to_folder: Base folder where the files will be extracted.
    :return: A list of paths to the extracted XML files.
    """
    xml_paths = []  # To store paths of extracted XML files

    zip_file_name = os.path.basename(zip_path)  # Get the name of the zip file
    zip_file_name_without_ext = os.path.splitext(zip_file_name)[0]  # Remove the extension
    # Full path for the new subdirectory
    full_extract_path = os.path.join(extract_to_folder, zip_file_name_without_ext)

    # Create the subdirectory if it doesn't exist
    if not os.path.exists(full_extract_path):
        os.makedirs(full_extract_path)

    with zipfile.ZipFile(zip_path, 'r') as zip_ref:
        # Extract all files in the zip to the subdirectory
        zip_ref.extractall(full_extract_path)
        # Loop through the file names
        for file_name in zip_ref.namelist():
            # Check if the file is an XML
            if file_name.endswith('.xml'):
                # Save the full path of the extracted XML file
                xml_paths.append(os.path.join(full_extract_path, file_name))

    print("All the program files extracted")

    return xml_paths


def is_robot_connected(ip: str) -> bool:
    """
    Check if the robot is connected via Ethernet by pinging its IP address.

    Args:
        ip (str): IP address of the robot.

    Returns:
        bool: True if the robot is connected, False otherwise.
    """
    # Ping the IP address with 1 packet and wait for a response
    command = f"ping -c 1 -W 1 {ip} > /dev/null 2>&1"
    # Execute the command
    response = os.system(command)
    # Return True if the ping was successful (exit code 0), False otherwise
    return response == 0

def get_remote_parameter(node_name: str, param_name: str, param_type: str, timeout_sec=5.0):
    """
    Reads a single parameter from a remote node.
    
    Args:
        node_name (str): Name of the node that has the parameter
        param_name (str): Name of the parameter to read
        param_type (str): Type of parameter ('bool', 'int', 'double', 'string', 'array')
        timeout_sec (float, optional): Timeout in seconds. Default is 5.0 seconds.
        
    Returns:
        The parameter value, or None if not found or timeout occurred
    """
    # Initialize a context and node just for this call
    context = rclpy.Context()
    context.init()
    
    # Create a temporary node with a unique name
    tmp_node = rclpy.create_node(
        f'param_reader_{id(context)}',
        context=context
    )
    
    # Create result container
    result = {'value': None, 'received': False, 'error': None}
    
    # Create service client
    client = tmp_node.create_client(
        GetParameters,
        f'/{node_name}/get_parameters'
    )
    
    # Set up executor
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(tmp_node)
    
    # Wait for service to be available
    start_time = tmp_node.get_clock().now()
    while not client.wait_for_service(timeout_sec=0.1):  # Small timeout to check conditions frequently
        executor.spin_once(timeout_sec=0.1)
        
        # Check for timeout
        current_time = tmp_node.get_clock().now()
        elapsed = (current_time - start_time).nanoseconds / 1e9
        if elapsed > timeout_sec:
            result['error'] = f'Service {node_name}/get_parameters not available after {timeout_sec} seconds'
            break
    
    # If service is available, make the request
    if result['error'] is None:
        # Create request
        request = GetParameters.Request()
        request.names = [param_name]
        
        # Define callback for when the future completes
        def future_callback(future):
            if future.result() is not None:
                values = future.result().values
                if not values:
                    result['error'] = f'No values returned for parameter {param_name}'
                    result['received'] = True
                    return
                
                try:
                    value = values[0]
                    
                    # Convert value based on parameter type
                    if param_type == 'bool':
                        result['value'] = value.bool_value
                    elif param_type == 'int':
                        result['value'] = value.integer_value
                    elif param_type == 'double':
                        result['value'] = value.double_value
                    elif param_type == 'string':
                        result['value'] = value.string_value
                    elif param_type == 'array':
                        result['value'] = value.array_value
                    else:
                        result['error'] = f'Unsupported parameter type: {param_type}'
                    
                    result['received'] = True
                except AttributeError as e:
                    result['error'] = f'Parameter value does not have expected type {param_type}: {str(e)}'
                    result['received'] = True
            else:
                result['error'] = 'Service call failed'
                result['received'] = True
        
        # Call service asynchronously
        future = client.call_async(request)
        future.add_done_callback(future_callback)
        
        # Spin until we get a result or timeout
        call_start_time = tmp_node.get_clock().now()
        while not result['received']:
            executor.spin_once(timeout_sec=0.1)
            
            # Check for timeout
            current_time = tmp_node.get_clock().now()
            elapsed = (current_time - call_start_time).nanoseconds / 1e9
            if elapsed > timeout_sec:
                result['error'] = f'Timeout waiting for parameter response after {timeout_sec} seconds'
                break
    
    # Clean up
    executor.remove_node(tmp_node)
    tmp_node.destroy_node()
    context.shutdown()
    
    # Return result
    if result['error']:
        print(f"Error: {result['error']}")
        return None
    return result['value']



def get_single_message(topic_name='robot_config', message_type=Config, timeout_sec=None):
    """
    Waits for and returns a single message from the specified topic.
    
    Args:
        topic_name (str): Name of the topic to subscribe to
        message_type: The message type class to use
        timeout_sec (float, optional): Timeout in seconds. None means wait forever.
        
    Returns:
        The received message, or None if timeout occurred
    """
    # Initialize a context and node just for this call
    context = rclpy.Context()
    context.init()
    
    # Create a temporary node with a unique name
    tmp_node = rclpy.create_node(
        f'single_message_reader_{id(context)}',
        context=context
    )
    
    # Create a container for our result
    result = {'message': None, 'received': False}
    
    # Define the callback
    def callback(msg):
        result['message'] = msg
        result['received'] = True
    
    # Create subscription
    subscription = tmp_node.create_subscription(
        message_type,
        topic_name,
        callback,
        10  # QoS profile depth
    )
    
    # Set up executor
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(tmp_node)
    
    # Spin until we get a message or timeout
    start_time = tmp_node.get_clock().now()
    while not result['received']:
        executor.spin_once(timeout_sec=0.1)  # Small timeout to check conditions frequently
        
        # Check for timeout if specified
        if timeout_sec is not None:
            current_time = tmp_node.get_clock().now()
            elapsed = (current_time - start_time).nanoseconds / 1e9
            if elapsed > timeout_sec:
                break
    
    # Clean up
    executor.remove_node(tmp_node)
    tmp_node.destroy_node()
    context.shutdown()
    
    return result['message']