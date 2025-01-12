#!/usr/bin/env python3
# ====================================================================================================================================
# @file       bus_state_getter.py
# @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
# @date       Thursday, 28th April 2022 1:05:25 pm
# @modified   Friday, 1st July 2022 5:45:23 pm
# @project    engineering-thesis
# @brief      Auxiliary ROS2 CLI utility requesting the running EtherCAT driver node for reading bus state
# @details    Usage:
#
#      ros2 run velmwheel_ethercat_driver bus_state_getter                         \
#          <fully_qualified_driver_node_name>                                      \
#             --ros-args                                                           \
#             -p service_wait_timeout:=<timeout_of_service_registration:default=1.0>
#
# @copyright Krzysztof Pierczyk © 2022
# ====================================================================================================================================

# ============================================================= Imports ============================================================ #

# System imports
import sys
# Private imports
from velmwheel_ethercat_driver_msgs.srv import GetBusState
from velmwheel_ethercat_driver.common import GET_BUS_STATE_SERVICE_NAME
from package_common_py.requester import Requester

# ============================================================== Nodes ============================================================= #

class BusStateGetter():

    """Node class implementing bus-state-get request"""

    def create_request(self, node):

        """Creates service request"""
        return GetBusState.Request()


    def handle_response(self, node, response):

        """Handles response"""

        # If service failed, print log
        if not response.success:
            node.get_logger().error(f'Failed to get bus state ({response.error_message})')
        # Else, print bus state
        elif response.state == GetBusState.Response.STOPPED:
            node.get_logger().info( f"Current bus state: 'stopped'")
        elif response.state == GetBusState.Response.RUNNING:
            node.get_logger().info( f"Current bus state: 'running'")
        else:
            node.get_logger().info( f"Invalid bus state read ({response.state})")

# ============================================================== Main ============================================================== #

def main():

    # Create requester
    requester = Requester(
        node_name   = 'bus_state_getter',
        srv_type    = GetBusState,
        topic_name  = f'{sys.argv[1]}/{GET_BUS_STATE_SERVICE_NAME}',
        srv_handler =  BusStateGetter()
    )

    # Run request
    requester.run()

# ================================================================================================================================== #

if __name__ == '__main__':
    main()
