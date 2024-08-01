# Copyright 2021 Open Source Robotics Foundation, Inc.
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


'''
    The RobotAPI class is a wrapper for API calls to the robot. Here users
    are expected to fill up the implementations of functions which will be used
    by the RobotCommandHandle. For example, if your robot has a REST API, you
    will need to make http request calls to the appropriate endpoints within
    these functions.
'''
import requests
from urllib.error import HTTPError

class RobotAPI:
    # The constructor below accepts parameters typically required to submit
    # http requests. Users should modify the constructor as per the
    # requirements of their robot's API
    def __init__(self, prefix: str, user: str, password: str):
        self.prefix = prefix
        self.user = user
        self.password = password
        self.connected = False
        self.timeout = 5.0
        # Test connectivity
        connected = self.check_connection()
        if connected:
            # print("Successfully able to query API server")
            self.connected = True
        else:
            print("Unable to query API server")

    def check_connection(self):
        ''' Return True if connection to the robot API server is successful'''
        if self.data() is None:
            return False
        return True


    def position(self, robot_name: str):
        ''' Return [x, y, theta] expressed in the robot's coordinate frame or
            None if any errors are encountered'''
        response = self.data(robot_name)
        if response is not None:
            if not response['success']:
                print(f'Response for {robot_name} was not successful')
                return None

            position = response['data']['position']
            if position is None:
                print(f'No position data received for {robot_name}')
                return None
            x = position['x']
            y = position['y']
            angle = position['yaw']

            return [x, y, angle]

        print(f'No response received for {robot_name}')
        return None

    def navigate(self, robot_name: str, pose, map_name: str):
        ''' Request the robot to navigate to pose:[x,y,theta] where x, y and
            and theta are in the robot's coordinate convention. This function
            should return True if the robot has accepted the request,
            else False'''
        url = self.prefix+f'/open-rmf/rmf-pinklab/navigate/{robot_name}'
        data = {
            "x": pose[0],
            "y": pose[1],
            "yaw": pose[2]
        }
        # # print(f"Sending navigation request to {url} with data: {data}")
        try:
            response = requests.post(url, json=data, timeout=self.timeout)
            if response.status_code == 200:
                if response.json()['success']:
                    # # print("Navigation request successful")
                    return True
                elif response.json()['success'] == False:
                    # # print("Robot is not ready for navigation")
                    return False
            else:
                # # print(f"Navigation request failed: {response.text}")
                return False
        except Exception as e:
            # # print(f"Error sending navigation request: {e}")
            return False

    def start_process(self, robot_name: str, process: str, map_name: str):
        ''' Request the robot to begin a process. This is specific to the robot
            and the use case. For example, load/unload a cart for Deliverybot
            or begin cleaning a zone for a cleaning robot.
            Return True if the robot has accepted the request, else False'''
        # print(f"start_process: {robot_name}, {process}, {map_name}")
        # ------------------------ #
        # IMPLEMENT YOUR CODE HERE #
        # ------------------------ #
        return False

    def stop(self, robot_name: str):
        ''' Command the robot to stop.
            Return True if robot has successfully stopped. Else False'''
        # print(f"stop: {robot_name}")
        try: 
            response = requests.post(f"{self.prefix}/open-rmf/rmf-pinklab/stop/{robot_name}", timeout=self.timeout)
            if response.status_code == 200:
                if response.json()['success']:
                    return True
                elif response.json()['success'] == False:
                    return False
            return False
        except Exception as e:
            # # print(f"Error while getting robot data: {e}")
            return False

    def navigation_remaining_duration(self, robot_name: str):
        ''' Return the number of seconds remaining for the robot to reach its
            destination'''
        response = self.data(robot_name)
        if response is not None:
            if not response['success']:
                print(f'Response for {robot_name} was not successful')
                return None

            remaining_duration = response['data']['destination_arrival']['duration']
            if remaining_duration is None:
                print(f'No remaining_duration data received for {robot_name}')
                return None

            return remaining_duration

    def navigation_completed(self, robot_name: str):
        ''' Return True if the robot has successfully completed its previous
            navigation request. Else False.'''
        try: 
            response = requests.post(f"{self.prefix}/open-rmf/rmf-pinklab/command_completed/{robot_name}", timeout=self.timeout)
            if response.status_code == 200:
                if response.json()['success']:
                    # # # print("Command completed")
                    return True
                elif response.json()['success'] == False:
                    # # # print("Robot has not reached the destination")
                    return False
            return False
        except Exception as e:
            # # print(f"Error while getting robot data: {e}")
            return False


    def process_completed(self, robot_name: str):
        ''' Return True if the robot has successfully completed its previous
            process request. Else False.'''
        # print(f"process_completed: {robot_name}")
        return False

    def battery_soc(self, robot_name: str):
        ''' Return the state of charge of the robot as a value between 0.0
            and 1.0. Else return None if any errors are encountered'''
        response = self.data(robot_name)
        if response is not None:
            return response['data']['battery']/100.0
        else:
            return None

    def data(self, robot_name=None):
        if robot_name is None:
            url = self.prefix + f'/open-rmf/rmf-pinklab/status/'
        else:
            url = self.prefix +\
                f'/open-rmf/rmf-pinklab/status/{robot_name}'
        try:
            response = requests.get(url, timeout=self.timeout)
            response.raise_for_status()
            if response is not None:
                return response.json()
            else:
                return None
        except HTTPError as http_err:
            print(f'HTTP error: {http_err}')
        except Exception as err:
            print(f'Other error: {err}')
        return None