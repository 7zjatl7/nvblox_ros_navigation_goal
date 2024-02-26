import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from .obstacle_map import GridMap
from .goal_generators import GoalReader
from .goal_generators.event_handler import EventHandler
import sys
from geometry_msgs.msg import PoseWithCovarianceStamped
import time
import os

from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler

current_file_path = os.path.abspath(__file__)
current_file_path = os.path.dirname(current_file_path)
root_path = os.path.dirname(current_file_path)


class SetNavigationGoal(Node):
    def __init__(self):
        super().__init__("set_navigation_goal")

        self.declare_parameters(
            namespace="",
            parameters=[
                ("initial_pose", rclpy.Parameter.Type.DOUBLE_ARRAY),
                ("map_yaml_path", rclpy.Parameter.Type.STRING),
                ("frame_id", rclpy.Parameter.Type.STRING),
                ("action_server_name", "navigate_to_pose"),
                ("obstacle_search_distance_in_meters", 0.2),
                ("iteration_count", 1)
            ],
        )
               
        # goal 생성
        action_server_name = self.get_parameter("action_server_name").value
        # Action Client 생성  
        self._action_client = ActionClient(self, NavigateToPose, action_server_name)

        self.MAX_ITERATION_COUNT = self.get_parameter("iteration_count").value
        assert self.MAX_ITERATION_COUNT > 0
        self.curr_iteration_count = 0

        self.start_line = 0

        self.__grid_map = None
        self.__distance = None

        self.__goal_text_file_path = os.path.join(root_path, 'maps', 'goals.txt')

        self.__initial_goal_publisher = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 1)

        self.__initial_pose = self.get_parameter("initial_pose").value
        self.__is_initial_pose_sent = True if self.__initial_pose is None else False


    def __send_initial_pose(self): # 2 
        """
        Publishes the initial pose.
        This function is only called once that too before sending any goal pose
        to the mission server.
        """
        goal = PoseWithCovarianceStamped()
        goal.header.frame_id = self.get_parameter("frame_id").value
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = self.__initial_pose[0]
        goal.pose.pose.position.y = self.__initial_pose[1]
        goal.pose.pose.position.z = self.__initial_pose[2]
        goal.pose.pose.orientation.x = self.__initial_pose[3]
        goal.pose.pose.orientation.y = self.__initial_pose[4]
        goal.pose.pose.orientation.z = self.__initial_pose[5]
        goal.pose.pose.orientation.w = self.__initial_pose[6]
        self.__initial_goal_publisher.publish(goal)

    def send_goal(self):
        """
        Sends the goal to the action server.
        """
        if not self.__is_initial_pose_sent:
            self.get_logger().info("Sending initial pose")
            self.__send_initial_pose()
            self.__is_initial_pose_sent = True

            # Assumption is that initial pose is set after publishing first time in this duration.
            # Can be changed to more sophisticated way. e.g. /particlecloud topic has no msg until
            # the initial pose is set.
            time.sleep(1)
            self.get_logger().info("Sending first goal")
        # action client 연결 
        self._action_client.wait_for_server()

        # txt 파일에서 goal 데이터 받아오기
        goal_msg = self.__get_goal()

        if goal_msg is None:
            self._send_goal_future.add_done_callback(self.__goal_response_callback)

            rclpy.shutdown()
            sys.exit(1)

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.__feedback_callback
        )
        self._send_goal_future.add_done_callback(self.__goal_response_callback)

    def __goal_response_callback(self, future): # 5
        """
        Callback function to check the response(goal accpted/rejected) from the server.\n
        If the Goal is rejected it stops the execution for now.(We can change to resample the pose if rejected.)
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected :(")
            rclpy.shutdown()
            return

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.__get_result_callback)

    def __get_goal(self): # 3

        """
        Get the next goal from the goal generator.

        Returns
        -------
        [NavigateToPose][goal] or None if the next goal couldn't be generated.

        """
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = self.get_parameter("frame_id").value
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        yaml_file_path = self.get_parameter("map_yaml_path").value
        grid_map = GridMap(yaml_file_path)
        obstacle_search_distance_in_meters = self.get_parameter("obstacle_search_distance_in_meters").value
        assert obstacle_search_distance_in_meters > 0

        self.__grid_map = grid_map
        self.__distance = obstacle_search_distance_in_meters
        goal_pose_line = self.__read_goal_txt(self.__goal_text_file_path)
        if goal_pose_line is not None:
            goal_pose = goal_pose_line.split()
            range_ = self.__grid_map.get_range()

            x, y = float(goal_pose[0]), float(goal_pose[1])
            orient_x, orient_y, orient_z, orient_w = float(goal_pose[2]), float(goal_pose[3]), float(goal_pose[4]), float(goal_pose[5])

            if  x > range_[0][0] and x > range_[0][1] and y > range_[1][0] and y > range_[1][1]:
                x = range_[0][0]
                y = range_[1][0]

            if self.__grid_map.is_valid_pose([x, y], self.__distance):
                pose = [x, y, orient_x, orient_y, orient_z, orient_w]

            # couldn't sample a pose which is not close to obstacles. Rare but might happen in dense maps.
            if pose is None:
                self.get_logger().error(
                    "Could not generate next goal. Returning. Possible reasons for this error could be:"
                )
                self.get_logger().error(
                    "1. If you are using GoalReader then please make sure iteration count <= number of goals avaiable in file."
                )
                self.get_logger().error(
                    "2. If RandomGoalGenerator is being used then it was not able to sample a pose which is given distance away from the obstacles."
                )
                return

            self.get_logger().info("Generated goal pose: {0}".format(pose))
            goal_msg.pose.pose.position.x = pose[0]
            goal_msg.pose.pose.position.y = pose[1]
            goal_msg.pose.pose.orientation.x = pose[2]
            goal_msg.pose.pose.orientation.y = pose[3]
            goal_msg.pose.pose.orientation.z = pose[4]
            goal_msg.pose.pose.orientation.w = pose[5]
            return goal_msg
        
        else:
            return goal_msg



    def __get_result_callback(self, future): # 6
        """
        Callback to check result.\n
        It calls the send_goal() function in case current goal sent count < required goals count.     
        """
        # Nav2 is sending empty message for success as well as for failure.
        result = future.result().result
        self.get_logger().info("Result: {0}".format(result.result))

        if self.curr_iteration_count < self.MAX_ITERATION_COUNT:
            self.curr_iteration_count += 1
            self.send_goal()
        else:

            with open(self.__goal_text_file_path, 'r', encoding='utf-8') as file:
                LINE_COUNT = len(list(file))
            if LINE_COUNT > self.MAX_ITERATION_COUNT:
                self.MAX_ITERATION_COUNT = LINE_COUNT
                if self.curr_iteration_count < self.MAX_ITERATION_COUNT:
                    self.curr_iteration_count += 1
                    self.send_goal()
                    
            else:
                goal_path = '/workspaces/isaac_ros-dev/src/isaac_ros_nvblox/nvblox_ros_navigation_goal/maps/goals.txt'
                # goal 데이터가 추가되었을때 이벤트 핸들러와 옵저버 생성
                self.__handler = EventHandler()
                self.__observer = Observer()
                self.__observer.schedule(self.__handler, path=goal_path, recursive=False)
                self.__observer.start()

                try:
                    while not self.__handler.file_modified:
                        time.sleep(1)
                        self.get_logger().info('Additionally, waiting pose...')

                except KeyboardInterrupt:
                    self.__observer.stop()
                
                self.__observer.stop()
                self.__observer.join()

                with open(self.__goal_text_file_path, 'r', encoding='utf-8') as file:
                    LINE_COUNT = len(list(file))
                
                if LINE_COUNT > self.MAX_ITERATION_COUNT:
                    self.MAX_ITERATION_COUNT = LINE_COUNT
                    self.curr_iteration_count += 1
                    self.send_goal()


    def __feedback_callback(self, feedback_msg): # 4

        """
        This is feeback callback. We can compare/compute/log while the robot is on its way to goal.
        """
        # self.get_logger().info('FEEDBACK: {}\n'.format(feedback_msg))
        pass

    def __read_goal_txt(self, goal_file_path):

        with open(goal_file_path, 'r', encoding='utf-8') as file:
            for current_line, line in enumerate(file, 1):
                if current_line >= self.start_line:
                    self.start_line += 1
                    return line.strip()
                    

def main():
    rclpy.init()
    set_goal = SetNavigationGoal()
    result = set_goal.send_goal()
    rclpy.spin(set_goal)


if __name__ == "__main__":
    main()
