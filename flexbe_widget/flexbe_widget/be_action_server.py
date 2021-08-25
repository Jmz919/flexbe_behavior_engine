import rclpy

from flexbe_widget.behavior_action_server import BehaviorActionServer

def main():
  rclpy.init()
  node = rclpy.create_node('flexbe_action_server')

  server = BehaviorActionServer(node)

  # Wait for ctrl-c to stop the application
  rclpy.spin()

if __name__ == '__main__':
    main()
