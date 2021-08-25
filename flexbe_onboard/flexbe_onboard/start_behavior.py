import rclpy

from flexbe_core.proxy import ProxySubscriberCached
from flexbe_onboard.flexbe_onboard import FlexbeOnboard

def main():
  node = rclpy.create_node('flexbe_onboard')

  FlexbeOnboard(node)

  # Wait for ctrl-c to stop the application
  rclpy.spin()

  ProxySubscriberCached().shutdown()


if __name__ == '__main__':
    main()
