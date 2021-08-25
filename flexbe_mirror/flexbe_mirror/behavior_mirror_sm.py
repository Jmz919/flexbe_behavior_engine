import rclpy

from flexbe_core.proxy import ProxySubscriberCached

from flexbe_mirror.flexbe_mirror import FlexbeMirror

def main():
  node = rclpy.create_node('flexbe_mirror')

  FlexbeMirror(node)
  # Wait for ctrl-c to stop the application
  rclpy.spin()

  ProxySubscriberCached().shutdown()

if __name__ == '__main__':
    main()
