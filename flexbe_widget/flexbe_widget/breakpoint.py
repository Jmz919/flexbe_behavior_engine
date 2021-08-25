import rclpy
import sys

def main():
  node = rclpy.create_node('flexbe_breakpoint')
  node.declare_parameter('/flexbe/breakpoints', sys.argv[1:])

if __name__ == '__main__':
  main()
