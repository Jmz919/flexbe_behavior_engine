import rclpy
import time
import sys
import getopt

from flexbe_widget.behavior_launcher import BehaviorLauncher
from flexbe_msgs.msg import BehaviorRequest


def usage():
    print("Soon...")

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('flexbe_widget')

    try:
      opts, args = getopt.getopt(sys.argv[1:], "hb:a:", ["help", "behavior=", "autonomy="])
    except getopt.GetoptError:
      usage()
      sys.exit(2)

    behavior = ""
    autonomy = 255

    for opt, arg in opts:
      if opt in ("-h", "--help"):
          usage()
          sys.exit()
      elif opt in ("-b", "--behavior"):
          behavior = arg
      elif opt in ("-a", "--autonomy"):
          autonomy = int(arg)
    ignore_args = ['__name', '__log']  # auto-set by roslaunch

    launcher = BehaviorLauncher(node)

    if behavior != "":
      request = BehaviorRequest()
      request.behavior_name = behavior
      request.autonomy_level = autonomy
      for arg in args:
          if ':=' not in arg:
              continue
          k, v = arg.split(':=', 1)
          if k in ignore_args:
              continue
          request.arg_keys.append('/'+k)
          request.arg_values.append(v)
      time.sleep(0.2)  # wait for publishers...
      launcher._callback(request)

    print("About to spin be_launcher")
    # Wait for ctrl-c to stop the application
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
