import sys
from std_msgs.msg import String
import rclpy
import termios
import tty



def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    # sys.stdin.read() returns a string on Linux
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def saveTerminalSettings():
    return termios.tcgetattr(sys.stdin)


def restoreTerminalSettings(old_settings):
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def main():
    settings = saveTerminalSettings()

    rclpy.init()

    node = rclpy.create_node('pico_keyboard')
    pub = node.create_publisher(String, '/cmd_vel', 10)
    print(f"Press keys for actions\n'a' to engage left magnet (LMB),\n's' to disengage left magnet (LM_B),\n'd' to engage right magnet (RM_B),\n'f' to disengage right magnet (RM_B),\n'l' to move left motor (LM_A),\n'r' to move right motor (RM_A),\n'q' to quit,\nany other alpha-numeric key to stop the motors")

    string = String()

    try:
        while True:
            key = getKey(settings)
            string.data = key
            if key == "q":
                break
            pub.publish(string)
    except Exception as e:
        print(e)
    finally:
        string.data = "k"
        pub.publish(string)

        restoreTerminalSettings(settings)


if __name__ == '__main__':
    main()