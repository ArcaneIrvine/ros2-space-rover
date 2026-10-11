import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
import threading
import curses
import time

# ─── ROS Node ────────────────────────────────────────────────────────────────

class RoverNode(Node):
    def __init__(self):
        super().__init__('rover_station')

        # Publishers
        self.mode_pub = self.create_publisher(String, '/rover_mode', 10)
        self.cmd_pub  = self.create_publisher(Twist,  '/cmd_vel_raw', 10)

        # State
        self.safety_status  = 'WAITING...'
        self.mission_status = 'WAITING...'
        self.distance       = 0.0
        self.mode           = 'manual'

        # Subscribers
        self.create_subscription(String, '/safety_status',  self.on_safety,  10)
        self.create_subscription(String, '/mission_status', self.on_mission, 10)
        self.create_subscription(Range,  '/ultrasonic/range', self.on_range, 10)

    def on_safety(self, msg):
        self.safety_status = msg.data

    def on_mission(self, msg):
        self.mission_status = msg.data

    def on_range(self, msg):
        self.distance = msg.range

    def set_mode(self, mode):
        self.mode = mode
        msg = String()
        msg.data = mode
        self.mode_pub.publish(msg)

    def send_cmd(self, linear, angular):
        msg = Twist()
        msg.linear.x  = linear
        msg.angular.z = angular
        self.cmd_pub.publish(msg)

    def stop(self):
        self.send_cmd(0.0, 0.0)


# ─── Terminal UI ─────────────────────────────────────────────────────────────

def draw(stdscr, rover):
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.timeout(100)

    # Colors
    curses.start_color()
    curses.init_pair(1, curses.COLOR_GREEN,  curses.COLOR_BLACK)  # green
    curses.init_pair(2, curses.COLOR_CYAN,   curses.COLOR_BLACK)  # cyan
    curses.init_pair(3, curses.COLOR_YELLOW, curses.COLOR_BLACK)  # yellow
    curses.init_pair(4, curses.COLOR_RED,    curses.COLOR_BLACK)  # red
    curses.init_pair(5, curses.COLOR_WHITE,  curses.COLOR_BLACK)  # white
    curses.init_pair(6, curses.COLOR_BLACK,  curses.COLOR_GREEN)  # highlight

    GREEN  = curses.color_pair(1)
    CYAN   = curses.color_pair(2)
    YELLOW = curses.color_pair(3)
    RED    = curses.color_pair(4)
    WHITE  = curses.color_pair(5)
    BOLD   = curses.A_BOLD

    speed   = 0.5
    running = True

    # Latched movement state — toggled on/off by key press, not held-key based
    current_linear  = 0.0
    current_angular = 0.0

    while running:
        stdscr.erase()
        h, w = stdscr.getmaxyx()

        # ── Border and title ──
        stdscr.attron(GREEN | BOLD)
        stdscr.border()
        title = '  ROVER CONTROL STATION  '
        stdscr.addstr(0, (w - len(title)) // 2, title)
        stdscr.attroff(GREEN | BOLD)

        # ── Divider column ──
        mid = 30
        for y in range(1, h - 1):
            stdscr.addch(y, mid, '│', GREEN)
        stdscr.addch(0,     mid, '┬', GREEN)
        stdscr.addch(h - 1, mid, '┴', GREEN)

        # ── LEFT PANEL ──

        # Mode section
        stdscr.addstr(2, 2, 'MODE', CYAN | BOLD)
        stdscr.addch(3, 1, '├', GREEN)
        stdscr.addstr(3, 2, '─' * (mid - 3), GREEN)

        manual_style = (curses.color_pair(6) | BOLD) if rover.mode == 'manual' else WHITE
        auto_style   = (curses.color_pair(6) | BOLD) if rover.mode == 'autonomous' else WHITE

        stdscr.addstr(4, 3, '[M]  Manual',     manual_style)
        stdscr.addstr(5, 3, '[A]  Autonomous', auto_style)

        # Controls section
        stdscr.addstr(7, 2, 'CONTROLS', CYAN | BOLD)
        stdscr.addch(8, 1, '├', GREEN)
        stdscr.addstr(8, 2, '─' * (mid - 3), GREEN)

        controls = [
            ('↑', 'Forward (toggle)'),
            ('↓', 'Backward (toggle)'),
            ('←', 'Turn Left (toggle)'),
            ('→', 'Turn Right (toggle)'),
            ('SPC', 'Stop'),
            ('Q', 'Quit'),
        ]
        for i, (key, desc) in enumerate(controls):
            stdscr.addstr(9 + i, 3, f'{key:<5}', YELLOW | BOLD)
            stdscr.addstr(9 + i, 8, desc, WHITE)

        # Speed
        stdscr.addstr(16, 2, 'SPEED', CYAN | BOLD)
        stdscr.addch(17, 1, '├', GREEN)
        stdscr.addstr(17, 2, '─' * (mid - 3), GREEN)
        bar_len = mid - 6
        filled  = int(speed * bar_len)
        bar     = '█' * filled + '░' * (bar_len - filled)
        stdscr.addstr(18, 3, f'{bar} {int(speed*100):3d}%', GREEN)
        stdscr.addstr(19, 3, '[+] faster   [-] slower', WHITE)

        # Movement state indicator
        stdscr.addstr(21, 2, 'MOVEMENT', CYAN | BOLD)
        stdscr.addch(22, 1, '├', GREEN)
        stdscr.addstr(22, 2, '─' * (mid - 3), GREEN)
        if current_linear > 0:
            move_text = '▲ FORWARD'
        elif current_linear < 0:
            move_text = '▼ BACKWARD'
        elif current_angular > 0:
            move_text = '◄ TURNING LEFT'
        elif current_angular < 0:
            move_text = '► TURNING RIGHT'
        else:
            move_text = '■ STOPPED'
        stdscr.addstr(23, 3, move_text, GREEN | BOLD)

        # ── RIGHT PANEL ──
        col = mid + 2

        # Status section
        stdscr.addstr(2, col, 'STATUS', CYAN | BOLD)
        stdscr.addch(3, mid, '├', GREEN)
        stdscr.addstr(3, mid + 1, '─' * (w - mid - 2), GREEN)

        safety  = rover.safety_status
        s_color = GREEN if safety == 'OK' else RED
        stdscr.addstr(4, col, 'Safety:  ', WHITE)
        stdscr.addstr(4, col + 9, safety, s_color | BOLD)

        stdscr.addstr(5, col, f'Distance: {rover.distance:.2f}m', WHITE)

        mission = rover.mission_status
        stdscr.addstr(6, col, 'Mission: ', WHITE)
        stdscr.addstr(7, col, mission[:w - mid - 3], YELLOW)

        # ── Bottom bar ──
        mode_text  = f'  ● {"MANUAL" if rover.mode == "manual" else "AUTONOMOUS"} MODE  '
        mode_color = GREEN if rover.mode == 'manual' else CYAN
        stdscr.addstr(h - 1, (w - len(mode_text)) // 2, mode_text, mode_color | BOLD)

        stdscr.refresh()

        # ── Input handling ──
        try:
            key = stdscr.getch()
        except Exception:
            key = -1

        if key == ord('q') or key == ord('Q'):
            running = False

        elif key == ord('m') or key == ord('M'):
            rover.set_mode('manual')

        elif key == ord('a') or key == ord('A'):
            rover.set_mode('autonomous')

        elif key == ord('+') or key == ord('='):
            speed = min(1.0, round(speed + 0.1, 1))

        elif key == ord('-'):
            speed = max(0.1, round(speed - 0.1, 1))

        elif key == curses.KEY_UP:
            if current_linear > 0:
                current_linear = 0.0
            else:
                current_linear  = speed
                current_angular = 0.0

        elif key == curses.KEY_DOWN:
            if current_linear < 0:
                current_linear = 0.0
            else:
                current_linear  = -speed
                current_angular = 0.0

        elif key == curses.KEY_LEFT:
            if current_angular > 0:
                current_angular = 0.0
            else:
                current_angular = speed
                current_linear  = 0.0

        elif key == curses.KEY_RIGHT:
            if current_angular < 0:
                current_angular = 0.0
            else:
                current_angular = -speed
                current_linear  = 0.0

        elif key == ord(' '):
            current_linear  = 0.0
            current_angular = 0.0

        # Keep speed changes applied to an active latch
        if current_linear > 0:
            current_linear = speed
        elif current_linear < 0:
            current_linear = -speed
        if current_angular > 0:
            current_angular = speed
        elif current_angular < 0:
            current_angular = -speed

        if rover.mode == 'manual':
            rover.send_cmd(current_linear, current_angular)

    rover.stop()
    rover.set_mode('manual')


# ─── Main ─────────────────────────────────────────────────────────────────────

def main():
    rclpy.init()
    rover = RoverNode()

    ros_thread = threading.Thread(
        target=rclpy.spin, args=(rover,), daemon=True)
    ros_thread.start()

    try:
        curses.wrapper(draw, rover)
    finally:
        rover.stop()
        rover.set_mode('manual')
        rover.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
