#!/usr/bin/env python

import sys
# Add rospkg path
sys.path.append('/vol/famula/nightly/lib/python3.6/site-packages')

from gui.gtk_wrap import Gtk
from gui.main_window import MainWindow
import rospy


def main():
    # Old viewer:
    width = 720
    height = 480

    win = MainWindow(width, height)
    win.connect('delete-event', Gtk.main_quit)

    if sys.platform != 'win32':
        # Reset KeyboardInterrupt SIGINT handler,
        # so that glib loop can be stopped by it
        import signal
        signal.signal(signal.SIGINT, signal.SIG_DFL)

    Gtk.main()


if __name__ == '__main__':
    rospy.init_node('smach_viewer',
                    anonymous=False,
                    disable_signals=True,
                    log_level=rospy.INFO)
    sys.argv = rospy.myargv()
    main()
