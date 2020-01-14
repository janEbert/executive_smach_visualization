import rospy


class StatusUpdater(object):
    """Callback for processing status messages for a connected viewer."""

    def __init__(self, viewer):
        self.viewer = viewer
        self.containers = viewer.containers

    def __call__(self, msg):
        self.status_msg_update(msg)

    def status_msg_update(self, msg):
        """Process the given status message."""
        # Do nothing if we're in the process of shutting down
        if not self.viewer.keep_running:
            return

        if self.viewer.auto_focus and msg.info:
            self.viewer.set_path(msg.info)
            self.viewer.set_max_depth(msg.info.count('/'))

        # Get path to the updating container
        path = msg.path
        rospy.logdebug('STATUS MSG: ' + path)

        # Check if this is a known container
        needs_update = False
        with self.viewer.update_cond:
            if path in self.containers:
                # Get the container and check if the status update
                # requires regeneration
                container = self.containers[path]
                if container.update_status(msg):
                    needs_update = True
            if needs_update:
                self.viewer.update_cond.notify_all()
