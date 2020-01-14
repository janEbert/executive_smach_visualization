import rospy

from container_node import ContainerNode


class StructureUpdater(object):
    """Callback for updating the structure of a connected viewer."""

    def __init__(self, viewer):
        self.viewer = viewer
        self.containers = viewer.containers

    def __call__(self, msg, server_name):
        self.structure_msg_update(msg, server_name)

    def structure_msg_update(self, msg, server_name):
        """Update the structure of the SMACH plan (re-generate the dotcode)."""
        # Do nothing if we're in the process of shutting down
        if not self.viewer.keep_running:
            return

        # Get the node path
        path = msg.path
        split_path = path.split('/')
        parent_path = '/'.join(split_path[0:-1])

        rospy.logdebug('STRUCTURE MSG: ' + path)
        rospy.logdebug('CONTAINERS: ' + str(self.containers.keys()))

        # Initialize redraw flag
        needs_redraw = False

        with self.viewer.update_cond:
            if path in self.containers:
                rospy.logdebug('UPDATING: ' + path)

                # Update the structure of this known container
                needs_redraw = self.containers[path].update_structure(msg)
            else:
                rospy.logdebug('CONSTRUCTING: ' + path)

                # Create a new container
                container = ContainerNode(server_name, msg)
                self.containers[path] = container

                # Store this as a root container if it has no parent
                if parent_path == '':
                    self.viewer.root_containers[path] = container

                # Append path to path models
                self.viewer.add_path(path)

                # Redraw the graph if this container's parent is already known
                if parent_path in self.containers:
                    needs_redraw = True

            # Update the graph if necessary
            if needs_redraw:
                self.viewer.structure_changed = True
                # TODO: Make it so you can disable this
                self.viewer.needs_zoom = True
                self.viewer.update_cond.notify_all()
