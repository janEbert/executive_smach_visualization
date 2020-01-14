from __future__ import print_function
from threading import Thread

import rospy
from smach_msgs.msg import SmachContainerStatus
from smach_msgs.msg import SmachContainerStructure
import smach_ros

from .structure_updater import StructureUpdater
from .status_updater import StatusUpdater


class SubscriptionManager(object):
    """
    A handler serving as the connection between a viewer, its updaters and an
    introspection client.
    Periodically updates the list of servers and subscribes to new ones.
    """

    def __init__(self, viewer):
        self.viewer = viewer
        self.structure_updater = StructureUpdater(viewer)
        self.status_updater = StatusUpdater(viewer)

        self.structure_subs = {}
        self.status_subs = {}

        self.client = smach_ros.IntrospectionClient()
        self.update_servers_thread = Thread(target=self.update_server_loop)
        self.update_servers_thread.start()

    def __del__(self):
        self.update_servers_thread.join()
        super(SubscriptionManager, self).__del__(self)

    def update_server_loop(self):
        """Periodically update the list of SMACH introspection servers."""
        while self.viewer.keep_running:
            server_names = self.client.get_servers()
            new_server_names = (sn for sn in server_names
                                if sn not in self.status_subs)

            # Subscribe to new servers
            for server_name in new_server_names:
                self.structure_subscribe(server_name)
                self.status_subscribe(server_name)

            # This doesn't need to happen very often
            rospy.sleep(1.0)

    def structure_subscribe(self, server_name):
        """Subscribe to a new structure messaging server."""
        self.structure_subs[server_name] = rospy.Subscriber(
            server_name + smach_ros.introspection.STRUCTURE_TOPIC,
            SmachContainerStructure,
            callback=self.structure_updater,
            callback_args=server_name,
            queue_size=50
        )
        print('subscribed to structure server', server_name)

    def status_subscribe(self, server_name):
        """Subscribe to a new status messaging server."""
        self.status_subs[server_name] = rospy.Subscriber(
            server_name + smach_ros.introspection.STATUS_TOPIC,
            SmachContainerStatus,
            callback=self.status_updater,
            queue_size=50
        )
        print('subscribed to status server', server_name)
