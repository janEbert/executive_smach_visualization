from __future__ import print_function
import threading

import rospy


class TreeUpdater(object):
    """Responsible for updating the tree view for the connected viewer."""

    def __init__(self, viewer):
        self.viewer = viewer
        self.tree_nodes = {}

        self.update_tree_thread = threading.Thread(
            target=self.update_tree_loop
        )
        self.update_tree_thread.start()

    def __del__(self):
        self.update_tree_thread.join()
        super(TreeUpdater, self).__del__(self)

    def update_tree_loop(self):
        """Continually update the tree view."""
        while self.viewer.keep_running and not rospy.is_shutdown():
            with self.viewer.update_cond:
                self.viewer.update_cond.wait()

                modified = False
                for path, root_container in self.viewer.root_containers.items():
                    modified |= self.add_to_tree(None, path)
                    print('update tree status:', path)
                    self.update_tree_status(root_container)
            print('\n')

    def add_to_tree(self, parent, path):
        """
        Add the given path to the tree under the given parent node.
        If ``parent`` is ``None``, add it as a root node.
        """
        modified = False
        container = self.tree_nodes.get(path, None)
        if container is None:
            print('adding to tree:', path)
            container = self.viewer.append_to_tree(parent,
                                                   TreeUpdater.get_label(path))
            self.tree_nodes[path] = container
            modified = True

        subtree = self.viewer.containers.get(path, None)
        if subtree is None:
            return modified

        # Add children to tree
        children = subtree._children
        added_children = False
        for label in children:
            child_path = '/'.join((path, label))
            added_children |= self.add_to_tree(container, child_path)
            if added_children:
                # ``expand_path_tree_up_to`` expects a ``Gtk.TreePath``;
                # ``container`` is a ``Gtk.TreeIter``.
                container_path = self.viewer.get_tree_path(container)
                self.viewer.expand_path_tree_up_to(container_path)

        return modified or added_children

    def update_tree_status(self, container):
        """
        Recursively update whether the container or its children are active by
        changing their font weight in the tree view.
        """
        for child_label in container._children:
            child_path = '/'.join((container._path, child_label))
            child_item = self.tree_nodes[child_path]
            self.viewer.path_tree_set_weight(
                child_item,
                child_label in container._active_states
            )
            if child_path in self.viewer.containers:
                self.update_tree_status(self.viewer.containers[child_path])

    @staticmethod
    def get_label(path):
        """Return the label of an xdot node."""
        path_tokens = path.split('/')
        return path_tokens[-1]
