import threading

from gi.repository import Pango
from xdot import DotWidget

from .client.subscription_manager import SubscriptionManager
from .client.tree_updater import TreeUpdater
from .detail_view import DetailView
from .gtk_wrap import Gtk
from .main_window_builder import MainWindowBuilder
from .toolbar import Toolbar


class MainWindow(Gtk.Window):
    """
    Main window of the smach viewer containing the graph, tree and
    detail view.
    """

    def __init__(self, width, height):
        Gtk.Window.__init__(self, title='Smach Viewer')
        self.set_border_width(10)
        self.set_default_size(width, height)

        # Client backend
        # ==============

        self.update_cond = threading.Condition()
        self.keep_running = True

        self.path = '/'
        self.needs_zoom = True
        self.structure_changed = True
        self.auto_focus = False

        self.containers = {}
        self.root_containers = {}
        self.selected_paths = {}

        self.subscription_manager = SubscriptionManager(self)
        self.tree_updater = TreeUpdater(self)

        # Models
        # =====

        # Tree model of the HSM
        # TODO make this own class
        self.path_tree_model = MainWindowBuilder.build_tree_model()

        # Model storing whole paths (non-hierarchically)
        self.path_list_model = MainWindowBuilder.build_list_model()
        # Separate path model for toolbar
        # Need to nest the data because individual rows are expected to be lists
        self.toolbar_path_list_model = MainWindowBuilder.build_list_model(
            (('/',),))

        # Text buffer for userdata
        self.text_buffer = MainWindowBuilder.build_text_buffer()

        # Tree model for the HSM's events
        # TODO use event data
        self.event_list_model = MainWindowBuilder.build_tree_model()

        # View
        # ====

        # Toolbar (above the graph view)
        # TODO make new class for this
        # TODO select '/' at start for path combo box
        self.toolbar = Toolbar(self.toolbar_path_list_model)

        # Graph view
        # TODO Is the widget enough?
        self.graph_widget = DotWidget()
        self.graph_view = MainWindowBuilder.build_graph_view(self.toolbar,
                                                             self.graph_widget)

        # Tree view
        self.tree_view = MainWindowBuilder.build_tree_view(
            self.path_tree_model)

        # Detail view (separately on the right)
        self.detail_view = DetailView(self.path_list_model, self.text_buffer,
                                      self.event_list_model)
        self.detail_view_frame = MainWindowBuilder.build_paned_frame()
        self.detail_view_frame.add(self.detail_view)

        # Notebook (tabbed navigator between graph and tree view)
        self.notebook = MainWindowBuilder.build_notebook(self.graph_view,
                                                         self.tree_view)

        # Paned (notebook on the left, detail view on the right)
        self.paned = MainWindowBuilder.build_paned(self.notebook,
                                                   self.detail_view_frame)

        self.add(self.paned)
        self.show_all()

        # Show tree view by default
        # Can only do this after showing, see:
        # https://developer.gnome.org/gtk3/stable/GtkNotebook.html#gtk-notebook-set-current-page
        # (Retrieved 2020-01-15)
        self.notebook.set_current_page(
            MainWindowBuilder.NOTEBOOK_TREE_VIEW_PAGE)

        # Control
        # ========

        # TODO keyboard controls for dotwidget
        # TODO connect detail_view's path combo box to graph and tree
        #      view (by updating to the selected node's path)
        #      and vice versa (combo box path selection selects node
        #      in tree view)
        # TODO connect userdata textarea (similar to the path combo box)
        # TODO connect 'trigger transition' button
        # TODO connect 'trigger event' button

    def __del__(self):
        """Signal that the viewer is going to shut down."""
        with self.update_cond:
            self.keep_running = False
            self.update_cond.notify_all()
        super(MainWindow, self).__del__(self)

    def append_to_tree(self, parent, path, make_bold=False):
        """
        Append the given path to the tree model, optionally in bold.
        Return the node.
        """
        if make_bold:
            weight = Pango.Weight.BOLD
        else:
            weight = Pango.Weight.NORMAL
        return self.path_tree_model.append(parent, (path, weight))

    def path_tree_set_weight(self,
                             item,  # type: Gtk.TreeIter
                             make_bold):
        """Change the weight (boldness) of the given item in the tree model."""
        if make_bold:
            weight = Pango.Weight.BOLD
        else:
            weight = Pango.Weight.NORMAL
        self.path_tree_model.set_value(
            item,
            MainWindowBuilder.PATH_TREE_MODEL_WEIGHT_COLUMN,
            weight
        )

    def add_path(self, path):
        """Add the given structure message path to the viewer."""
        row = (path,)
        self.path_list_model.append(row)
        self.toolbar_path_list_model.append(row)

    def set_path(self, path):
        """Set the given status message path as current."""
        self.path = path
        self.needs_zoom = True
        # TODO FIXME self.toolbar.path_combo_box.set_selected/active(path)
        # may be 'hard' to get path index from tree model
        # in old viewer: self.path_combo.SetValue(path) (ComboBox)
        # FIXME not implemented
        self.notify_update_graph()

    def expand_path_tree_up_to(self, tree_path):  # type: (Gtk.TreePath)
        """Expand the whole path tree up to the given ``Gtk.TreePath``."""
        # ``expand_to_path`` expects a ``Gtk.TreePath``
        self.tree_view.expand_to_path(tree_path)

    def get_tree_path(self, tree_iter):  # type: (Gtk.TreeIter)
        """
        Return the ``Gtk.TreePath`` corresponding to the
        given ``Gtk.TreeIter``.
        """
        return self.path_tree_model.get_path(tree_iter)

    def set_max_depth(self, max_depth):
        self.max_depth = max_depth
        # TODO
        # self.toolbar.depth_spinner.set_value(max_depth)
        self.needs_zoom = True
        self.notify_update_graph()

    def notify_update_graph(self):
        with self.update_cond:
            self.update_cond.notify_all()
