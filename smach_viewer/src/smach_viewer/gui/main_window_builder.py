from .gtk_wrap import Gtk


class MainWindowBuilder(object):
    """Helper functions for creating the main window."""

    """The page in ``self.notebook`` containing the tree view."""
    PATH_TREE_MODEL_TEXT_COLUMN = 0
    """The column of the path tree model containing the text shown."""
    PATH_TREE_MODEL_WEIGHT_COLUMN = 1
    """
    The column of the path tree model containing the weight the text is
    rendered with.
    """

    NOTEBOOK_GRAPH_VIEW_PAGE = 0
    """
    The page in the notebook (tabbed navigator) containing the
    graph view.
    """
    NOTEBOOK_TREE_VIEW_PAGE = 1
    """The page in the notebook (tabbed navigator) containing the tree view."""

    @staticmethod
    def build_tree_model():  # data=None):
        """Return an empty tree model."""  # optionally filled with the given data."""
        tree_model = Gtk.TreeStore(str, int)
        # TODO remove?
        # if data is not None:
        #     MainWindowBuilder.fill_tree(tree_model, data)
        return tree_model

    @staticmethod
    def build_list_model(data=None):
        """Return a list model optionally filled with the given data."""
        list_model = Gtk.ListStore(str)
        if data is not None:
            for row in data:
                list_model.append(row)
        return list_model

    # TODO remove?
    # @staticmethod
    # def fill_tree(tree_model, data):  # TODO
    #     """Fill the given tree model with the given data."""
    #     MainWindowBuilder.recursive_map(lambda x: tree_model.append(x), data)

    @staticmethod
    def recursive_map(f, x):
        """
        Recursively map function ``f`` on the given iterable ``x``
        unless it is a string.
        """
        if not isinstance(x, str):
            try:
                iter(x)
            except TypeError:  # x is not iterable
                pass
            else:  # x is iterable but not string
                for y in x:
                    MainWindowBuilder.recursive_map(f, y)
                return
        f(x)  # x is not iterable or string

    @staticmethod
    def build_text_buffer():
        """Return an empty text buffer."""
        return Gtk.TextBuffer()

    @staticmethod
    def build_graph_view(toolbar, graph_widget):
        """Return the graph view containing the given toolbar and widget."""
        graph_view = Gtk.Box()
        graph_view.set_orientation(Gtk.Orientation.VERTICAL)
        # graph_view.set_spacing(5)

        # child_item: Gtk.Widget, expand: bool, fill: bool, padding: int
        graph_view.pack_start(toolbar, False, False, 0)
        graph_view.pack_end(graph_widget, True, True, 0)
        return graph_view

    @staticmethod
    def build_tree_view(model):
        """Return the tree view for the given model."""
        tree_view = Gtk.TreeView(model)
        renderer = Gtk.CellRendererText()
        column = Gtk.TreeViewColumn(
            'Path',
            renderer,
            text=MainWindowBuilder.PATH_TREE_MODEL_TEXT_COLUMN,
            weight=MainWindowBuilder.PATH_TREE_MODEL_WEIGHT_COLUMN,
            weight_set=True
        )
        tree_view.append_column(column)
        return tree_view

    @staticmethod
    def build_paned_frame(label=None):
        """
        Return a frame for use in a ``Gtk.Paned`` with the given label.
        """
        # Put `Paned` child in frame as recommended by
        # https://lazka.github.io/pgi-docs/Gtk-3.0/classes/Paned.html#Gtk.Paned
        # "Often, it is useful to put each child inside a Gtk.Frame with
        # the shadow type set to Gtk.ShadowType.IN so that the gutter
        # appears as a ridge." (retrieved 2019-03-08)
        #
        # We only do this for the detail view because the graph view
        # should have all the space it can get.
        frame = Gtk.Frame.new(label)
        frame.set_shadow_type(Gtk.ShadowType.IN)
        return frame

    @staticmethod
    def build_notebook(graph_view, tree_view):
        """
        Return the notebook holding the given graph and tree view.
        The notebook is a tabbed navigator between the two.
        """
        # When you change this, make sure to change the class constants
        # NOTEBOOK_GRAPH_VIEW_PAGE and NOTEBOOK_TREE_VIEW_PAGE as well!
        notebook = Gtk.Notebook()
        notebook.append_page(graph_view,
                             MainWindowBuilder.build_label('Graph View'))
        notebook.append_page(tree_view,
                             MainWindowBuilder.build_label('Tree View'))
        return notebook

    @staticmethod
    def build_label(label):
        """Return a label for the notebook."""
        label = Gtk.Label(label)
        label.set_selectable(False)
        return label

    @staticmethod
    def build_paned(notebook, detail_view_frame):
        # type: (Gtk.Notebook, Gtk.Frame)
        """
        Return a ``Gtk.Paned`` holding the given notebook and detail
        view frame.
        """
        paned = Gtk.Paned.new(Gtk.Orientation.HORIZONTAL)
        paned.pack1(notebook,          resize=True,  shrink=True)
        paned.pack2(detail_view_frame, resize=False, shrink=True)
        return paned
