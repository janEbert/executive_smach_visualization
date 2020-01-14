from .gtk_wrap import Gtk


# In the old viewer, this was called the 'userdata widget'
class DetailView(Gtk.Box):
    """
    Detail view showing the path to states and containing buttons to
    start transitions between states.
    """

    def __init__(self, path_list_model, text_buffer, event_list_model):
        Gtk.Box.__init__(self)
        self.set_border_width(5)
        self.set_orientation(Gtk.Orientation.VERTICAL)
        self.set_spacing(5)

        # TODO We can remove these class variables if desired.
        # TODO Or remove them in the main window... Decoupling
        self.path_list_model = path_list_model
        self.text_buffer = text_buffer
        self.event_list_model = event_list_model

        # Show the HSM's stages' paths and update to the selected stage's path
        self.path_combo_box = DetailView.build_combo_box(self.path_list_model)
        self.text_view = DetailView.build_text_view(self.text_buffer)
        self.event_combo_box = DetailView.build_combo_box(
            self.event_list_model)

        self.add_gui_elements(
            self.path_combo_box,
            self.text_view,
            self.event_combo_box
        )

    def add_gui_elements(self, path_combo_box, text_view, event_combo_box):
        """Complete the detail view with its graphical elements."""
        elements = (
            DetailView.build_label('Path:'),
            path_combo_box,
            DetailView.build_label('Userdata:'),
            text_view,
            DetailView.build_button('Trigger Transition'),
            event_combo_box,
            DetailView.build_button('Trigger Event'),
        )

        for e in elements:
            self.add(e)

    @staticmethod
    def build_label(label):
        """Return a label for the detail view."""
        label = Gtk.Label(label, xalign=0)
        label.set_selectable(False)
        return label

    @staticmethod
    def build_combo_box(model):
        """Return a combo box containing the given model's data as choices."""
        combo_box = Gtk.ComboBox.new_with_model(model)
        return combo_box

    @staticmethod
    def build_text_view(text_buffer):
        """Return a text view."""
        text_view = Gtk.TextView.new_with_buffer(text_buffer)
        text_view.set_vexpand(True)
        text_view.set_editable(False)
        text_view.set_cursor_visible(False)
        return text_view

    @staticmethod
    def build_button(label):
        """Return a button with the given label."""
        button = Gtk.Button.new_with_label(label)
        return button
