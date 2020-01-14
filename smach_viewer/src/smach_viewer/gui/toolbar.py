from .gtk_wrap import Gtk


class Toolbar(Gtk.Toolbar):
    """Toolbar at the top of the graph view."""

    def __init__(self, path_list_model):
        Gtk.Toolbar.__init__(self)
        self.set_orientation(Gtk.Orientation.HORIZONTAL)
        # Disable overflow menu (we create our own)
        self.set_show_arrow(False)
        # self.set_style(Gtk.ToolbarStyle.TEXT)

        self.path_list_model = path_list_model

        # Show the HSM's stages' paths and update to the selected stage's path
        self.path_combo_box = Toolbar.build_combo_box(self.path_list_model)
        self.spin_button = Toolbar.build_spin_button()
        # TODO when adding keyboard accelerator, check if it is listed with mnemonic
        self.help_menu_item = Toolbar.build_menu_item('_Help',
                                                      has_mnemonic=False)
        self.save_menu_item = Toolbar.build_menu_item('_Save',
                                                      has_mnemonic=False)

        overflow_menu = Toolbar.build_overflow_menu(self.help_menu_item,
                                                    self.save_menu_item)

        # Create the path toolbar item because we want to modify it.
        path_item = Toolbar.build_tool_item(self.path_combo_box)
        path_item.set_expand(True)

        self.insert_tool_items(path_item, self.spin_button, overflow_menu)

        # TODO see pic

    def insert_tool_items(self, path_item, spin_button, overflow_menu):
        """Complete the toolbar with its tool items."""
        elements = (
            Toolbar.build_label(' Path: '),
            path_item,
            Toolbar.build_label(' Depth: '),
            spin_button,
            Toolbar.build_separator(),
            overflow_menu
            # TODO make overflow possible
            # TODO Help/Save
        )

        for e in elements:
            if not isinstance(e, Gtk.ToolItem):
                e = Toolbar.build_tool_item(e)
            # Insert at end (append)
            self.insert(e, -1)

    @staticmethod
    def build_label(label):
        """Return a label for the toolbar."""
        label = Gtk.Label(label)  # TODO, xalign=0)
        label.set_selectable(False)
        return label

    @staticmethod
    def build_combo_box(model):
        """Return a combo box containing the given model's data as choices."""
        combo_box = Gtk.ComboBox.new_with_model(model)
        return combo_box

    @staticmethod
    def build_spin_button():
        """Return a spin button with adjustments for the toolbar."""
        adjustment = Gtk.Adjustment(value=-1, lower=-1, upper=1337,
                                    step_incr=1, page_incr=5, page_size=0)
        spin_button = Gtk.SpinButton()
        spin_button.set_adjustment(adjustment)

        # Only accept numeric input
        spin_button.set_numeric(True)
        return spin_button

    @staticmethod
    def build_separator():
        """
        Return a separator for the toolbar.
        The separator will force following elements to the end of the toolbar.
        """
        separator = Gtk.SeparatorToolItem.new()
        # With these settings, the separator forces following elements to
        # the end.
        # https://lazka.github.io/pgi-docs/Gtk-3.0/classes/SeparatorToolItem.html#Gtk.SeparatorToolItem
        # (Retrieved on 2020-16-01)
        separator.set_expand(True)
        separator.set_draw(False)
        return separator

    @staticmethod
    def build_overflow_menu(help_item, save_item):
        menu = Gtk.Menu.new()
        menu.append(help_item)
        menu.append(save_item)

        menu_button = Gtk.MenuButton.new()
        menu_button.set_popup(menu)
        return menu_button

    @staticmethod
    def build_menu_item(label, has_mnemonic=False):
        """
        Return a menu item with the given label. If ``has_mnemonic`` is
        ``True``, the mnemonic character is indicated by an underscore
        before it.
        """
        # return Gtk.MenuItem(label, has_mnemonic)
        if has_mnemonic:
            return Gtk.MenuItem.new_with_mnemonic(label)
        else:
            return Gtk.MenuItem.new_with_label(label)

    @staticmethod
    def build_tool_item(content=None):
        """Return a tool item with the given content."""
        # Use the ``new`` constructor for non-button contents as recommended by
        # https://lazka.github.io/pgi-docs/Gtk-3.0/classes/ToolItem.html#Gtk.ToolItem
        # (Retrieved on 2020-16-01)
        tool_item = Gtk.ToolItem.new()
        # tool_item.set_orientation(Gtk.Orientation.HORIZONTAL)
        # tool_item.set_style(Gtk.ToolbarStyle.TEXT)
        if content is not None:
            tool_item.add(content)
        return tool_item
