
# Include files

- [include/bonogps_board_settings.h](bonogps_board_settings.h) includes board settings (PINS mostly) that are specific to a tested board.

## Generated files

These header files are generated so you should not manually modify them. Regenerate them with [generate_css.sh](../generate_css.sh)

- [include/bonogps_css_base.h](bonogps_css_base.h) preprocessing macro that defines the strings returned by the `http://bonogps.local/css` url in the portal
- [include/bonogps_css_base_battery.h](bonogps_css_base_battery.h) preprocessing macro that defines the strings returned by the `http://bonogps.local/css` url in the portal with additional definitions to show the battery (no need to increase the flash size if you don't need a battery gauge). This is used when macro `SHOWBATTERY` is used

## Source files

Apply your customizations to CSS files here in their native format: they are minimized when imported in the source code.

- [include/style_base.css](style_base.css) CSS definitions for the web portal
- [include/style_base_battery.css](style_base_battery.css) CSS definitions required to display the battery gauge in the web portal
