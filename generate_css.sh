#!/bin/bash

# Generate header files with a minified version of CSS definitions
# Author: Renato Bonomini
# More info at https://github.com/renatobo/bonogps
# you need to install csscompressor with `pip3 install csscompressor`

echo "/* Auto generated with generate_css.sh, do not modify */" > include/bonogps_css_base.h
printf "const char WEBPORTAL_CSS[] PROGMEM = \"%s\";" \
"`python3 -m csscompressor  include/style_base.css`" \
 >> include/bonogps_css_base.h

 echo "include/bonogps_css_base.h generated"

echo "/* Auto generated with generate_css.sh, do not modify */" > include/bonogps_css_base_battery.h
printf "const char WEBPORTAL_CSS[] PROGMEM = \"%s%s\";" \
"`python3 -m csscompressor  include/style_base.css`" \
"`python3 -m csscompressor  include/style_base_battery.css`" \
>> include/bonogps_css_base_battery.h

echo "include/bonogps_css_base_battery.h generated"