cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(switch2_ll)

target_sources(app PRIVATE src/main.c src/switch2_gatt.c src/switch2_memory.c
                          src/switch2_store.c src/rf_radio.c
                          src/puck_usb.c src/puck_hid.c
                          src/rf_link.c src/puck_input.c src/switch_pro.c
                          src/switch_pro_usb.c src/puck_trace.c
                          src/puck_settings.c
                          src/puck_rumble.c src/puck_webdisk.c
                          src/puck_melody.c)

# The console's LL_ENC_REQ carries EDIV=0/Rand=0 -- an out-of-band key agreed
# over GATT, not via SMP -- so the derived LTK has to be pushed straight into
# the host's key store. bt_keys_get_addr()/bt_keys_add_type() are host-internal
# with no public equivalent, so the host directory is on the include path.
# Reassess if a supported API for injecting an out-of-band LTK ever appears.
target_include_directories(app PRIVATE $ENV{ZEPHYR_BASE}/subsys/bluetooth/host)
