# SPDX-License-Identifier: GPL-2.0-only
#
# Makefile for Bosch SMI230 driver
#
KDIR ?= /lib/modules/$(shell uname -r)/build
BUILD_DIR ?= $(PWD)/build
BUILD_DIR_MAKEFILE := $(BUILD_DIR)/Makefile

all: $(BUILD_DIR_MAKEFILE)
	$(MAKE) -C $(KDIR) M=$(BUILD_DIR) src=$(PWD) modules

$(BUILD_DIR):
	mkdir -p "$@"

$(BUILD_DIR_MAKEFILE): $(BUILD_DIR)
	touch "$@"

modules_install:
	$(MAKE) -C $(KDIR) M=$(BUILD_DIR) src=$(PWD) modules_install

clean:
	$(MAKE) -C $(KDIR) M=$(BUILD_DIR) src=$(PWD) clean
