#*******************************************************************************
#
# Simple Makefile to prepare binaries for AMOTA.
#
#*******************************************************************************

FLAG_ADDR_APOLLO2 = 0x6000
LOAD_ADDR_APOLLO2 = 0x8000
BOOTBIN_APOLLO2 =../../../boards/apollo2_evb/examples/multi_boot/iar/bin/multi_boot.bin

# Apollo2-BLUE EVB
UPDATEBIN_APOLLO2_BLUE =../../../boards/apollo2_blue_evb/examples/ble_freertos_amota_blinky/iar/bin/ble_freertos_amota_blinky.bin
APPBIN_APOLLO2_BLUE =../../../boards/apollo2_blue_evb/examples/ble_freertos_amota/iar/bin/ble_freertos_amota.bin
APPBIN_APOLLO2_BLUE_ETHERMIND =../../../boards/apollo2_blue_evb/examples/mindtree_amota/iar/bin/mindtree_amota.bin

all: $(BOOTBIN_APOLLO2) \
	$(APPBIN_APOLLO2_BLUE) $(UPDATEBIN_APOLLO2_BLUE) \
	
	python3 bootloader_binary_combiner.py --bootbin "${BOOTBIN_APOLLO2}" --appbin "${APPBIN_APOLLO2_BLUE}" --flag-addr ${FLAG_ADDR_APOLLO2} --load-address ${LOAD_ADDR_APOLLO2} -o starter_binary_apollo2_blue
	python3 ota_binary_converter.py --appbin "${UPDATEBIN_APOLLO2_BLUE}" --load-address ${LOAD_ADDR_APOLLO2} -o update_binary_apollo2_blue


$(BOOTBIN_APOLLO2):
	$(MAKE) -C ../../../boards/apollo2_evb/examples/multi_boot/iar/ $(MAKECMDGOALS)

$(APPBIN_APOLLO2_BLUE):
	$(MAKE) -C ../../../boards/apollo2_blue_evb/examples/ble_freertos_amota/iar/ $(MAKECMDGOALS)

$(UPDATEBIN_APOLLO2_BLUE):
	$(MAKE) -C ../../../boards/apollo2_blue_evb/examples/ble_freertos_amota_blinky/iar/ $(MAKECMDGOALS)


clean:
	rm -rf *.bin
	$(MAKE) -C ../../../boards/apollo2_evb/examples/multi_boot/iar/ $(MAKECMDGOALS)
	$(MAKE) -C ../../../boards/apollo2_blue_evb/examples/ble_freertos_amota/iar/ $(MAKECMDGOALS)
	$(MAKE) -C ../../../boards/apollo2_blue_evb/examples/ble_freertos_amota_blinky/iar/ $(MAKECMDGOALS)
	