
HTML_DIR = ./html

XML_SRC_DIR = ../xml_src

XML_SRC = $(XML_SRC_DIR)/adc_regs.xml \
          $(XML_SRC_DIR)/gpio_regs.xml \
          $(XML_SRC_DIR)/ioslave_regs.xml \
          $(XML_SRC_DIR)/iomstr_regs.xml \
          $(XML_SRC_DIR)/vcomp_regs.xml \
          $(XML_SRC_DIR)/ctimer_regs.xml \
          $(XML_SRC_DIR)/iomstr_regs.xml \
          $(XML_SRC_DIR)/uart_regs.xml

HTML_RESULTS = $(patsubst $(XML_SRC_DIR)/%.xml, $(HTML_DIR)/%.htm, $(XML_SRC))

INTERNAL_HTML_DIR = ./internal_html

INTERNAL_XML_SRC_DIR = ../xml_src/internal

INTERNAL_XML_SRC = $(INTERNAL_XML_SRC_DIR)/flash_ctrl_regs.xml $(INTERNAL_XML_SRC_DIR)/mcu_ctrl_regs.xml


SRCS = ./src/xml_reg.c ./src/create_dirs.c ./src/parse_device_tree.c ./src/string_store.c ./src/print_device_tree.c ./src/emit_html.c ./src/emit_rtf.c



all: xml_reg $(HTML_RESULTS) 
	cp ./html/html/* ../html


adc: xml_reg
	-@mkdir trial_run
	./xml_reg  $(XML_SRC_DIR)/adc_regs.xml trial_run |tee adc_run.log

iomstr: xml_reg
	-@mkdir trial_run
	./xml_reg  $(XML_SRC_DIR)/iomstr_regs.xml trial_run |tee iom_run.log

xml_reg: $(SRCS) Makefile
	gcc $(SRCS) `xml2-config --cflags --libs` -I./src -o xml_reg
	-@mkdir trial_run


$(HTML_DIR)/%.htm:$(XML_SRC_DIR)/%.xml xml_reg
	@echo  making $<
	@echo $< >>/tmp/run_list	
	./xml_reg $<   $(HTML_DIR) |tee -a big_run.log



$(INTERNAL_HTML_DIR)/%.htm:$(INTERNAL_XML_SRC_DIR)/%.xml xml_reg
	./xml_reg $<   $(INTERNAL_HTML_DIR) |tee -a big_run.log


trial:
	echo $(HTML_DIR)
	@echo 
	echo $(XML_SRC)
	@echo 
	echo $(HTML_RESULTS)
	@echo 
	echo $(INTERNAL_HTML_DIR)
	@echo 
	echo $(INTERNAL_XML_SRC)
	@echo 


clean:
	-@/bin/rm -rf xml_reg /tmp/bin /tmp/html /tmp/rtf trial_run *.log html/*
