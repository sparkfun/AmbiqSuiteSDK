SRCPATH = ../../../../devices/apollo/regs/source

SRC = adc_regs.src
SRC+= clk_gen_regs.src
SRC+= ctimer_regs.src
SRC+= gpio_regs.src
SRC+= iomstr_regs.src
SRC+= ioslave_regs.src
SRC+= rstgen_regs.src
SRC+= uart_regs.src
SRC+= vcomp_regs.src
SRC+= wdt_regs.src

INTSRC = mcu_ctrl_regs.src
INTSRC+= flash_ctrl_regs.src

XML = $(SRC:.src=.xml)
INTXML = $(INTSRC:%.src=internal/%.xml)

all: $(XML) $(INTXML)

realclean:
	rm -rf *.xml internal/*.xml

%.xml: $(SRCPATH)/%.src
	python3 ../../../../scripts/regGen/regGen.py -d $<

internal/%.xml: $(SRCPATH)/%.src
	python3 ../../../../scripts/regGen/regGen.py -d -dird internal $<
