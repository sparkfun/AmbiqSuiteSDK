SRC = adc_regs.src
SRC+= clkgen_regs.src
SRC+= ctimer_regs.src
SRC+= flashctrl_regs.src
SRC+= gpio_regs.src
SRC+= iomstr_regs.src
SRC+= ioslave_regs.src
SRC+= mcuctrlpriv_regs.src
SRC+= mcuctrl_regs.src
SRC+= rstgen_regs.src
SRC+= rtc_regs.src
SRC+= uart_regs.src
SRC+= vcomp_regs.src
SRC+= wdt_regs.src

ARMSRC = itm_regs.arm.src
ARMSRC+= tpiu_regs.arm.src
ARMSRC+= nvic_regs.arm.src
ARMSRC+= sysctrl_regs.arm.src
ARMSRC+= systick_regs.arm.src

HTML = $(SRC:%.src=%.html)
HTML+= $(SRC:%.src=%_internal.html)
HTML+= $(ARMSRC:%.arm.src=%.html)

VPATH = ../../../mcu/apollo/regs/source pages

all: $(HTML) ../../../scripts/header_gen/html_gen.py
	dos2unix pages/*.html

clean:
	rm pages/*.html

%.html:	%.src
	python3 ../../../scripts/header_gen/html_gen.py $< > pages/$@

%_internal.html: %.src
	python3 ../../../scripts/header_gen/html_gen.py --internal $< > pages/$@

%.html:	%.arm.src
	python3 ../../../scripts/header_gen/html_gen.py $< > pages/$@

