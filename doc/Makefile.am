RNOTES=D3-Jetson-BSP-Release-Notes.pdf
doc_DATA = $(RNOTES)

PANDOC_FLAGS = --data-dir=./ --template d3-style --number-sections --listings

%.pdf : %.md
	$(AM_V_GEN)pandoc $(PANDOC_FLAGS) -s -o "$@" "$<"

%.md: %.in

clean-local:
	$(v)rm $(RNOTES)
