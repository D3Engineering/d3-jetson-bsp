RNOTES=D3-Jetson-BSP-Release-Notes.pdf
MD=D3-Jetson-BSP-Release-Notes.md
doc_DATA = $(RNOTES)

doc_deps = \
	img/d3_logo.png img/d3-cic-key-points.png \
	img/d3-2xocto-key-points.png img/sdk-installer.png \
	d3-style.latex

PANDOC_FLAGS = --data-dir=./ --template d3-style --number-sections --listings

%.pdf : %.md $(doc_deps)
	$(AM_V_GEN)pandoc $(PANDOC_FLAGS) -s -o "$@" "$<"

%.md: %.in

clean-local:
	$(v)rm $(RNOTES) $(MD)
