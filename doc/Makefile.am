RNOTES=D3-Jetson-BSP-Release-Notes.pdf
RNOTES_MD=D3-Jetson-BSP-Release-Notes.md
NXBUP=D3-Jetson-NX-Carrier-Bringup.pdf
NXBUP_MD=D3-Jetson-NX-Carrier-Bringup.md
doc_DATA = $(RNOTES) $(NXBUP)

.PHONY: always_rebuild

doc_deps = \
	img/d3_logo.png img/d3-6x-serdes-key-points.png \
	img/d3-xavier-16x-fpdlink-key-points.png img/sdk-installer.png \
	d3-style.latex

nx_doc_deps = \
	img/jetson-nx-sdkmanager-step1.png \
	img/jetson-nx-sdkmanager-step2.png

PANDOC_FLAGS = --data-dir=./ --template d3-style --number-sections --listings

$(RNOTES): $(RNOTES_MD) $(doc_deps) $(if $(VIEW), always_rebuild)
	$(AM_V_GEN)pandoc $(PANDOC_FLAGS) -s -o "$@" "$<"
	$(AM_V_at)if [ -n "$$VIEW" ]; then xdg-open "$@" & fi

$(NXBUP): $(NXBUP_MD) $(nx_doc_deps)
	$(AM_V_GEN)pandoc -s -o "$@" "$<"

%.md: %.in

clean-local:
	$(v)rm $(RNOTES) $(MD) $(NXBUP) $(NXBUP_MD)
