# -*-makefile-*-
obj-$(CONFIG_D3_EARLY_AT24) += early-at24-generated.o

# We are copying and modifying the kernel's at24.c.
orig_at24 = $(srctree)/drivers/misc/eeprom/at24.c
modified_at24 = $(obj)/early-at24-generated.c

# The script that makes the modifications
sed_script = $(srctree)/$(src)/modify-at24.sed

# How to make the generated file
quiet_cmd_early_at24 = CONVERT $@
      cmd_early_at24 = sed -E -f "$(sed_script)" "$<" > "$@"

$(obj)/early-at24-generated.o: $(modified_at24)

$(modified_at24): $(orig_at24) $(sed_script)
	$(call cmd,early_at24)
