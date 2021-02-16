DUT = ../../../../../../kernel/d3/drivers/d3/config-eeprom

.PHONY: test clean

TESTS = sanity.test omelette-decode.test

CFLAGS += -std=gnu99 -I . -I $(DUT)
LDFLAGS += -L/usr/local/lib/UnityTest -lunity

test: $(TESTS)
	for test in $^ ; do "./$$test" ; done

omelette-decode.o: omelette-decode.c
	gcc -c -o $@ $^ $(CFLAGS)

dut-omelette-decode.o: $(DUT)/omelette-decode.c
	gcc -c -o $@ $^ $(CFLAGS)

omelette-decode.test: omelette-decode.o dut-omelette-decode.o
	gcc -o $@  $^ $(LDFLAGS)

sanity.test: sanity.c
	gcc -o $@ $^ $(CFLAGS) $(LDFLAGS)

clean:
	-rm *.o $(TESTS)
