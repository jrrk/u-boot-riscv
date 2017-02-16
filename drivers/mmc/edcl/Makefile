O = api_core.o api_utils.o attribute.o autobuffer.o edcl.o elfloader.o main.o udp.o # stub_main.o
CPPFLAGS=-std=c++11 -g

libedcl.a: $O
	rm -f $@
	ar rc $@ $O

clean:
	rm -f $O
