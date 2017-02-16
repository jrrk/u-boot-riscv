O = api_core.o api_utils.o attribute.o autobuffer.o edcl.o elfloader.o main.o udp.o # stub_main.o
CPPFLAGS=-std=c++11 -g

tester: stub_main.o libedcl.a
	g++ -o $@ stub_main.o libedcl.a -lpthread -ldl -lbfd

libedcl.o: $O
	ld -r -o libedcl.o $O

libedcl.a: libedcl.o
	rm -f $@
	ar rc $@ libedcl.o

clean:
	rm -f tester $O
