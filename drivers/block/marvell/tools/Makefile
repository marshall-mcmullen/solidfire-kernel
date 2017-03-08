target = wamctl wamdiag nvperf
#fw_header package wamlibtest
LD_FLAGS = -lmvwam -L./ -lrt
sharedlib = libmvwam.so
CFLAGS = -g -I../ -Wall -Werror -DCONFIG_ENABLE_ASYNC_APIS

all: $(sharedlib) $(target)

objs = wamlib.o

$(sharedlib): $(objs)
	ld $(objs) -lc -shared -o $(sharedlib)

nvperf: nvperf.c
	gcc $(CFLAGS) wamlib.c nvperf.c -o nvperf

wamlibtest: wamlibtest.c
	gcc $(CFLAGS) $(LD_FLAGS) wamlibtest.c -o wamlibtest

wamctl: wamctl.c wamlib.c
	gcc $(CFLAGS) wamctl.c wamlib.c -o wamctl

package: package.c
	gcc $(CFLAGS) package.c wamlib.c -o package


%.o: %.c
	gcc -I../ -Wall -Werror -fPIC -c $< -o $@
	
%: %.c
	gcc -g -I../ -Wall -Werror -lrt $< -o $@ $(objs)
	
.PHONY : clean
clean:
	rm -f $(target) $(objs) $(sharedlib)
