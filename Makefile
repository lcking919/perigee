CC      = gcc
CFLAGS  = -std=c11 -Wall -Wextra -Werror -Icore/include

TEST_SRCS = \
	test/test_apogee.c \
	core/src/apogee.c \
	core/src/vapogee.c \
	core/src/boost.c \
	core/src/burnout.c \
	core/src/landing.c \
	core/src/state.c \
	core/src/log.c

.PHONY: test clean

test: test_apogee
	./test_apogee

test_apogee: $(TEST_SRCS)
	$(CC) $(CFLAGS) $(TEST_SRCS) -lm -o test_apogee

clean:
	rm -f test_apogee