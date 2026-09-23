CC ?= gcc
CFLAGS ?= -std=c11 -Wall -Wextra -Wpedantic -Werror -O2

.PHONY: all test clean

all: approach_state_test approach_project_adapter_test

approach_state_test: approach_state.c approach_state_test.c approach_state.h approach_state_types.h approach_state_debug.h
	$(CC) $(CFLAGS) -DAPPROACH_ENABLE_DEBUG approach_state.c approach_state_test.c -lm -o $@

approach_project_adapter_test: approach_state.c approach_project_adapter.c approach_project_adapter_test.c
	$(CC) $(CFLAGS) -I../path_validation -I../teaching_state approach_state.c approach_project_adapter.c approach_project_adapter_test.c -lm -o $@

test: approach_state_test approach_project_adapter_test
	./approach_state_test
	./approach_project_adapter_test

clean:
	rm -f approach_state_test approach_project_adapter_test
