# `make generate` - generate the header.
# `make test_all` or just `make` - generate and run tests for all compilers.
#     set `COMPILERS=...` to only test specific compilers, or run e.g. `make 'test_g++'`.

ifeq (OS,Windows_NT)
TARGET_OS = windows
else
TARGET_OS = linux
endif

ifeq (TARGET_OS,windows)
COMPILERS = clang++ g++ cl clang-cl
else
COMPILERS = clang++ g++
endif

# Compiler flags, for GCC-like compilers and for MSVC-like respectively.
FLAGS :=
FLAGS_CL :=

# Clone the repo.
box2c:
	rm -rf box2c
	mkdir $@
	#git clone https://github.com/erincatto/box2c

# Generate the file.
include/box2c.hpp test/test_header.hpp &: box2c $(wildcard box2c/include/box2d/*.h) | include test
	$(call, ### Strip the newlines inside parentheses, then give the result to awk.)
	$(call, ### `box2d.h` is the primary header. `types.h` and `joint_types.h` are needed for `b2Default...Def` functions.)
	$(call, ### Yes, `cat` will mangle first/last lines, but it doesn't matter.)
	cat box2c/include/box2d/* \
		| perl -pe 's/(\([^)]*)\n/$$1/' \
		| gawk -f generator/generate_classes.awk >include/box2c.hpp -vsecond_file=tmp.part2
	cat tmp.part2 >>include/box2c.hpp
	rm tmp.part2
	sed include/box2c.hpp -e 's|<box2d/\(.*\)>|"../box2c/include/box2d/\1"|' >test/test_header.hpp

# Directories:
$(strip include) test:
	mkdir -p $@

# Force regeneration.
.PHONY: include/box2c.hpp

.PHONY: generate
generate: include/box2c.hpp

override all_test_targets =
override define test_snippet =
.PHONY: test_$1
test_$1: include/box2c.hpp
	$1 test/test.cpp $(if $(filter %cl,$1)\
		,/EHsc /std:c++latest /W4 /WX \
			-lbox2d $(FLAGS_CL) \
			/link '/out:test/test_$1.exe' \
		,-std=c++20 -Wall -Wextra -pedantic-errors -Wconversion -Wextra-semi -Wdeprecated -Werror -g \
			$(if $(filter windows,$(TARGET_OS)),-fsanitize=address -fsanitize=undefined) \
			-lbox2d $(FLAGS) \
			-o 'test/test_$1' \
	)
	'test/test_$1'
	@echo 'Test passed on: $1'

$(eval override all_test_targets += test_$1)
endef

$(foreach c,$(COMPILERS),\
	$(call,$(shell which $c >/dev/null 2>/dev/null))\
	$(if $(filter 0,$(.SHELLSTATUS)),$(eval $(call test_snippet,$c)))\
)

.PHONY: run_tests
run_tests: $(all_test_targets)

.DEFAULT_GOAL := run_tests
