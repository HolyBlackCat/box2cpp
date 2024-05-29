# `make` - generate the header.
# To run tests:
# * Compile but not link:
#     make run_tests SYNTAX_ONLY=1
# * Compile, link and run:
#     make run_tests FLAGS=-L/path/to/box2d/lib
#       For MSVC, use FLAGS_CL instead of FLAGS.
#   You can set `COMPILERS=...` to only test specific compilers.
#
# Windows users should run this in MSYS2 `make`. To test MSVC, run `msys2_shell.cmd` inside VS developer prompt.

ifeq (OS,Windows_NT)
COMPILERS = clang++ g++ cl clang-cl
else
COMPILERS = clang++ g++
endif

# Compiler flags, for GCC-like compilers and for MSVC-like respectively.
FLAGS :=
FLAGS_CL :=

# Set to 1 to only compile the tests but don't link.
SYNTAX_ONLY := 0
override SYNTAX_ONLY := $(filter-out 0,$(SYNTAX_ONLY))

# Clone the repo.
box2c:
	rm -rf box2c
	git clone https://github.com/erincatto/box2c

# Generate the file.
include/box2cpp/box2c.hpp test/test_header.hpp &: box2c $(wildcard box2c/include/box2d/*.h) | include test
	$(call, ### Strip the newlines inside parentheses, then give the result to awk.)
	$(call, ### `box2d.h` is the primary header. `types.h` and `joint_types.h` are needed for `b2Default...Def` functions.)
	$(call, ### Yes, `cat` will mangle first/last lines, but it doesn't matter.)
	cat box2c/include/box2d/* \
		| perl -pe 's/(\([^)]*)\n/$$1/' \
		| gawk -f generator/generate.awk >include/box2cpp/box2c.hpp -vsecond_file=tmp.part2
	cat tmp.part2 >>include/box2cpp/box2c.hpp
	rm tmp.part2
	sed 's|<box2d/\(.*\)>|"../box2c/include/box2d/\1"|' include/box2cpp/box2c.hpp >test/test_header.hpp

# Directories:
$(strip include) test:
	mkdir -p $@

# Force regeneration.
.PHONY: include/box2cpp/box2c.hpp

.PHONY: generate
generate: include/box2cpp/box2c.hpp

override all_test_targets =
override define test_snippet =
.PHONY: test_$1
test_$1: include/box2cpp/box2c.hpp
	MSYS2_ARG_CONV_EXCL=* LANG= $1 test/test.cpp $(if $(filter %cl,$1)\
		,/nologo /EHsc /std:c++latest /W4 /WX \
			/Ibox2c/include \
			$(if $(SYNTAX_ONLY),/Zs,/link -lbox2d '/out:test/test_$1.exe') \
			$(FLAGS_CL) \
		,-std=c++20 -Wall -Wextra -pedantic-errors -Wconversion -Wextra-semi -Wdeprecated -Werror -g \
			-Ibox2c/include \
			$(if $(filter windows,$(TARGET_OS)),,-fsanitize=address -fsanitize=undefined) \
			$(if $(SYNTAX_ONLY),-fsyntax-only,-lbox2d -o 'test/test_$1') \
			$(FLAGS) \
	)
	$(if $(SYNTAX_ONLY),,'test/test_$1')
	@echo 'Test passed on: $1'
	@rm -f test.obj

$(eval override all_test_targets += test_$1)
endef

$(foreach c,$(COMPILERS),\
	$(call,$(shell which $c >/dev/null 2>/dev/null))\
	$(if $(filter 0,$(.SHELLSTATUS)),$(eval $(call test_snippet,$c)))\
)

.PHONY: run_tests
run_tests: $(all_test_targets)

.DEFAULT_GOAL := generate
