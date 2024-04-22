# Set to 1 to force regenerate the header.
FORCE := 0

ifneq ($(filter-out 0,$(FORCE)),)
.PHONY: include/box2c.hpp
endif

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

$(strip include) test:
	mkdir -p $@

.DEFAULT_GOAL := include/box2c.hpp
