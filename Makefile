# Line-feed character.
override define lf :=
$(strip)
$(strip)
endef

# Clone the repo.
box2c:
	rm -rf box2c
	git clone https://github.com/erincatto/box2c

box2c/%: box2c

# Generate the file.
box2c.hpp: box2c/include/box2d/box2d.h
	$(call, ### Strip the newlines inside parentheses, then give the result to awk.)
	$(call, ### `box2d.h` is the primary header. `types.h` and `joint_types.h` are needed for `b2Default...Def` functions.)
	$(call, ### Yes, `cat` will mangle first/last lines, but it doesn't matter.)
	cat $(addprefix box2c/include/box2d/,box2d.h types.h joint_types.h) \
		| perl -pe 's/(\([^)]*)\n/$$1/' \
		| gawk -f generator/generate_classes.awk >$@

.DEFAULT_GOAL := box2c.hpp