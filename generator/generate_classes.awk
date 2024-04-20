BEGIN {
    print "#pragma once"
    print ""
    #print "#include <box2d/box2d.h>"
    print "#include \"box2c/include/box2d/box2d.h\""
    print ""
    print "#include <concepts>" # For `std::same_as`.
    print "#include <stdexcept>" # For `std::runtime_error`.
    print "#include <utility>" # For `std::exchange`.
    print ""
    print "namespace box2d"
    print "{"
}

# Accumulate documentation comments.

{
    func_doc_comment = ""
}

# Checking just two slashes, to catch all comments, not only doc.
/\s*\/\// {
    func_doc_comment = func_doc_comment $0 "\n"
}

# Detect function declarations.

/B2_API/ {
    str = $0

    # Strip B2_API prefix
    str = gensub(/^\s*B2_API\s+/, "", 1, str)

    # Extract the declaration elements.
    match(str, /^((const\s+)?\w+(\s*\*)*)\s*(\w+)\s*\(([^)]*)\)/, elems)
    if (RLENGTH == -1)
    {
        print "Failed to parse the function declaration." >"/dev/stderr"
        exit 1
    }

    func_name = elems[4]
    func_param_string = elems[5]

    funcs[func_name]["comment"] = func_doc_comment
    funcs[func_name]["ret"] = gensub(/\s*\*\s*/, "* ", "g", elems[1])

    # Extract individual parameters.
    patsplit(func_param_string, elems, /[^ ,][^,]*[^ ,]/)
    for (i in elems)
    {
        match(elems[i], /^\s*((const\s+)?\w+(\s*\*)*)\s*(\w+)*\s*$/, subelems)
        if (RLENGTH == -1)
        {
            print "Failed to parse the parameters." >"/dev/stderr"
            exit 1
        }
        funcs[func_name]["params"][i]["type"] = gensub(/\s*\*\s*/, "* ", "g", subelems[1])
        funcs[func_name]["params"][i]["name"] = subelems[4]
    }

    # Debug print the functions.
    # printf "ret=[%s] name=[%s] params[", funcs[func_name]["ret"], func_name
    # for (i in funcs[func_name]["params"])
    #     printf "(%s:%s)", funcs[func_name]["params"][i]["type"], funcs[func_name]["params"][i]["name"]
    # printf "%s\n", "]" $0

    # Collect class names.
    match(func_name, /b2(\w+)_\w+/, elems)
    if (RLENGTH != -1)
    {
        classes_set[elems[1]] = 1
        classes[length(classes_set)] = elems[1]
    }
}

# Codegen.

END {
    # Output classes
    first = 1
    for (i in classes)
    {
        type = classes[i]

        if (first)
            first = false
        else
            print ""

        public_constructible = "b2Create" type in funcs;
        has_params_struct = ("b2Default" type "Def") in funcs;

        is_joint_kind = type ~ /.+Joint/
        if (is_joint_kind)
            base_type_or_self = "Joint"
        else
            base_type_or_self = type

        printf "    class " type
        if (is_joint_kind)
            printf " : Joint"
        print ""
        print "    {"
        if (!is_joint_kind)
        {
            print "        b2" type "Id id = b2_null" type "Id;"
            print ""
        }
        if (!public_constructible)
        {
            print "      protected:"
            print "        constexpr " type "() {}"
            print "        explicit constexpr " type "(b2" type "Id id) : id(id) {}"
            print ""
        }
        print "      public:"
        if (public_constructible)
        {
            print "        constexpr " type "() {}"
            print ""
        }

        if (has_params_struct)
        {
            print "        struct Params : b2" type "Def"
            print "        {"
            print "            Params() : b2" type "Def(b2Default" type "Def()) {}"
            print "        };"
        }

        # The parametrized constructor.
        if (public_constructible)
        {
            if (type == "Body" || is_joint_kind)
            {
                extra_param_decl = "World &world, "
                extra_param_use = "world.Handle(), "
                extra_param_name = "world, "
            }
            else if (type == "Chain")
            {
                extra_param_decl = "Body &body, "
                extra_param_use = "body.Handle(), "
                extra_param_name = "body, "
            }
            else
            {
                extra_param_decl = ""
                extra_param_use = ""
                extra_param_body = ""
            }

            print "        " type "(" extra_param_decl "const Params &params) : " type "(" extra_param_name "static_cast<const b2" type "Def &>(params)) {}"
            print "        // This overload constructs directly from `b2" type "Def`. Using `std::same_as` to force `{}` argument"
            print "        // to be interpreted as `b2Default" type "Def()` by the previous overload."
            printf "        " type "(" extra_param_decl "const std::same_as<b2" type "Def> auto &params) : "
            if (is_joint_kind)
                printf "Joint"
            else
                printf "id"
            print "(b2Create" type "(" extra_param_use "&params)) {if (!*this) throw std::runtime_error(\"Failed to create a `b2" type "`.\");}"
        }


        if (!is_joint_kind)
        {
            print ""
            print "        " type "(" type " &&other) noexcept : id(std::exchange(other.id, b2_null" type "Id)) {}"
            print "        " type " &operator=(" type " other) noexcept {std::swap(id, other.id); return *this;}"
            print ""
            print "        ~" type "() {if (*this) b2Destroy" type "(id);}"
            print ""
            print "        [[nodiscard]] explicit operator bool() const {return B2_IS_NON_NULL(id);}"
            print "        [[nodiscard]] const b2" type "Id &Handle() const {return id;}"
        }

        delete funcs["b2Create" type]
        delete funcs["b2Destroy" type]
        delete funcs["b2Default" type "Def"]
        delete funcs["b2" type "_IsValid"] # We don't need to expose this, I believe.

        # Expose all the functions.
        first_func = 1
        for (func_name in funcs)
        {
            if (func_name !~ "b2" type "_.*")
                continue

            if (first_func)
            {
                first_func = 0
                print ""
            }

            printf "        "

print "Dump the comment here!!!" >"/dev/stderr"
exit 1

            clean_func_name = gensub("^b2" type "_", "", 1, func_name)
            return_type = funcs[func_name]["ret"]
            if (return_type != "void")
                printf "[[nodiscard]] "
            printf return_type " " clean_func_name "("

            # Parameters.
            first_param = 1
            for (i in funcs[func_name]["params"])
            {
                param_type = funcs[func_name]["params"][i]["type"]

                if (first_param && param_type == "b2" base_type_or_self "Id")
                    continue # This is the `self` param.

                if (first_param)
                    first_param = 0
                else
                    printf ", "

                printf param_type " " funcs[func_name]["params"][i]["name"]
            }
            printf ")"

            if (clean_func_name ~ /^(Get|Overlap|Is|Compute|Are|Test)[A-Z]/ || clean_func_name ~ /Cast(Closest)?$/ || clean_func_name == "Draw")
                is_const = 1
            else if (clean_func_name ~ /^(Set|Enable|Apply|Disable|Reset|Wake)($|[A-Z])/ || clean_func_name == "Step")
                is_const = 0
            else
            {
                print "Can't guess from this function name if it's const or not." >"/dev/stderr"
                exit 1
            }

            if (is_const)
                printf " const"
            print ";"
        }

        print "    };"


    }

    print "} // namespace box2d"
}
