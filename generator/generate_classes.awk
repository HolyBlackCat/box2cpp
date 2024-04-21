BEGIN {
    print "#pragma once"
    print ""
    print "#include <box2d/box2d.h>"
    print "#include <box2d/color.h>" # We include a bunch of semi-internal headers, because we wrap all of them, for better or worse.
    print "#include <box2d/distance.h>"
    print "#include <box2d/dynamic_tree.h>"
    print "#include <box2d/math.h>"
    print "#include <box2d/timer.h>"
    print ""
    print "#include <cstddef>" # For `std::nullptr_t`.
    print "#include <concepts>" # For `std::derived_from`.
    print "#include <stdexcept>" # For `std::runtime_error`.
    print "#include <utility>" # For `std::exchange`.
    print ""
    print "namespace b2"
    print "{"

    cur_enum_name = ""

    # For those types, we don't generate a class even if we otherwise would.
    forced_non_classes["Vec2"] = 1 # Yeah no, adding one more vector type just for a single member function `b2Vec2_IsValid` is dumb.
}

# Collect enums.

cur_enum_name && /\}/ {
    # Finish the enum.
    cur_enum_name = ""
    next
}

cur_enum_name && /\{/ {next}

cur_enum_name {
    # Extract name and value.
    match($0, /^\s*b2_(\w+)(\s*=\s*(\w+))?,?$/, elems)
    if (RLENGTH == -1)
    {
        print "Unable to parse enum element `" $0 "` in `" cur_enum_name "`." >"/dev/stderr"
        exit 1
    }

    enums[cur_enum_name]["elems"][length(enums[cur_enum_name]["elems"])+1] = elems[1]
    next
}

/^\s*typedef enum / {
    # Begin the enum.

    match($0, /^\s*typedef enum b2(\w+)$/, elems)
    if (RLENGTH == -1)
    {
        print "Unable to parse the enum header `" $0 "`." >"/dev/stderr"
        exit 1
    }

    cur_enum_name = elems[1]
    enums[cur_enum_name]["comment"] = gensub(/\n/, "\n    ", "g", doc_comment)
    next
}

# Collect struct names.

/^\s*typedef struct / {
    # Extract the unprefixed name.
    match($0, /^\s*typedef struct b2(\w*)/, elems)
    if (RLENGTH == -1)
    {
        print "This typedef struct (`" $0 "`) isn't prefixed, why?" >"/dev/stderr"
        exit 1
    }

    name = elems[1]
    if (!(name in typedef_structs_set))
    {
        typedef_structs_set[name] = length(typedef_structs_set) + 1
        typedef_structs[length(typedef_structs_set)]["name"] = name
        typedef_structs[length(typedef_structs_set)]["comment"] = gensub(/\n/, "\n    ", "g", doc_comment)
    }

    next
}

# Collect function declarations.

/^\s*(B2_API|B2_INLINE)/ {
    str = $0

    # Strip B2_API prefix
    str = gensub(/^\s*(B2_API|B2_INLINE)\s+/, "", 1, str)

    # Extract the declaration elements.
    match(str, /^((const\s+)?\w+(\s*\*)*)\s*(\w+)\s*\(([^)]*)\)/, elems)
    if (RLENGTH == -1)
    {
        print "Failed to parse the function declaration." >"/dev/stderr"
        exit 1
    }

    func_name = elems[4]
    func_param_string = elems[5]

    funcs[func_name]["comment"] = gensub(/\n/, "\n        ", "g", doc_comment)
    funcs[func_name]["ret"] = gensub(/\s+$/, "", 1, gensub(/\s*\*\s*/, "* ", "g", elems[1]))

    # Extract individual parameters.
    patsplit(func_param_string, elems, /[^ ,][^,]*[^ ,]/)
    for (i in elems)
    {
        match(elems[i], /^\s*(((const|enum)\s+)*\w+(\s*\*)*)\s*(\w+)*\s*$/, subelems)
        if (RLENGTH == -1)
        {
            print "Failed to parse the parameters for " func_name "." >"/dev/stderr"
            exit 1
        }
        funcs[func_name]["params"][i]["type"] = gensub(/\s+$/, "", 1, gensub(/\s*\*\s*/, "* ", "g", subelems[1]))
        funcs[func_name]["params"][i]["name"] = subelems[5]
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
        if (!(elems[1] in classes_set))
        {
            classes_set[elems[1]] = length(classes_set) + 1
            classes[length(classes_set)] = elems[1]
        }
    }
}


# --------

# Collect documentation comments for other entities (must be last!).

{
    line_is_doc_comment = 0
}

/\s*\/\// {
    # Checking just two slashes, to catch all comments, not only doc.
    doc_comment = doc_comment $0 "\n"
    line_is_doc_comment = 1
}

{
    if (!line_is_doc_comment)
        doc_comment = ""
}

# Codegen.

# We use this order to sort our classes in the correct order. (Since some classes depend on others.)
function class_order(c)
{
    if (c == "AABB")
        return 10
    else
        return 20
}

function sort_classes_comparator(ai, av, bi, bv)
{
    return class_order(av) - class_order(bv)
}


END {
    # Emit typedef structs.

    for (i in typedef_structs)
    {
        type = typedef_structs[i]["name"]

        if (!(type in forced_non_classes))
        {
            if (type in classes_set)
                continue; # This is also a full-blown class, don't emit it.

            if (gensub(/Def$/, "", 1, type) in classes_set)
                continue; # This is a `...Def` struct, we don't need to emit it outside of the respective class.
        }

        printf "    %s", typedef_structs[i]["comment"]
        print "using " type " = b2" type ";"
    }

    # Emit enums.

    for (enum_name in enums)
    {
        print ""
        printf "    %s", enums[enum_name]["comment"]
        print "enum class " enum_name
        print "    { "
        for (i in enums[enum_name]["elems"])
        {
            elem_name = enums[enum_name]["elems"][i]

            if (elem_name ~ /^[a-z]+TypeCount$/)
                elem_name_fixed = "_count"
            else if (enum_name == "HexColor")
                elem_name_fixed = gensub(/^color/, "", 1, elem_name)
            else if (enum_name == "ShapeType")
                elem_name_fixed = gensub(/Shape$/, "", 1, elem_name)
            else if (enum_name == "TOIState")
                elem_name_fixed = gensub(/^toiState/, "", 1, elem_name)
            else
                elem_name_fixed = elem_name

            # Convert first symbol to uppercase.
            elem_name_fixed = toupper(substr(elem_name_fixed, 1, 1)) substr(elem_name_fixed, 2)

            print "        " elem_name_fixed " = b2_" elem_name ", "
        }
        print "    };"
    }

    # Emit classes.
    asort(classes, classes, "sort_classes_comparator")
    for (i in classes)
    {
        type = classes[i]

        if (type in forced_non_classes)
            continue

        print ""

        # Is a class derived from `Joint`?
        is_joint_kind = type ~ /.+Joint/
        if (is_joint_kind)
            base_type_or_self = "Joint"
        else
            base_type_or_self = type

        # Whether the destructor should check validity before destruction.
        # Joints need the check because `b2DestroyBodyAndJoints()` can destroy our joints...
        destructor_needs_validation = base_type_or_self == "Joint"

        # Those store a box2d struct by value, and act as a RAII wrapper.
        is_by_value_raii_wrapper = type == "DynamicTree"
        # Those just inherit from the original struct and add some member functions.
        is_id_based = !is_by_value_raii_wrapper && ("b2Destroy" type in funcs || is_joint_kind)
        is_dumb_wrapper = !is_by_value_raii_wrapper && !is_id_based

        # Has public constructors?
        public_constructible = ("b2Create" type in funcs) || !is_id_based;
        # Has a struct with parameters?
        has_params_struct = ("b2Default" type "Def") in funcs;

        # Primary class comment. Snatch it from the `...Def` struct because those have the best comments.
        printf "    %s", typedef_structs[typedef_structs_set[type "Def"]]["comment"]

        # Class head.
        printf "class " type
        # Base classes.
        if (is_joint_kind)
            printf " : public Joint"
        else if (is_dumb_wrapper)
            printf " : public b2" type
        print ""
        print "    {"

        # Member variables.
        if (is_joint_kind)
        {
            # Nothing, we inherit the ID member.
        }
        else if (is_id_based)
        {
            # ID.
            print "        b2" type "Id id = b2_null" type "Id;"
            print ""
        }
        else if (is_by_value_raii_wrapper)
        {
            # By value.
            print "        b2" type " value{};"
            print ""
        }
        else
        {
            # Nothing.
        }

        # Protected ctor.
        if (!public_constructible)
        {
            print "      protected:"
            print "        constexpr " type "() {}"
            print "        explicit constexpr " type "(b2" type "Id id) : id(id) {}"
            print ""
        }

        # Public members...
        print "      public:"

        # Default ctor.
        if (public_constructible)
        {
            if (is_by_value_raii_wrapper)
                print "        // Consturcts a null (invalid) object."

            print "        constexpr " type "() {}"
            print ""
        }

        # Params struct.
        if (has_params_struct)
        {
            print "        // The constructor accepts either this or directly `b2" type "Def`."
            print "        struct Params : b2" type "Def"
            print "        {"
            print "            Params() : b2" type "Def(b2Default" type "Def()) {}"
            print "        };"
            print ""
        }

        # The parametrized constructor.
        has_parametrized_ctor = 0
        if (public_constructible)
        {
            if (has_params_struct)
            {
                has_parametrized_ctor = 1

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

                printf "        %s", funcs["b2Create" type]["comment"]
                printf type "(" extra_param_decl "const std::derived_from<b2" type "Def> auto &params) : "
                if (is_joint_kind)
                    printf "Joint"
                else
                    printf "id"
                print "(b2Create" type "(" extra_param_use "&params)) { if (!*this) throw std::runtime_error(\"Failed to create a `b2" type "`.\"); }"
            }
            else if (is_by_value_raii_wrapper)
            {
                has_parametrized_ctor = 1

                printf "        %s", funcs["b2" type "_Create"]["comment"]
                print type "(std::nullptr_t) : value(b2" type "_Create()) {}"
                delete funcs["b2" type "_Create"]
            }
            else if (is_dumb_wrapper)
            {
                has_parametrized_ctor = 1

                if (type == "Rot")
                {
                    # Why, why isn't the consine argument first? D:
                    print "        constexpr Rot(float s, float c) : b2Rot{.s = s, .c = c} {}"
                }
                else if (type == "AABB")
                {
                    print "        constexpr AABB(b2Vec2 lowerBound, b2Vec2 upperBound) : b2AABB{.lowerBound = lowerBound, .upperBound = upperBound} {}"
                }
                else if (type == "Vec2")
                {
                    print "        constexpr Vec2(float x, float y) : b2Vec2{.x = x, .y = y} {}"
                }
                else
                {
                    print "How do I generate a parametrized constructor for this type?" >"/dev/stderr"
                    exit 1
                }
            }
        }

        if (!is_joint_kind)
        {
            # Copy/move ctors.
            if (is_dumb_wrapper)
            {
                # Nothing.
            }
            else if (is_id_based)
            {
                if (has_parametrized_ctor)
                    print ""

                print "        " type "(" type "&& other) noexcept : id(std::exchange(other.id, b2_null" type "Id)) {}"
                print "        " type "& operator=(" type " other) noexcept { std::swap(id, other.id); return *this; }"
            }
            else if (type == "DynamicTree")
            {
                if (has_parametrized_ctor)
                    print ""

                print "        " type "(const " type "& other) : " type "() { *this = other; }"
                print "        " type "(" type "&& other) noexcept : value(other.value) { other.value = {}; }"
                printf "        %s", funcs["b2DynamicTree_Clone"]["comment"]
                print type "& operator=(const " type "& other)"
                print "        {"
                print "            if (this == &other) {}"
                print "            else if (!other) *this = {};"
                print "            else"
                print "            {"
                print "                if (!*this) *this = nullptr;"
                print "                b2DynamicTree_Clone(&value, &other.value);"
                print "            }"
                print "            return *this;"
                print "        }"
                print "        " type "& operator=(" type "&& other) noexcept"
                print "        {"
                print "            if (this == &other) return *this;"
                print "            if (*this) *this = {};"
                print "            value = other.value;"
                print "            other.value = {};"
                print "            return *this;"
                print "        }"

                delete funcs["b2DynamicTree_Clone"]
            }
            else
            {
                print "How do I generate copy/move operators for this type?" >"/dev/stderr"
                exit 1
            }

            # Destructor.
            if (is_dumb_wrapper)
            {
                # Nothing.
            }
            else if (is_by_value_raii_wrapper)
            {
                print ""
                printf "        %s", funcs["b2" type "_Destroy"]["comment"]
                printf "~" type "() { if (*this"
                if (destructor_needs_validation)
                    printf " && IsValid()"
                print ") b2" type "_Destroy(&value); }"
                delete funcs["b2" type "_Destroy"]
            }
            else if (is_id_based)
            {
                print ""
                if (destructor_needs_validation && type == "Joint")
                    print "        // Destructor validates the handle because it could've been destroyed by `Body::DestroyBodyAndJoints()`."
                printf "        ~" type "() { if (*this"
                if (destructor_needs_validation)
                    printf " && IsValid()"
                print ") b2Destroy" type "(id); }"
            }

            # ID operations.
            if (is_id_based)
            {
                print ""
                print "        [[nodiscard]] explicit operator bool() const { return B2_IS_NON_NULL(id); }"
                print "        [[nodiscard]] const b2" type "Id &Handle() const { return id; }"
            }
            else if (type == "DynamicTree")
            {
                print ""
                print "        [[nodiscard]] explicit operator bool() const { return bool( value.nodes ); }"
                print "        [[nodiscard]]       b2DynamicTree *RawTreePtr()       { return *this ? &value : nullptr; }"
                print "        [[nodiscard]] const b2DynamicTree *RawTreePtr() const { return *this ? &value : nullptr; }"
            }
        }

        delete funcs["b2Create" type]
        delete funcs["b2Destroy" type]
        delete funcs["b2" type "_Create"]
        delete funcs["b2" type "_Destroy"]
        delete funcs["b2Default" type "Def"]

        # Expose all the functions.
        for (func_name in funcs)
        {
            if (func_name !~ "b2" type "_.*")
                continue

            print ""

            printf "        %s", funcs[func_name]["comment"]

            clean_func_name = gensub("^b2" type "_", "", 1, func_name)
            return_type = funcs[func_name]["ret"]
            if (return_type != "void")
                printf "[[nodiscard]] "
            printf return_type " " clean_func_name "("

            # Parameters.
            first_param = 1
            first_param_is_self = 0
            for (i in funcs[func_name]["params"])
            {
                param_type = funcs[func_name]["params"][i]["type"]

                if (first_param && !first_param_is_self && (is_id_based ? param_type == "b2" base_type_or_self "Id" : param_type ~ "(const )?" type "*"))
                {
                    first_param_is_self = 1
                    continue # This is the `self` param.
                }

                if (first_param)
                    first_param = 0
                else
                    printf ", "

                param_type_fixed = param_type

                param_is_func = param_type ~ /Fcn\*$/

                # Adjust pointer parameters to references (except for `void *`).
                if (!param_is_func && param_type_fixed != "void*" && param_type_fixed ~ /\*$/)
                {
                    param_type_fixed = gensub(/\*$/, "\\&", 1, param_type_fixed)
                    funcs[func_name]["params"][i]["ptr_adjusted_to_ref"] = 1
                }
                # Adjust box2d types to our typedefs.
                if (!param_is_func)
                    param_type_fixed = gensub(/^b2/, "", 1, param_type_fixed)

                printf param_type_fixed " " funcs[func_name]["params"][i]["name"]
            }
            printf ")"

            is_const = 0
            if (is_id_based || is_dumb_wrapper)
            {
                if (clean_func_name ~ /^(Get|Overlap|Is|Compute|Are|Test|Extents|Contains|Union|Center)($|[A-Z])/ || clean_func_name ~ /Cast(Closest)?$/ || clean_func_name == "Draw")
                    is_const = 1
                else if (clean_func_name ~ /^(Set|Enable|Apply|Disable|Reset|Wake|Create|Destroy|Enlarge)($|[A-Z])/ || clean_func_name == "Step")
                    is_const = 0
                else
                {
                    print "Can't guess from this function name if it's const or not." >"/dev/stderr"
                    exit 1
                }
            }
            else if (is_by_value_raii_wrapper && first_param_is_self)
            {
                if (funcs[func_name]["params"][i]["type"] ~ /^const /)
                    is_const = 1
                else
                    is_const = 0
            }
            else
            {
                print "Not sure if this func is const or not." >"/dev/stderr"
                exit 1
            }

            if (is_const)
                printf " const"

            # Function body.

            printf " { return " func_name "("

            first_param = 1
            for (i in funcs[func_name]["params"])
            {
                if (first_param && first_param_is_self)
                {
                    if (is_by_value_raii_wrapper)
                        printf "&value"
                    else if (is_dumb_wrapper)
                        printf "*this"
                    else
                        printf "Handle()"
                    first_param = 0
                    continue
                }

                if (first_param)
                    first_param = 0
                else
                    printf ", "

                # Prepend `&` to take address of a reference.
                if (funcs[func_name]["params"][i]["ptr_adjusted_to_ref"])
                    printf "&"

                param_type = funcs[func_name]["params"][i]["type"]

                # Cast our enums to the original enums.
                if (param_type in enums)
                    printf "(b2" param_type ")"

                param_name = funcs[func_name]["params"][i]["name"]

                printf "%s", param_name
            }
            print "); }"

            # Destroy the function we just generated.
            delete funcs[func_name]
        }

        # Close the class.
        print "    };"
    }

    print "} // namespace box2d"
}
