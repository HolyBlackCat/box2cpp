BEGIN {
    own_version = "0.13"

    print "#pragma once"
    print ""
    print "// box2cpp, C++ bindings for box2d 3.x"
    printf "// Generated from box2d commit: "

    "cd box2d && git log -1 --format=\"%h %(describe) %cs\"" | getline box2d_ver_desc
    box2d_ver_desc = gensub(/^\s+/, "", 1, gensub(/\s+$/, "", 1, gensub(/\s{2,}/, " ", "g", box2d_ver_desc)))
    print box2d_ver_desc
    print "// Generator version: " own_version

    print ""
    print "#include <box2d/box2d.h>"
    print ""
    print "#include <concepts>" # For `std::derived_from`.
    print "#include <cstddef>" # For `std::nullptr_t`.
    print "#include <functional>" # For `std::invoke`.
    print "#include <type_traits>" # For our callback helpers, and `std::invoke_result_t`.
    print "#include <utility>" # For `std::exchange`.
    print ""
    print "#ifndef BOX2CPP_ASSERT"
    print "#include <cassert>" # For `assert()`.
    print "#define BOX2CPP_ASSERT(...) assert(__VA_ARGS__)"
    print "#endif"
    print ""
    print "namespace b2"
    print "{"

    print "" >second_file
    print "// Implementations:" >second_file
    print "namespace b2" >second_file
    print "{" >second_file

    cur_enum_name = ""

    # We don't want those classes.
    forced_non_classes["Vec2"] = 1
    forced_non_classes["Rot"] = 1
    forced_non_classes["AABB"] = 1
}

# Collect enums.

cur_enum_name && /\}/ {
    # Finish the enum.
    cur_enum_name = ""
    next
}

cur_enum_name && /\{/ {next}

cur_enum_name {
    if ($0 !~ /^\s*$/ && $0 !~ /^\s*\/\//)
    {
        # Extract name and value.
        match($0, /^\s*b2_(\w+)(\s*=\s*(\w+))?,?$/, elems)
        if (RLENGTH == -1)
        {
            print "Unable to parse enum element `" $0 "` in `" cur_enum_name "`." >"/dev/stderr"
            exit 1
        }

        enums[cur_enum_name]["elems"][length(enums[cur_enum_name]["elems"])+1] = elems[1]
    }
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

    funcs[func_name]["comment"] = doc_comment
    funcs[func_name]["ret"] = gensub(/\s+$/, "", 1, gensub(/\s*\*\s*/, "* ", "g", elems[1]))

    # Extract individual parameters.
    if (func_param_string !~ /^\s*void\s*$/)
    {
        patsplit(func_param_string, elems, /[^ ,][^,]*[^ ,]/)
        for (i in elems)
        {
            match(elems[i], /^\s*(((const|enum)\s+)*\w+(\s*\*)*)\s*(\w+)*\s*$/, subelems)
            if (RLENGTH == -1)
            {
                print "Failed to parse the parameters for " func_name "." >"/dev/stderr"
                exit 1
            }
            funcs[func_name]["params"][i]["type"] = gensub(/^enum /, "", 1, gensub(/\s+$/, "", 1, gensub(/\s*\*\s*/, "* ", "g", subelems[1])))
            funcs[func_name]["params"][i]["name"] = subelems[5]

            if (funcs[func_name]["params"][i]["type"] == "void")
            {
                print "In " func_name ", why is the parameter type `void`? Param string is `" func_param_string "`" >"/dev/stderr"
                exit 1
            }
        }
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
        if (!(elems[1] in forced_non_classes) && !(elems[1] in classes_set))
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
    if (c == "Chain")
        return 10
    else if (c == "Shape")
        return 11
    else if (c == "Joint")
        return 30
    else if (c ~ /Joint/)
        return 31
    else if (c == "Body")
        return 40
    else
        return 50
}

function sort_classes_comparator(ai, av, bi, bv)
{
    return class_order(av) - class_order(bv)
}

# We use this order to sort our functions, to make them look better.
function func_order(f)
{
    if (f ~ /^b2Create/)
        return 10
    if (f ~ /^b2Destroy/)
        return 11
    if (f ~ /_IsValid$/)
        return 20

    return 30
}

function func_order_2(f)
{
    if (f ~ /^Set/ || f ~ /^Enable/)
        return 10

    return 20
}

function simplify_func_name_for_sorting(f)
{
    return gensub(/^(Get|Set|Is|Enable)/, "", 1, gensub(/^(Are|Is)(.*)Enabled$/, "\\2", 1, f))
}

function sort_funcs_comparator(ai, av, bi, bv,      na, nb, oa, ob)
{
    na = simplify_func_name_for_sorting(funcs[av]["clean_name"])
    nb = simplify_func_name_for_sorting(funcs[bv]["clean_name"])

    oa = func_order(av)
    ob = func_order(bv)
    if (oa != ob)
        return oa - ob

    if (na != nb)
        return na < nb ? -1 : 1

    oa = func_order_2(funcs[av]["clean_name"])
    ob = func_order_2(funcs[bv]["clean_name"])
    if (oa != ob)
        return oa - ob

    if (av != bv)
        return av < bv ? -1 : 1

    return 0
}

# Emits a single function `func_name`.
# `type` is the enclosing class name, or an empty string.
# If `type` is specified and `func_name` doesn't match it, does nothing.
# On success, removes the function from the global list.
# `indent` is some spaces that we prepend to every line.
# `func_variant_index` must initially be 0. We call ourselves recursively, increasing it, to generate some function variants.
function emit_func(func_name, type, func_variant_index, indent)
{
    # If this is a factory function for creating other classes.
    is_factory_func = funcs[func_name]["is_factory"]
    if (is_factory_func && funcs[func_name]["params"][1]["type"] != "b2" type "Id")
        return # This factory function is not for our type.
    # Is this is owning overload of the factory function?
    factory_func_owning = is_factory_func && func_variant_index == 0

    # Whether this is the destruction func.
    is_destroy_func = func_name ~ "b2Destroy" type

    # Ignore functions not from this class.
    if (type && func_name !~ "b2" type "_.*" && !is_factory_func && !is_destroy_func)
        return

    # Figure out the return type.
    return_type = funcs[func_name]["ret"]
    return_type_fixed = funcs[func_name]["ret_fixed"]

    # The cleaned up name
    clean_func_name = funcs[func_name]["clean_name"]

    # Whether this is a getter that should be duplicated to have const and non-const versions.
    is_const_nonconst_getter = return_type_fixed ~ /Ref$/

    variant_return_type = return_type_fixed
    if (is_const_nonconst_getter && func_variant_index == 1)
        variant_return_type = gensub(/Ref$/, "ConstRef", 1, variant_return_type)

    # The comment of this function.
    if (func_variant_index == 0)
    {
        print ""
        printf indent "%s", gensub(/\n/, "\n" indent, "g", funcs[func_name]["comment"])
    }
    else
    {
        printf indent
    }
    printf "    " >second_file

    # Nodiscard?
    if (return_type == "void")
    {}
    else if ((type == "DynamicTree" && func_name == "b2DynamicTree_Rebuild") || return_type == "b2TreeStats")
    {} # This returns optional statistics.
    else if (is_factory_func && !factory_func_owning)
    {} # You don't have to store the shape handle when it's non-owning.
    else
        printf "[[nodiscard]] "

    if (!type)
        printf "inline "

    if (type in classes_id_based)
        printf "template <typename D, bool ForceConst> " >second_file
    else
        printf "inline " >second_file

    printf variant_return_type
    printf variant_return_type >second_file
    if (is_factory_func && !factory_func_owning)
    {
        printf "Ref"
        printf "Ref" >second_file
    }
    printf " " clean_func_name "("

    if (type in classes_id_based)
        printf " Basic" type "Interface<D, ForceConst>::" clean_func_name "(" >second_file
    else
        printf " " type "::" clean_func_name "(" >second_file

    # Parameters.
    first_param = 1
    first_param_is_self = 0
    for (i in funcs[func_name]["params"])
    {
        if (funcs[func_name]["params"][i]["is_callback_context"])
            continue # Skip `void* context` parameter for callbacks.

        param_type = funcs[func_name]["params"][i]["type"]

        if (type && first_param && !first_param_is_self && (is_id_based ? param_type == "b2" base_type_or_self "Id" : param_type ~ "(const )?" type "*"))
        {
            first_param_is_self = 1
            continue # This is the `self` param.
        }

        if (first_param)
        {
            first_param = 0

            # Inject an extra ownership parameter for shape factories.
            if (is_factory_func)
            {
                printf "%s, ", factory_func_owning ? "Tags::OwningHandle" : "Tags::DestroyWithParent"
                printf "%s, ", factory_func_owning ? "Tags::OwningHandle" : "Tags::DestroyWithParent" >second_file
            }
        }
        else
        {
            printf ", "
            printf ", " >second_file
        }

        param_type_fixed = param_type

        if (funcs[func_name]["params"][i]["is_callback"])
        {
            # Adjust callback parameters.
            param_type_fixed = "detail::FuncRef<" gensub(/\*$/, "", 1, param_type) "," (func_variant_index ? "true" : "false") ">"
        }
        else
        {
            # Adjust pointer parameters to references (except for `void *`).
            if (param_type_fixed != "void*" &&
                param_type_fixed != "const char*" &&
                param_type_fixed ~ /\*$/ &&
                param_type_fixed !~ /Fcn\*$/ &&
                (param_type_fixed ~ /^const/ || param_type_fixed == "b2DebugDraw*"))
            {
                param_type_fixed = gensub(/\*$/, "\\&", 1, param_type_fixed)
                funcs[func_name]["params"][i]["ptr_adjusted_to_ref"] = 1
            }

            # Adjust `...Def` structs to our classes.
            if (param_type_fixed ~ /^const b2.*Def&$/)
            {
                param_underlying_class_type = gensub(/^const b2(.*)Def&$/, "\\1", 1, param_type_fixed)
                if (param_underlying_class_type in classes_set)
                    param_type_fixed = "const std::derived_from<b2" param_underlying_class_type "Def> auto&"
            }
        }

        funcs[func_name]["params"][i]["type_fixed"] = param_type_fixed
        printf param_type_fixed " " funcs[func_name]["params"][i]["name"]
        printf param_type_fixed " " funcs[func_name]["params"][i]["name"] >second_file
    }
    printf ")"
    printf ")" >second_file

    # Constness.
    is_const = 0
    if (is_const_nonconst_getter)
    {
        is_const = func_variant_index # A const version of a getter.
    }
    else if (funcs[func_name]["accepts_const_nonconst_callback"])
    {
        is_const = func_variant_index # Accepting const and non-const callbacks.
    }
    else if (!type)
    {
        is_const = 0 # Not a member function.
    }
    else if (is_factory_func || is_destroy_func)
    {
        is_const = 0
    }
    else if (is_id_based)
    {
        # When we don't have a pointer parameter, we have to guess constness from the name.

        if (clean_func_name ~ /^(Get|Is|Compute|Are|Test|Extents|Contains|Union|Center|Cast|RayCast|Collide)($|[A-Z])/ || clean_func_name == "Draw")
            is_const = 1
        else if (clean_func_name ~ /^(Set|Enable|Apply|Disable|Reset|Wake|Create|Destroy|Enlarge|Explode|Dump|Rebuild)($|[A-Z])/ || clean_func_name == "Step")
            is_const = 0
        else
        {
            print "Can't guess from this function name if it's const or not: " clean_func_name >"/dev/stderr"
            exit 1
        }
    }
    else if (is_by_value_raii_wrapper && first_param_is_self)
    {
        if (length(funcs[func_name]["params"]) > 0 && funcs[func_name]["params"][1]["type"] ~ /^const /)
            is_const = 1
        else
            is_const = 0
    }
    else
    {
        print "Not sure if this func is const or not: " func_name >"/dev/stderr"
        exit 1
    }

    if (is_const)
    {
        printf " const"
        printf " const" >second_file
    }
    else if (is_id_based)
    {
        printf " /*non-const*/ requires (!ForceConst)"
        printf " requires (!ForceConst)" >second_file
    }

    # Function body.

    print ";"

    printf " { " >second_file

    if (is_destroy_func)
        printf "if (*this) { " >second_file # Intentionally not checking `IsOwner()` to allow destroying through non-owning objects.

    if (return_type != "void" && !factory_func_owning)
        printf "return " >second_file

    if (factory_func_owning)
        printf return_type_fixed " ret; ret.id = " >second_file
    else if (is_factory_func && !factory_func_owning && return_type_fixed ~ /^\w+Joint$/)
        printf "(" return_type_fixed "Ref)" >second_file
    # Cast return value to our enum if needed.
    if (return_type_fixed in enums)
        printf "(" return_type_fixed ")" >second_file

    printf func_name "(" >second_file

    first_param = 1
    for (i in funcs[func_name]["params"])
    {
        if (first_param && first_param_is_self)
        {
            if (is_by_value_raii_wrapper)
                printf "&value" >second_file
            else
                printf "static_cast<const D &>(*this).Handle()" >second_file
            first_param = 0
            continue
        }

        if (funcs[func_name]["params"][i]["is_callback_context"])
            continue # Skip passing `void *` to callbacks.

        if (first_param)
            first_param = 0
        else
            printf ", " >second_file

        param_type_fixed = funcs[func_name]["params"][i]["type_fixed"]
        param_name = funcs[func_name]["params"][i]["name"]

        if (funcs[func_name]["params"][i]["is_callback"])
        {
            printf "%s.GetFunc(), %s.GetContext()", param_name, param_name >second_file
            continue
        }

        # Prepend `&` to take address of a reference.
        if (funcs[func_name]["params"][i]["ptr_adjusted_to_ref"])
            printf "&" >second_file


        # Cast our enums to the original enums.
        if (param_type_fixed in enums)
            printf "(b2" param_type_fixed ")" >second_file

        printf "%s", param_name >second_file
    }
    printf ")" >second_file
    if (factory_func_owning)
        printf "; return ret" >second_file
    printf ";" >second_file
    if (is_destroy_func)
        printf " static_cast<D &>(*this).id = {}; }" >second_file
    print " }" >second_file

    if (func_variant_index == 0 && (is_factory_func || is_const_nonconst_getter || funcs[func_name]["accepts_const_nonconst_callback"]))
        emit_func(func_name, type, func_variant_index + 1, indent)

    # Destroy the function we just generated.
    delete funcs[func_name]
}

END {
    # An extra analysis pass for functions.
    for (func_name in funcs)
    {
        # If this is a factory function for creating other classes.
        funcs[func_name]["is_factory"] = func_name ~ /^b2Create/ && length(funcs[func_name]["params"]) > 0 && funcs[func_name]["params"][1]["type"] ~ /^b2.*Id$/

        # Figure out the fixed return type.
        return_type_fixed = funcs[func_name]["ret"]

        if (funcs[func_name]["is_factory"])
        {
            factory_target_class = gensub(/^b2Create/, "", 1, func_name)
            if (factory_target_class in classes_set)
                return_type_fixed = factory_target_class
            else
                return_type_fixed = gensub(/^b2(.*)Id$/, "\\1", 1, return_type_fixed)
        }
        else
        {
            # Adjust IDs to our wrappers.
            candidate_class_name = gensub(/^b2(.*)Id$/, "\\1", 1, return_type_fixed)
            if (candidate_class_name in classes_set)
                return_type_fixed = candidate_class_name "Ref"
        }

        funcs[func_name]["ret_fixed"] = return_type_fixed

        # Figure out the fixed name.

        # Remove class name from the func name.
        clean_func_name = func_name
        if (func_name ~ /^b2Destroy/)
            clean_func_name = "Destroy"
        else if (func_name ~ /_/ && !funcs[func_name]["is_factory"])
            clean_func_name = gensub("^b2.*_", "", 1, clean_func_name)
        else
            clean_func_name = gensub("^b2", "", 1, clean_func_name)

        # Overload factory funcs and setters to the same name.
        if (clean_func_name ~ /^Create.*Joint$/)
            clean_func_name = gensub("^Create.+Joint$", "CreateJoint", 1, clean_func_name)
        else if (clean_func_name ~ /^Create.*Shape$/)
            clean_func_name = gensub("^Create.+Shape$", "CreateShape", 1, clean_func_name)
        else if (clean_func_name ~ /Set(Capsule|Polygon|Circle|Segment)/)
            clean_func_name = "Set"
        else if (clean_func_name ~ /^Overlap.*/)
            clean_func_name = "Overlap"
        else if (clean_func_name ~ /^Cast/ && !(clean_func_name ~ /CastRay/) && !(clean_func_name ~ /CastMover/))
            clean_func_name = "Cast"

        funcs[func_name]["clean_name"] = clean_func_name

        # Callback handling.
        funcs[func_name]["accepts_const_nonconst_callback"] = 0 # Whether this function must be duplicated for const and nonconst callback argument.
        skip_context_param = 0
        for (i in funcs[func_name]["params"])
        {
            param_type = funcs[func_name]["params"][i]["type"]
            funcs[func_name]["params"][i]["is_callback"] = 0
            funcs[func_name]["params"][i]["is_callback_context"] = skip_context_param
            skip_context_param = 0
            if (param_type ~ /^(b2CastResultFcn|b2OverlapResultFcn)\*$/)
            {
                funcs[func_name]["params"][i]["is_callback"] = 1
                funcs[func_name]["accepts_const_nonconst_callback"] = 1
                skip_context_param = 1
            }
        }
    }

    # Make a sorted list of functions.
    split("", sorted_funcs);
    for (func_name in funcs)
        sorted_funcs[length(sorted_funcs)+1] = func_name
    asort(sorted_funcs, sorted_funcs, "sort_funcs_comparator")

    # An extra analysis pass for classes.
    for (i in classes)
    {
        type = classes[i]

        # Is a class derived from `Joint`?
        if (type ~ /.+Joint/)
            classes_joint_kinds[type] = 1

        # Those store a box2d struct by value, and act as a RAII wrapper.
        if (type == "DynamicTree")
            classes_by_value_raii_wrappers[type] = 1
        # Those just inherit from the original struct and add some member functions.
        if (!(type in classes_by_value_raii_wrappers) && ("b2Destroy" type in funcs || type in classes_joint_kinds))
            classes_id_based[type] = 1
    }

    # Emit our own helpers for classes.

    print "    namespace Tags" # Using a namespace to stop Clangd from autocompleting the tags! >:o
    print "    {"
    print "        struct OwningHandle { explicit OwningHandle() = default; };"
    print "        struct DestroyWithParent { explicit DestroyWithParent() = default; };"
    print ""
    print "        template <typename T>"
    print "        concept OwnershipTag = std::same_as<T, OwningHandle> || std::same_as<T, DestroyWithParent>;"
    print "    }"
    print ""
    print "    // Pass to `Create...()` if you want the resulting object to destroy this thing in its destructor."
    print "    inline constexpr Tags::OwningHandle OwningHandle{};"
    print ""
    print "    // Pass to `Create...()` if you want the parent object (that you're calling this on) to destroy this thing in its destructor. You'll receive a non-owning reference."
    print "    inline constexpr Tags::DestroyWithParent DestroyWithParent{};"
    print ""
    print ""

    # Forward-declare classes.
    for (i in classes)
    {
        type = classes[i]

        if (type in classes_id_based)
        {
            print "    template <bool IsConstRef> class MaybeConst"type"Ref;"
            print "    using "type"Ref = MaybeConst"type"Ref<false>;"
            print "    using "type"ConstRef = MaybeConst"type"Ref<true>;"
        }
    }

    print ""

    # The `detail` namespace with various helpers.

    print "    namespace detail"
    print "    {"
    print "        // Map box2d IDs to our reference types. Specific joint types aren't there, since they don't have their own ID types."
    print "        template <typename T, bool IsConst> struct Box2dIdToRef { static constexpr bool IsIdType = false; };"
    for (i in classes)
    {
        type = classes[i]

        if (type in classes_id_based && !(type in classes_joint_kinds))
            print "        template <bool IsConst> struct Box2dIdToRef<b2"type"Id, IsConst> { static constexpr bool IsIdType = true; using type = MaybeConst"type"Ref<IsConst>; };"
    }
    print ""
    system("cat generator/detail.cpp")
    print "    }"

    print ""

    # Emit classes.
    asort(classes, classes, "sort_classes_comparator")
    for (i in classes)
    {
        type = classes[i]

        print ""

        # Is a class derived from `Joint`?
        is_joint_kind = type in classes_joint_kinds
        if (is_joint_kind)
            base_type_or_self = "Joint"
        else
            base_type_or_self = type

        # Those store a box2d struct by value, and act as a RAII wrapper.
        is_by_value_raii_wrapper = type in classes_by_value_raii_wrappers
        # Those just inherit from the original struct and add some member functions.
        is_id_based = type in classes_id_based

        if (!is_by_value_raii_wrapper && !is_id_based)
        {
            print "Unsure what to do with this class: " type >"/dev/stderr"
            exit 1
        }

        if (is_id_based)
        {
            # Has public constructors?
            public_constructible = ("b2Create" type in funcs);

            # Has a struct with parameters?
            has_params_struct = ("b2Default" type "Def") in funcs;

            # The joint enum value for this joint kind.
            if (is_joint_kind)
                joint_enum_value = "b2_" tolower(substr(type, 1, 1)) substr(type, 2)

            # Class head.
            printf "    template <typename D, bool ForceConst>\n    class Basic"type"Interface"
            print ""
            # Body.
            print "    {"
            print "      protected:"
            print "        Basic"type"Interface() = default;"
            print ""
            printf "      public:"

            # ID operations.
            if (!is_joint_kind)
            {
                print ""
                print "        [[nodiscard]] explicit operator bool() const { return B2_IS_NON_NULL(Handle()); }"
                print "        [[nodiscard]] const b2" type "Id &Handle() const { return static_cast<const D &>(*this).id; }"
                print "        // Convert to a handle. Using a `same_as` template to prevent implicit madness. In particular, to prevent const-to-non-const conversions between non-owning wrappers."

                print "        // I want this to return a const reference, but GCC 13 and earlier choke on that (fixed in 14). GCC bug: https://gcc.gnu.org/bugzilla/show_bug.cgi?id=61663"
                print "        template <std::same_as<b2"type"Id> T> [[nodiscard]] operator T() const { return Handle(); }"
                #print "        template <std::same_as<b2"type"Id> T> [[nodiscard]] operator const T &() const { return Handle(); }"
            }

            # Expose all the functions.
            for (i in sorted_funcs)
            {
                func_name = sorted_funcs[i]
                if (func_name in funcs) # The function could've been deleted.
                    emit_func(func_name, type, 0, "        ")
            }

            # Close the class.
            print "    };"

            # Now the implementations:

            # The owning (non-reference) version.
            print ""

            # Primary class comment. Snatch it from the `...Def` struct because those have the best comments.
            printf "    %s", typedef_structs[typedef_structs_set[type "Def"]]["comment"]

            printf "class "type" : "
            if (is_joint_kind)
                printf "public "base_type_or_self", "
            print "public Basic"type"Interface<"type", false>"
            print "    {"
            print "        template <typename, bool>"
            print "        friend class Basic"type"Interface;"
            # Joint needs to friend all the derived joints for downcasts.
            if (type == "Joint")
            {
                for (other_type in classes_joint_kinds)
                {
                    print "        friend class "other_type";"
                }
            }
            # Emit extra friends (for `Create...` functions).
            split("", this_class_friends)
            for (func_name in funcs)
            {
                if (funcs[func_name]["is_factory"] && funcs[func_name]["ret_fixed"] == type)
                {
                    this_friend = gensub(/^b2(.*)Id$/, "\\1", 1, funcs[func_name]["params"][1]["type"])
                    if (!(this_friend in this_class_friends))
                    {
                        this_class_friends[this_friend] = 1
                        print "        template <typename, bool>\n        friend class Basic" this_friend "Interface;"
                    }
                }
            }
            print ""
            if (!is_joint_kind)
            {
                print "      protected:"
                print "        b2"type"Id id{};" # Not using `b2_null"type "Id` for simplicity and constexpr-ness, here and everywhere else.
                print ""
            }
            print "      public:"
            print "        static constexpr bool IsOwning = true;"
            print ""
            print "        // Constructs a null (invalid) object."
            print "        constexpr "type"() noexcept {}"
            # Params struct.
            if (has_params_struct)
            {
                print ""
                print "        // The constructor accepts either this or directly `b2" type "Def`."
                print "        struct Params : b2" type "Def"
                print "        {"
                print "            Params() : b2" type "Def(b2Default" type "Def()) {}"
                print "        };"

                delete funcs["b2Default" type "Def"]
            }
            # The parametrized constructor.
            has_parametrized_ctor = 0
            if (has_params_struct && type == "World") # Only the `World` class can self-construct, others are constructed by other classes.
            {
                has_parametrized_ctor = 1

                print ""
                printf "        %s", gensub(/\n/, "\n        ", "g", funcs["b2Create" type]["comment"])
                print type "(const std::derived_from<b2" type "Def> auto &params) { id = b2Create" type "(&params); }"

                delete funcs["b2Create" type]
            }

            # Downcast.
            if (is_joint_kind)
            {
                print ""
                print "        // Downcast from a generic joint."
                print "        // Triggers an assertion if this isn't the right joint kind."
                print "        explicit "type"(Joint&& other) noexcept"
                print "        {"
                print "            if (!other || other.GetType() == "joint_enum_value")"
                print "                this->id = std::exchange(other.id, {});"
                print "            else"
                print "                BOX2CPP_ASSERT(false && \"This joint is not a `"type"`.\");"
                print "        }"
            }

            if (!is_joint_kind)
            {
                # Copy/move ctors.
                print ""
                print "        " type "(" type "&& other) noexcept { id = std::exchange(other.id, b2"type"Id{}); }"
                print "        " type "& operator=(" type " other) noexcept { std::swap(id, other.id); return *this; }"

                # Destructor.
                dtor_args = ""
                dtor_comment = ""
                if (type == "Shape")
                {
                    dtor_args = "true"
                    dtor_comment = " // Update mass by default. Call `Destroy(false)` manually if you don't want this."
                }
                print ""
                print "        ~" type "() { if (*this) Destroy(" dtor_args "); }" dtor_comment
            }

            print "    };"

            # The reference version.
            print ""
            print "    template <bool IsConstRef>"
            printf "    class MaybeConst"type"Ref : "
            if (is_joint_kind)
                printf "public MaybeConst"base_type_or_self"Ref<IsConstRef>, "
            print "public Basic"type"Interface<"type"Ref, IsConstRef>"
            print "    {"
            print "        template <typename, bool>"
            print "        friend class Basic"type"Interface;"
            print ""
            if (!is_joint_kind)
            {
                print "      protected:"
                print "        b2" type "Id id{};" # Not using `b2_null"type "Id` for simplicity and constexpr-ness, here and everywhere else.
                print ""
            }
            print "      public:"
            print "        static constexpr bool IsOwning = false;"
            print "        static constexpr bool IsConst = IsConstRef;"
            print ""
            print "        // Constructs a null (invalid) object."
            print "        constexpr MaybeConst"type"Ref() noexcept {}"
            print ""
            # Constructor from handle.
            print "        // Point to an existing handle."
            print "        // Using a `same_as` template to prevent implicit madness. In particular, to prevent const-to-non-const conversions between non-owning wrappers."
            if (is_joint_kind)
            {
                print "        // Downcast from a generic joint reference (or owning joint)."
                print "        // Triggers an assertion if this isn't the right joint kind."
                print "        explicit constexpr MaybeConst"type"Ref(std::same_as<b2JointId> auto id) noexcept"
                print "        {"
                print "            if (B2_IS_NULL(id) || b2Joint_GetType(id) == "joint_enum_value")"
                print "                this->id = id;"
                print "            else"
                print "                BOX2CPP_ASSERT(false && \"This joint is not a `"type"`.\");"
                print "        }"
            }
            else
            {
                print "        constexpr MaybeConst"type"Ref(std::same_as<b2" type "Id> auto id) noexcept { this->id = id; }"
            }
            print ""
            # Create from a non-reference.
            print "        // Create from a non-reference."
            print "        constexpr MaybeConst"type"Ref(const "type"& other) noexcept : MaybeConst"type"Ref(other.Handle()) {}"
            print ""
            if (is_joint_kind)
            {
                # Downcast from a reference or an owning object.
                print "        // Triggers an assertion if this isn't the right joint kind."
                print "        explicit constexpr MaybeConst"type"Ref(MaybeConstJointRef<IsConstRef> other) noexcept : MaybeConst"type"Ref(other.Handle()) {}"
            }
            # Convert non-const reference to const.
            print "        // Convert a non-const reference to a const reference."
            print "        constexpr MaybeConst"type"Ref(const MaybeConst"type"Ref<!IsConstRef>& other) noexcept requires IsConstRef : MaybeConst"type"Ref(other.Handle()) {}"
            print "    };"
        }
        else if (is_by_value_raii_wrapper)
        {
            # Primary class comment.
            printf "    %s", typedef_structs[typedef_structs_set[type]]["comment"]

            # Class head.
            printf "class " type
            # Base classes.
            if (is_joint_kind)
                printf " : public Joint"
            print ""
            print "    {"

            # Member variables.
            print "        b2" type " value{};"
            print ""

            # Public members...
            print "      public:"

            # Default ctor.
            print "        // Consturcts a null (invalid) object."
            print "        constexpr " type "() {}"
            print ""

            # Parametrized ctor.
            printf "        %s", gensub(/\n/, "\n        ", "g", funcs["b2" type "_Create"]["comment"])
            print type "(std::nullptr_t) : value(b2" type "_Create()) {}"
            delete funcs["b2" type "_Create"]

            # Copy/move ctors.
            if (type == "DynamicTree")
            {
                print ""
                print "        " type "(" type "&& other) noexcept : value(other.value) { other.value = {}; }"
                print "        " type "& operator=(" type "&& other) noexcept"
                print "        {"
                print "            if (this == &other) return *this;"
                print "            if (*this) b2" type "_Destroy(&value);"
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
            print ""
            printf "        %s", gensub(/\n/, "\n        ", "g", funcs["b2" type "_Destroy"]["comment"])
            print "~" type "() { if (*this) b2" type "_Destroy(&value); }"

            delete funcs["b2" type "_Destroy"]

            # Some custom members.
            if (type == "DynamicTree")
            {
                print ""
                print "        [[nodiscard]] explicit operator bool() const { return bool( value.nodes ); }"
                print "        [[nodiscard]]       b2DynamicTree *RawTreePtr()       { return *this ? &value : nullptr; }"
                print "        [[nodiscard]] const b2DynamicTree *RawTreePtr() const { return *this ? &value : nullptr; }"
            }

            # Expose all the functions.
            for (i in sorted_funcs)
            {
                func_name = sorted_funcs[i]
                if (func_name in funcs) # The function could've been deleted.
                    emit_func(func_name, type, 0, "        ")
            }

            # Close the class.
            print "    };"
        }
    }

    print "} // namespace box2d"
    print "} // namespace box2d" >second_file
}
