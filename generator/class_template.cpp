    class @
    {
        b2@Id id = b2_null@Id;

      public:
        constexpr @() {}

        struct Params : b2@Def
        {
            Params() : b2@Def(b2Default@Def()) {}
        };
        @(const Params &params) : id(b2Create@(&params)) {throw std::runtime_error("Creating box2c TYPE failed.");}

        @(@ &&other) noexcept : id(std::exchange(other.id, b2_null@Id)) {}
        @ &operator=(@ other) noexcept {std::swap(id, other.id); return *this;}

        ~@() {if (*this) b2Destroy@(id);}

        [[nodiscard]] explicit operator bool() const {return B2_IS_NON_NULL(id);}
        [[nodiscard]] const b2@Id &Handle() const {return id;}
    };
