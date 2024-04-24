        // Adjust one of the callback arguments, replacing raw box2d IDs with our reference types.
        template <bool IsConst, typename T>
        T &&AdjustCallbackArg(T &&arg) {return std::forward<T>(arg);}
        template <bool IsConst, typename T> requires Box2dIdToRef<std::remove_cvref_t<T>, IsConst>::IsIdType
        typename Box2dIdToRef<std::remove_cvref_t<T>, IsConst>::type AdjustCallbackArg(T &&arg) {return arg;}

        // Given a function type `R(P..., void *)`, returns `R(P...)`. Otherwise causes a hard error.
        template <typename T, typename U = void>
        struct StripTrailingVoidPtrParam;
        template <typename R, typename ...P>
        struct StripTrailingVoidPtrParam<R(P...), void>
        {
            using type = typename StripTrailingVoidPtrParam<R(P...), R()>::type;
        };
        template <typename R, typename P0, typename P1, typename ...P, typename ...Q>
        struct StripTrailingVoidPtrParam<R(P0, P1, P...), R(Q...)>
        {
            using type = typename StripTrailingVoidPtrParam<R(P1, P...), R(Q..., P0)>::type;
        };
        template <typename R, typename ...Q>
        struct StripTrailingVoidPtrParam<R(void *), R(Q...)>
        {
            using type = R(Q...);
        };

        // Helper for `FuncRef` below.
        template <typename T, bool IsConst>
        class FuncRefBase {};
        template <typename R, typename ...P, bool IsConst>
        class FuncRefBase<R(P...), IsConst>
        {
            using CallerType = R (*)(P..., void *);
            CallerType caller = nullptr;
            void *context = nullptr;

          public:
            template <typename F>
            requires std::same_as<std::invoke_result_t<F &&, decltype((AdjustCallbackArg<IsConst>)(std::declval<P>()))...>, R>
            constexpr FuncRefBase(F &&func)
                : caller([](P ...params, void *context) -> R {return std::invoke(std::forward<F>(*static_cast<std::remove_cvref_t<F> *>(context)), (AdjustCallbackArg<IsConst>)(std::forward<P>(params))...);}),
                context(&func)
            {}

            [[nodiscard]] constexpr CallerType GetFunc() const {return caller;}
            [[nodiscard]] constexpr void *GetContext() const {return context;}
        };

        // Acts as `std::function_ref` for our callbacks. Converts the incoming functors into a C-style pointer plus a `void *`.
        template <typename T, bool IsConst>
        class FuncRef : public detail::FuncRefBase<typename detail::StripTrailingVoidPtrParam<T>::type, IsConst>
        {
            using base = detail::FuncRefBase<typename detail::StripTrailingVoidPtrParam<T>::type, IsConst>;
            using base::base;
        };
