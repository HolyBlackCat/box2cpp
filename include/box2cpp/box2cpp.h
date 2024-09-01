#pragma once

// box2cpp, C++ bindings for box2d 3.x
// Generated from box2d commit: 8e7a17c 2024-08-11
// Generator version: 0.7

#include <box2d/box2d.h>

#include <concepts>
#include <cstddef>
#include <functional>
#include <type_traits>
#include <utility>

#ifndef BOX2CPP_ASSERT
#include <cassert>
#define BOX2CPP_ASSERT(...) assert(__VA_ARGS__)
#endif

namespace b2
{
    namespace Tags
    {
        struct OwningHandle { explicit OwningHandle() = default; };
        struct DestroyWithParent { explicit DestroyWithParent() = default; };

        template <typename T>
        concept OwnershipTag = std::same_as<T, OwningHandle> || std::same_as<T, DestroyWithParent>;
    }

    // Pass to `Create...()` if you want the resulting object to destroy this thing in its destructor.
    inline constexpr Tags::OwningHandle OwningHandle{};

    // Pass to `Create...()` if you want the parent object (that you're calling this on) to destroy this thing in its destructor. You'll receive a non-owning reference.
    inline constexpr Tags::DestroyWithParent DestroyWithParent{};


    template <bool IsConstRef> class MaybeConstWorldRef;
    using WorldRef = MaybeConstWorldRef<false>;
    using WorldConstRef = MaybeConstWorldRef<true>;
    template <bool IsConstRef> class MaybeConstBodyRef;
    using BodyRef = MaybeConstBodyRef<false>;
    using BodyConstRef = MaybeConstBodyRef<true>;
    template <bool IsConstRef> class MaybeConstShapeRef;
    using ShapeRef = MaybeConstShapeRef<false>;
    using ShapeConstRef = MaybeConstShapeRef<true>;
    template <bool IsConstRef> class MaybeConstChainRef;
    using ChainRef = MaybeConstChainRef<false>;
    using ChainConstRef = MaybeConstChainRef<true>;
    template <bool IsConstRef> class MaybeConstJointRef;
    using JointRef = MaybeConstJointRef<false>;
    using JointConstRef = MaybeConstJointRef<true>;
    template <bool IsConstRef> class MaybeConstDistanceJointRef;
    using DistanceJointRef = MaybeConstDistanceJointRef<false>;
    using DistanceJointConstRef = MaybeConstDistanceJointRef<true>;
    template <bool IsConstRef> class MaybeConstMotorJointRef;
    using MotorJointRef = MaybeConstMotorJointRef<false>;
    using MotorJointConstRef = MaybeConstMotorJointRef<true>;
    template <bool IsConstRef> class MaybeConstMouseJointRef;
    using MouseJointRef = MaybeConstMouseJointRef<false>;
    using MouseJointConstRef = MaybeConstMouseJointRef<true>;
    template <bool IsConstRef> class MaybeConstPrismaticJointRef;
    using PrismaticJointRef = MaybeConstPrismaticJointRef<false>;
    using PrismaticJointConstRef = MaybeConstPrismaticJointRef<true>;
    template <bool IsConstRef> class MaybeConstRevoluteJointRef;
    using RevoluteJointRef = MaybeConstRevoluteJointRef<false>;
    using RevoluteJointConstRef = MaybeConstRevoluteJointRef<true>;
    template <bool IsConstRef> class MaybeConstWeldJointRef;
    using WeldJointRef = MaybeConstWeldJointRef<false>;
    using WeldJointConstRef = MaybeConstWeldJointRef<true>;
    template <bool IsConstRef> class MaybeConstWheelJointRef;
    using WheelJointRef = MaybeConstWheelJointRef<false>;
    using WheelJointConstRef = MaybeConstWheelJointRef<true>;

    namespace detail
    {
        // Map box2d IDs to our reference types. Specific joint types aren't there, since they don't have their own ID types.
        template <typename T, bool IsConst> struct Box2dIdToRef { static constexpr bool IsIdType = false; };
        template <bool IsConst> struct Box2dIdToRef<b2WorldId, IsConst> { static constexpr bool IsIdType = true; using type = MaybeConstWorldRef<IsConst>; };
        template <bool IsConst> struct Box2dIdToRef<b2BodyId, IsConst> { static constexpr bool IsIdType = true; using type = MaybeConstBodyRef<IsConst>; };
        template <bool IsConst> struct Box2dIdToRef<b2ShapeId, IsConst> { static constexpr bool IsIdType = true; using type = MaybeConstShapeRef<IsConst>; };
        template <bool IsConst> struct Box2dIdToRef<b2ChainId, IsConst> { static constexpr bool IsIdType = true; using type = MaybeConstChainRef<IsConst>; };
        template <bool IsConst> struct Box2dIdToRef<b2JointId, IsConst> { static constexpr bool IsIdType = true; using type = MaybeConstJointRef<IsConst>; };

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
    }


    template <typename D, bool ForceConst>
    class BasicChainInterface
    {
      protected:
        BasicChainInterface() = default;

      public:
        [[nodiscard]] explicit operator bool() const { return B2_IS_NON_NULL(Handle()); }
        [[nodiscard]] const b2ChainId &Handle() const { return static_cast<const D &>(*this).id; }
        // Convert to a handle. Using a `same_as` template to prevent implicit madness. In particular, to prevent const-to-non-const conversions between non-owning wrappers.
        // I want this to return a const reference, but GCC 13 and earlier choke on that (fixed in 14). GCC bug: https://gcc.gnu.org/bugzilla/show_bug.cgi?id=61663
        template <std::same_as<b2ChainId> T> [[nodiscard]] operator T() const { return Handle(); }

        /// Destroy a chain shape
        void Destroy() /*non-const*/ requires (!ForceConst);

        /// Chain identifier validation. Provides validation for up to 64K allocations.
        [[nodiscard]] bool IsValid() const;

        /// Set the chain friction
        /// @see b2ChainDef::friction
        void SetFriction(float friction) /*non-const*/ requires (!ForceConst);

        /// Set the chain restitution (bounciness)
        /// @see b2ChainDef::restitution
        void SetRestitution(float restitution) /*non-const*/ requires (!ForceConst);
    };

    /// Used to create a chain of edges. This is designed to eliminate ghost collisions with some limitations.
    ///	- chains are one-sided
    ///	- chains have no mass and should be used on static bodies
    ///	- chains have a counter-clockwise winding order
    ///	- chains are either a loop or open
    /// - a chain must have at least 4 points
    ///	- the distance between any two points must be greater than b2_linearSlop
    ///	- a chain shape should not self intersect (this is not validated)
    ///	- an open chain shape has NO COLLISION on the first and final edge
    ///	- you may overlap two open chains on their first three and/or last three points to get smooth collision
    ///	- a chain shape creates multiple smooth edges shapes on the body
    /// https://en.wikipedia.org/wiki/Polygonal_chain
    /// Must be initialized using b2DefaultChainDef().
    ///	@warning Do not use chain shapes unless you understand the limitations. This is an advanced feature.
    /// @ingroup shape
    class Chain : public BasicChainInterface<Chain, false>
    {
        template <typename, bool>
        friend class BasicChainInterface;
        template <typename, bool>
        friend class BasicBodyInterface;

      protected:
        b2ChainId id{};

      public:
        static constexpr bool IsOwning = true;

        // Constructs a null (invalid) object.
        constexpr Chain() noexcept {}

        // The constructor accepts either this or directly `b2ChainDef`.
        struct Params : b2ChainDef
        {
            Params() : b2ChainDef(b2DefaultChainDef()) {}
        };

        Chain(Chain&& other) noexcept { id = std::exchange(other.id, b2ChainId{}); }
        Chain& operator=(Chain other) noexcept { std::swap(id, other.id); return *this; }

        ~Chain() { if (*this) Destroy(); }
    };

    template <bool IsConstRef>
    class MaybeConstChainRef : public BasicChainInterface<ChainRef, IsConstRef>
    {
        template <typename, bool>
        friend class BasicChainInterface;

      protected:
        b2ChainId id{};

      public:
        static constexpr bool IsOwning = false;
        static constexpr bool IsConst = IsConstRef;

        // Constructs a null (invalid) object.
        constexpr MaybeConstChainRef() noexcept {}

        // Point to an existing handle.
        // Using a `same_as` template to prevent implicit madness. In particular, to prevent const-to-non-const conversions between non-owning wrappers.
        constexpr MaybeConstChainRef(std::same_as<b2ChainId> auto id) noexcept { this->id = id; }

        // Create from a non-reference.
        constexpr MaybeConstChainRef(const Chain& other) noexcept : MaybeConstChainRef(other.Handle()) {}

        // Convert a non-const reference to a const reference.
        constexpr MaybeConstChainRef(const MaybeConstChainRef<!IsConstRef>& other) noexcept requires IsConstRef : MaybeConstChainRef(other.Handle()) {}
    };

    template <typename D, bool ForceConst>
    class BasicShapeInterface
    {
      protected:
        BasicShapeInterface() = default;

      public:
        [[nodiscard]] explicit operator bool() const { return B2_IS_NON_NULL(Handle()); }
        [[nodiscard]] const b2ShapeId &Handle() const { return static_cast<const D &>(*this).id; }
        // Convert to a handle. Using a `same_as` template to prevent implicit madness. In particular, to prevent const-to-non-const conversions between non-owning wrappers.
        // I want this to return a const reference, but GCC 13 and earlier choke on that (fixed in 14). GCC bug: https://gcc.gnu.org/bugzilla/show_bug.cgi?id=61663
        template <std::same_as<b2ShapeId> T> [[nodiscard]] operator T() const { return Handle(); }

        /// Destroy a shape
        void Destroy() /*non-const*/ requires (!ForceConst);

        /// Shape identifier validation. Provides validation for up to 64K allocations.
        [[nodiscard]] bool IsValid() const;

        /// Allows you to change a shape to be a capsule or update the current capsule.
        /// This does not modify the mass properties.
        ///	@see b2Body_ApplyMassFromShapes
        void Set(const b2Capsule& capsule) /*non-const*/ requires (!ForceConst);

        /// Allows you to change a shape to be a circle or update the current circle.
        /// This does not modify the mass properties.
        ///	@see b2Body_ApplyMassFromShapes
        void Set(const b2Circle& circle) /*non-const*/ requires (!ForceConst);

        /// Allows you to change a shape to be a polygon or update the current polygon.
        /// This does not modify the mass properties.
        ///	@see b2Body_ApplyMassFromShapes
        void Set(const b2Polygon& polygon) /*non-const*/ requires (!ForceConst);

        /// Allows you to change a shape to be a segment or update the current segment.
        void Set(const b2Segment& segment) /*non-const*/ requires (!ForceConst);

        /// Get the current world AABB
        [[nodiscard]] b2AABB GetAABB() const;

        /// Get the id of the body that a shape is attached to
        [[nodiscard]] BodyRef GetBody() /*non-const*/ requires (!ForceConst);
        [[nodiscard]] BodyConstRef GetBody() const;

        /// Get a copy of the shape's capsule. Asserts the type is correct.
        [[nodiscard]] b2Capsule GetCapsule() const;

        /// Get a copy of the shape's circle. Asserts the type is correct.
        [[nodiscard]] b2Circle GetCircle() const;

        /// Get the closest point on a shape to a target point. Target and result are in world space.
        [[nodiscard]] b2Vec2 GetClosestPoint(b2Vec2 target) const;

        /// Get the maximum capacity required for retrieving all the touching contacts on a shape
        [[nodiscard]] int GetContactCapacity() const;

        /// Get the touching contact data for a shape. The provided shapeId will be either shapeIdA or shapeIdB on the contact data.
        [[nodiscard]] int GetContactData(b2ContactData& contactData, int capacity) const;

        /// Enable contact events for this shape. Only applies to kinematic and dynamic bodies. Ignored for sensors.
        ///	@see b2ShapeDef::enableContactEvents
        void EnableContactEvents(bool flag) /*non-const*/ requires (!ForceConst);

        /// Returns true if contact events are enabled
        [[nodiscard]] bool AreContactEventsEnabled() const;

        /// Set the mass density of a shape, typically in kg/m^2.
        ///	This will not update the mass properties on the parent body.
        ///	@see b2ShapeDef::density, b2Body_ApplyMassFromShapes
        void SetDensity(float density) /*non-const*/ requires (!ForceConst);

        /// Get the density of a shape, typically in kg/m^2
        [[nodiscard]] float GetDensity() const;

        /// Set the current filter. This is almost as expensive as recreating the shape.
        ///	@see b2ShapeDef::filter
        void SetFilter(b2Filter filter) /*non-const*/ requires (!ForceConst);

        /// Get the shape filter
        [[nodiscard]] b2Filter GetFilter() const;

        /// Set the friction on a shape
        ///	@see b2ShapeDef::friction
        void SetFriction(float friction) /*non-const*/ requires (!ForceConst);

        /// Get the friction of a shape
        [[nodiscard]] float GetFriction() const;

        /// Enable contact hit events for this shape. Ignored for sensors.
        ///	@see b2WorldDef.hitEventThreshold
        void EnableHitEvents(bool flag) /*non-const*/ requires (!ForceConst);

        /// Returns true if hit events are enabled
        [[nodiscard]] bool AreHitEventsEnabled() const;

        /// Get the parent chain id if the shape type is b2_smoothSegmentShape, otherwise
        /// returns b2_nullChainId.
        [[nodiscard]] ChainRef GetParentChain() /*non-const*/ requires (!ForceConst);
        [[nodiscard]] ChainConstRef GetParentChain() const;

        /// Get a copy of the shape's convex polygon. Asserts the type is correct.
        [[nodiscard]] b2Polygon GetPolygon() const;

        /// Enable pre-solve contact events for this shape. Only applies to dynamic bodies. These are expensive
        ///	and must be carefully handled due to multithreading. Ignored for sensors.
        ///	@see b2PreSolveFcn
        void EnablePreSolveEvents(bool flag) /*non-const*/ requires (!ForceConst);

        /// Returns true if pre-solve events are enabled
        [[nodiscard]] bool ArePreSolveEventsEnabled() const;

        /// Ray cast a shape directly
        [[nodiscard]] b2CastOutput RayCast(b2Vec2 origin, b2Vec2 translation) const;

        /// Set the shape restitution (bounciness)
        ///	@see b2ShapeDef::restitution
        void SetRestitution(float restitution) /*non-const*/ requires (!ForceConst);

        /// Get the shape restitution
        [[nodiscard]] float GetRestitution() const;

        /// Get a copy of the shape's line segment. Asserts the type is correct.
        [[nodiscard]] b2Segment GetSegment() const;

        /// Returns true If the shape is a sensor
        [[nodiscard]] bool IsSensor() const;

        /// Enable sensor events for this shape. Only applies to kinematic and dynamic bodies. Ignored for sensors.
        ///	@see b2ShapeDef::isSensor
        void EnableSensorEvents(bool flag) /*non-const*/ requires (!ForceConst);

        /// Returns true if sensor events are enabled
        [[nodiscard]] bool AreSensorEventsEnabled() const;

        /// Get a copy of the shape's smooth line segment. These come from chain shapes.
        /// Asserts the type is correct.
        [[nodiscard]] b2SmoothSegment GetSmoothSegment() const;

        /// Test a point for overlap with a shape
        [[nodiscard]] bool TestPoint(b2Vec2 point) const;

        /// Get the type of a shape
        [[nodiscard]] b2ShapeType GetType() const;

        /// Set the user data for a shape
        void SetUserData(void* userData) /*non-const*/ requires (!ForceConst);

        /// Get the user data for a shape. This is useful when you get a shape id
        ///	from an event or query.
        [[nodiscard]] void* GetUserData() const;
    };

    /// Used to create a shape.
    /// This is a temporary object used to bundle shape creation parameters. You may use
    ///	the same shape definition to create multiple shapes.
    /// Must be initialized using b2DefaultShapeDef().
    /// @ingroup shape
    class Shape : public BasicShapeInterface<Shape, false>
    {
        template <typename, bool>
        friend class BasicShapeInterface;
        template <typename, bool>
        friend class BasicBodyInterface;

      protected:
        b2ShapeId id{};

      public:
        static constexpr bool IsOwning = true;

        // Constructs a null (invalid) object.
        constexpr Shape() noexcept {}

        // The constructor accepts either this or directly `b2ShapeDef`.
        struct Params : b2ShapeDef
        {
            Params() : b2ShapeDef(b2DefaultShapeDef()) {}
        };

        Shape(Shape&& other) noexcept { id = std::exchange(other.id, b2ShapeId{}); }
        Shape& operator=(Shape other) noexcept { std::swap(id, other.id); return *this; }

        ~Shape() { if (*this) Destroy(); }
    };

    template <bool IsConstRef>
    class MaybeConstShapeRef : public BasicShapeInterface<ShapeRef, IsConstRef>
    {
        template <typename, bool>
        friend class BasicShapeInterface;

      protected:
        b2ShapeId id{};

      public:
        static constexpr bool IsOwning = false;
        static constexpr bool IsConst = IsConstRef;

        // Constructs a null (invalid) object.
        constexpr MaybeConstShapeRef() noexcept {}

        // Point to an existing handle.
        // Using a `same_as` template to prevent implicit madness. In particular, to prevent const-to-non-const conversions between non-owning wrappers.
        constexpr MaybeConstShapeRef(std::same_as<b2ShapeId> auto id) noexcept { this->id = id; }

        // Create from a non-reference.
        constexpr MaybeConstShapeRef(const Shape& other) noexcept : MaybeConstShapeRef(other.Handle()) {}

        // Convert a non-const reference to a const reference.
        constexpr MaybeConstShapeRef(const MaybeConstShapeRef<!IsConstRef>& other) noexcept requires IsConstRef : MaybeConstShapeRef(other.Handle()) {}
    };

    template <typename D, bool ForceConst>
    class BasicJointInterface
    {
      protected:
        BasicJointInterface() = default;

      public:
        [[nodiscard]] explicit operator bool() const { return B2_IS_NON_NULL(Handle()); }
        [[nodiscard]] const b2JointId &Handle() const { return static_cast<const D &>(*this).id; }
        // Convert to a handle. Using a `same_as` template to prevent implicit madness. In particular, to prevent const-to-non-const conversions between non-owning wrappers.
        // I want this to return a const reference, but GCC 13 and earlier choke on that (fixed in 14). GCC bug: https://gcc.gnu.org/bugzilla/show_bug.cgi?id=61663
        template <std::same_as<b2JointId> T> [[nodiscard]] operator T() const { return Handle(); }

        /// Destroy a joint
        void Destroy() /*non-const*/ requires (!ForceConst);

        /// Joint identifier validation. Provides validation for up to 64K allocations.
        [[nodiscard]] bool IsValid() const;

        /// Get body A id on a joint
        [[nodiscard]] BodyRef GetBodyA() /*non-const*/ requires (!ForceConst);
        [[nodiscard]] BodyConstRef GetBodyA() const;

        /// Get body B id on a joint
        [[nodiscard]] BodyRef GetBodyB() /*non-const*/ requires (!ForceConst);
        [[nodiscard]] BodyConstRef GetBodyB() const;

        /// Toggle collision between connected bodies
        void SetCollideConnected(bool shouldCollide) /*non-const*/ requires (!ForceConst);

        /// Is collision allowed between connected bodies?
        [[nodiscard]] bool GetCollideConnected() const;

        /// Get the current constraint force for this joint
        [[nodiscard]] b2Vec2 GetConstraintForce() const;

        /// Get the current constraint torque for this joint
        [[nodiscard]] float GetConstraintTorque() const;

        /// Get the local anchor on bodyA
        [[nodiscard]] b2Vec2 GetLocalAnchorA() const;

        /// Get the local anchor on bodyB
        [[nodiscard]] b2Vec2 GetLocalAnchorB() const;

        /// Get the joint type
        [[nodiscard]] b2JointType GetType() const;

        /// Set the user data on a joint
        void SetUserData(void* userData) /*non-const*/ requires (!ForceConst);

        /// Get the user data on a joint
        [[nodiscard]] void* GetUserData() const;

        /// Wake the bodies connect to this joint
        void WakeBodies() /*non-const*/ requires (!ForceConst);
    };

    class Joint : public BasicJointInterface<Joint, false>
    {
        template <typename, bool>
        friend class BasicJointInterface;
        friend class WeldJoint;
        friend class WheelJoint;
        friend class RevoluteJoint;
        friend class DistanceJoint;
        friend class MouseJoint;
        friend class MotorJoint;
        friend class PrismaticJoint;

      protected:
        b2JointId id{};

      public:
        static constexpr bool IsOwning = true;

        // Constructs a null (invalid) object.
        constexpr Joint() noexcept {}

        Joint(Joint&& other) noexcept { id = std::exchange(other.id, b2JointId{}); }
        Joint& operator=(Joint other) noexcept { std::swap(id, other.id); return *this; }

        ~Joint() { if (*this) Destroy(); }
    };

    template <bool IsConstRef>
    class MaybeConstJointRef : public BasicJointInterface<JointRef, IsConstRef>
    {
        template <typename, bool>
        friend class BasicJointInterface;

      protected:
        b2JointId id{};

      public:
        static constexpr bool IsOwning = false;
        static constexpr bool IsConst = IsConstRef;

        // Constructs a null (invalid) object.
        constexpr MaybeConstJointRef() noexcept {}

        // Point to an existing handle.
        // Using a `same_as` template to prevent implicit madness. In particular, to prevent const-to-non-const conversions between non-owning wrappers.
        constexpr MaybeConstJointRef(std::same_as<b2JointId> auto id) noexcept { this->id = id; }

        // Create from a non-reference.
        constexpr MaybeConstJointRef(const Joint& other) noexcept : MaybeConstJointRef(other.Handle()) {}

        // Convert a non-const reference to a const reference.
        constexpr MaybeConstJointRef(const MaybeConstJointRef<!IsConstRef>& other) noexcept requires IsConstRef : MaybeConstJointRef(other.Handle()) {}
    };

    template <typename D, bool ForceConst>
    class BasicDistanceJointInterface
    {
      protected:
        BasicDistanceJointInterface() = default;

      public:
        /// Get the current length of a distance joint
        [[nodiscard]] float GetCurrentLength() const;

        /// Get the spring damping ratio
        [[nodiscard]] float GetDampingRatio() const;

        /// Get the spring Hertz
        [[nodiscard]] float GetHertz() const;

        /// Set the rest length of a distance joint
        /// @param jointId The id for a distance joint
        /// @param length The new distance joint length
        void SetLength(float length) /*non-const*/ requires (!ForceConst);

        /// Get the rest length of a distance joint
        [[nodiscard]] float GetLength() const;

        /// Set the minimum and maximum length parameters of a distance joint
        void SetLengthRange(float minLength, float maxLength) /*non-const*/ requires (!ForceConst);

        /// Enable joint limit. The limit only works if the joint spring is enabled. Otherwise the joint is rigid
        ///	and the limit has no effect.
        void EnableLimit(bool enableLimit) /*non-const*/ requires (!ForceConst);

        /// Is the distance joint limit enabled?
        [[nodiscard]] bool IsLimitEnabled() const;

        /// Get the distance joint maximum length
        [[nodiscard]] float GetMaxLength() const;

        /// Set the distance joint maximum motor force, typically in newtons
        void SetMaxMotorForce(float force) /*non-const*/ requires (!ForceConst);

        /// Get the distance joint maximum motor force, typically in newtons
        [[nodiscard]] float GetMaxMotorForce() const;

        /// Get the distance joint minimum length
        [[nodiscard]] float GetMinLength() const;

        /// Enable/disable the distance joint motor
        void EnableMotor(bool enableMotor) /*non-const*/ requires (!ForceConst);

        /// Is the distance joint motor enabled?
        [[nodiscard]] bool IsMotorEnabled() const;

        /// Get the distance joint current motor force, typically in newtons
        [[nodiscard]] float GetMotorForce() const;

        /// Set the distance joint motor speed, typically in meters per second
        void SetMotorSpeed(float motorSpeed) /*non-const*/ requires (!ForceConst);

        /// Get the distance joint motor speed, typically in meters per second
        [[nodiscard]] float GetMotorSpeed() const;

        /// Enable/disable the distance joint spring. When disabled the distance joint is rigid.
        void EnableSpring(bool enableSpring) /*non-const*/ requires (!ForceConst);

        /// Is the distance joint spring enabled?
        [[nodiscard]] bool IsSpringEnabled() const;

        /// Set the spring damping ratio, non-dimensional
        void SetSpringDampingRatio(float dampingRatio) /*non-const*/ requires (!ForceConst);

        /// Set the spring stiffness in Hertz
        void SetSpringHertz(float hertz) /*non-const*/ requires (!ForceConst);
    };

    /// Distance joint definition
    ///
    /// This requires defining an anchor point on both
    /// bodies and the non-zero distance of the distance joint. The definition uses
    /// local anchor points so that the initial configuration can violate the
    /// constraint slightly. This helps when saving and loading a game.
    /// @ingroup distance_joint
    class DistanceJoint : public Joint, public BasicDistanceJointInterface<DistanceJoint, false>
    {
        template <typename, bool>
        friend class BasicDistanceJointInterface;
        template <typename, bool>
        friend class BasicWorldInterface;

      public:
        static constexpr bool IsOwning = true;

        // Constructs a null (invalid) object.
        constexpr DistanceJoint() noexcept {}

        // The constructor accepts either this or directly `b2DistanceJointDef`.
        struct Params : b2DistanceJointDef
        {
            Params() : b2DistanceJointDef(b2DefaultDistanceJointDef()) {}
        };

        // Downcast from a generic joint.
        // Triggers an assertion if this isn't the right joint kind.
        explicit DistanceJoint(Joint&& other) noexcept
        {
            if (other.GetType() == b2_distanceJoint)
                this->id = std::exchange(other.id, {});
            else
                BOX2CPP_ASSERT(false && "This joint is not a `DistanceJoint`.");
        }
    };

    template <bool IsConstRef>
    class MaybeConstDistanceJointRef : public MaybeConstJointRef<IsConstRef>, public BasicDistanceJointInterface<DistanceJointRef, IsConstRef>
    {
        template <typename, bool>
        friend class BasicDistanceJointInterface;

      public:
        static constexpr bool IsOwning = false;
        static constexpr bool IsConst = IsConstRef;

        // Constructs a null (invalid) object.
        constexpr MaybeConstDistanceJointRef() noexcept {}

        // Point to an existing handle.
        // Using a `same_as` template to prevent implicit madness. In particular, to prevent const-to-non-const conversions between non-owning wrappers.
        // Downcast from a generic joint reference (or owning joint).
        // Triggers an assertion if this isn't the right joint kind.
        explicit constexpr MaybeConstDistanceJointRef(std::same_as<b2JointId> auto id) noexcept
        {
            if (b2Joint_GetType(id) == b2_distanceJoint)
                this->id = id;
            else
                BOX2CPP_ASSERT(false && "This joint is not a `DistanceJoint`.");
        }

        // Create from a non-reference.
        constexpr MaybeConstDistanceJointRef(const DistanceJoint& other) noexcept : MaybeConstDistanceJointRef(other.Handle()) {}

        // Triggers an assertion if this isn't the right joint kind.
        explicit constexpr MaybeConstDistanceJointRef(MaybeConstJointRef<IsConstRef> other) noexcept : MaybeConstDistanceJointRef(other.Handle()) {}
        // Convert a non-const reference to a const reference.
        constexpr MaybeConstDistanceJointRef(const MaybeConstDistanceJointRef<!IsConstRef>& other) noexcept requires IsConstRef : MaybeConstDistanceJointRef(other.Handle()) {}
    };

    template <typename D, bool ForceConst>
    class BasicMotorJointInterface
    {
      protected:
        BasicMotorJointInterface() = default;

      public:
        /// Set the motor joint angular offset target in radians
        void SetAngularOffset(float angularOffset) /*non-const*/ requires (!ForceConst);

        /// Get the motor joint angular offset target in radians
        [[nodiscard]] float GetAngularOffset() const;

        /// Set the motor joint correction factor, typically in [0, 1]
        void SetCorrectionFactor(float correctionFactor) /*non-const*/ requires (!ForceConst);

        /// Get the motor joint correction factor, typically in [0, 1]
        [[nodiscard]] float GetCorrectionFactor() const;

        /// Set the motor joint linear offset target
        void SetLinearOffset(b2Vec2 linearOffset) /*non-const*/ requires (!ForceConst);

        /// Get the motor joint linear offset target
        [[nodiscard]] b2Vec2 GetLinearOffset() const;

        /// Set the motor joint maximum force, typically in newtons
        void SetMaxForce(float maxForce) /*non-const*/ requires (!ForceConst);

        /// Get the motor joint maximum force, typically in newtons
        [[nodiscard]] float GetMaxForce() const;

        /// Set the motor joint maximum torque, typically in newton-meters
        void SetMaxTorque(float maxTorque) /*non-const*/ requires (!ForceConst);

        /// Get the motor joint maximum torque, typically in newton-meters
        [[nodiscard]] float GetMaxTorque() const;
    };

    /// A motor joint is used to control the relative motion between two bodies
    ///
    /// A typical usage is to control the movement of a dynamic body with respect to the ground.
    /// @ingroup motor_joint
    class MotorJoint : public Joint, public BasicMotorJointInterface<MotorJoint, false>
    {
        template <typename, bool>
        friend class BasicMotorJointInterface;
        template <typename, bool>
        friend class BasicWorldInterface;

      public:
        static constexpr bool IsOwning = true;

        // Constructs a null (invalid) object.
        constexpr MotorJoint() noexcept {}

        // The constructor accepts either this or directly `b2MotorJointDef`.
        struct Params : b2MotorJointDef
        {
            Params() : b2MotorJointDef(b2DefaultMotorJointDef()) {}
        };

        // Downcast from a generic joint.
        // Triggers an assertion if this isn't the right joint kind.
        explicit MotorJoint(Joint&& other) noexcept
        {
            if (other.GetType() == b2_motorJoint)
                this->id = std::exchange(other.id, {});
            else
                BOX2CPP_ASSERT(false && "This joint is not a `MotorJoint`.");
        }
    };

    template <bool IsConstRef>
    class MaybeConstMotorJointRef : public MaybeConstJointRef<IsConstRef>, public BasicMotorJointInterface<MotorJointRef, IsConstRef>
    {
        template <typename, bool>
        friend class BasicMotorJointInterface;

      public:
        static constexpr bool IsOwning = false;
        static constexpr bool IsConst = IsConstRef;

        // Constructs a null (invalid) object.
        constexpr MaybeConstMotorJointRef() noexcept {}

        // Point to an existing handle.
        // Using a `same_as` template to prevent implicit madness. In particular, to prevent const-to-non-const conversions between non-owning wrappers.
        // Downcast from a generic joint reference (or owning joint).
        // Triggers an assertion if this isn't the right joint kind.
        explicit constexpr MaybeConstMotorJointRef(std::same_as<b2JointId> auto id) noexcept
        {
            if (b2Joint_GetType(id) == b2_motorJoint)
                this->id = id;
            else
                BOX2CPP_ASSERT(false && "This joint is not a `MotorJoint`.");
        }

        // Create from a non-reference.
        constexpr MaybeConstMotorJointRef(const MotorJoint& other) noexcept : MaybeConstMotorJointRef(other.Handle()) {}

        // Triggers an assertion if this isn't the right joint kind.
        explicit constexpr MaybeConstMotorJointRef(MaybeConstJointRef<IsConstRef> other) noexcept : MaybeConstMotorJointRef(other.Handle()) {}
        // Convert a non-const reference to a const reference.
        constexpr MaybeConstMotorJointRef(const MaybeConstMotorJointRef<!IsConstRef>& other) noexcept requires IsConstRef : MaybeConstMotorJointRef(other.Handle()) {}
    };

    template <typename D, bool ForceConst>
    class BasicMouseJointInterface
    {
      protected:
        BasicMouseJointInterface() = default;

      public:
        /// Set the mouse joint maximum force, typically in newtons
        void SetMaxForce(float maxForce) /*non-const*/ requires (!ForceConst);

        /// Get the mouse joint maximum force, typically in newtons
        [[nodiscard]] float GetMaxForce() const;

        /// Set the mouse joint spring damping ratio, non-dimensional
        void SetSpringDampingRatio(float dampingRatio) /*non-const*/ requires (!ForceConst);

        /// Get the mouse joint damping ratio, non-dimensional
        [[nodiscard]] float GetSpringDampingRatio() const;

        /// Set the mouse joint spring stiffness in Hertz
        void SetSpringHertz(float hertz) /*non-const*/ requires (!ForceConst);

        /// Get the mouse joint spring stiffness in Hertz
        [[nodiscard]] float GetSpringHertz() const;

        /// Set the mouse joint target
        void SetTarget(b2Vec2 target) /*non-const*/ requires (!ForceConst);

        /// Get the mouse joint target
        [[nodiscard]] b2Vec2 GetTarget() const;
    };

    /// A mouse joint is used to make a point on a body track a specified world point.
    ///
    /// This a soft constraint and allows the constraint to stretch without
    /// applying huge forces. This also applies rotation constraint heuristic to improve control.
    /// @ingroup mouse_joint
    class MouseJoint : public Joint, public BasicMouseJointInterface<MouseJoint, false>
    {
        template <typename, bool>
        friend class BasicMouseJointInterface;
        template <typename, bool>
        friend class BasicWorldInterface;

      public:
        static constexpr bool IsOwning = true;

        // Constructs a null (invalid) object.
        constexpr MouseJoint() noexcept {}

        // The constructor accepts either this or directly `b2MouseJointDef`.
        struct Params : b2MouseJointDef
        {
            Params() : b2MouseJointDef(b2DefaultMouseJointDef()) {}
        };

        // Downcast from a generic joint.
        // Triggers an assertion if this isn't the right joint kind.
        explicit MouseJoint(Joint&& other) noexcept
        {
            if (other.GetType() == b2_mouseJoint)
                this->id = std::exchange(other.id, {});
            else
                BOX2CPP_ASSERT(false && "This joint is not a `MouseJoint`.");
        }
    };

    template <bool IsConstRef>
    class MaybeConstMouseJointRef : public MaybeConstJointRef<IsConstRef>, public BasicMouseJointInterface<MouseJointRef, IsConstRef>
    {
        template <typename, bool>
        friend class BasicMouseJointInterface;

      public:
        static constexpr bool IsOwning = false;
        static constexpr bool IsConst = IsConstRef;

        // Constructs a null (invalid) object.
        constexpr MaybeConstMouseJointRef() noexcept {}

        // Point to an existing handle.
        // Using a `same_as` template to prevent implicit madness. In particular, to prevent const-to-non-const conversions between non-owning wrappers.
        // Downcast from a generic joint reference (or owning joint).
        // Triggers an assertion if this isn't the right joint kind.
        explicit constexpr MaybeConstMouseJointRef(std::same_as<b2JointId> auto id) noexcept
        {
            if (b2Joint_GetType(id) == b2_mouseJoint)
                this->id = id;
            else
                BOX2CPP_ASSERT(false && "This joint is not a `MouseJoint`.");
        }

        // Create from a non-reference.
        constexpr MaybeConstMouseJointRef(const MouseJoint& other) noexcept : MaybeConstMouseJointRef(other.Handle()) {}

        // Triggers an assertion if this isn't the right joint kind.
        explicit constexpr MaybeConstMouseJointRef(MaybeConstJointRef<IsConstRef> other) noexcept : MaybeConstMouseJointRef(other.Handle()) {}
        // Convert a non-const reference to a const reference.
        constexpr MaybeConstMouseJointRef(const MaybeConstMouseJointRef<!IsConstRef>& other) noexcept requires IsConstRef : MaybeConstMouseJointRef(other.Handle()) {}
    };

    template <typename D, bool ForceConst>
    class BasicPrismaticJointInterface
    {
      protected:
        BasicPrismaticJointInterface() = default;

      public:
        /// Enable/disable a prismatic joint limit
        void EnableLimit(bool enableLimit) /*non-const*/ requires (!ForceConst);

        /// Is the prismatic joint limit enabled?
        [[nodiscard]] bool IsLimitEnabled() const;

        /// Set the prismatic joint limits
        void SetLimits(float lower, float upper) /*non-const*/ requires (!ForceConst);

        /// Get the prismatic joint lower limit
        [[nodiscard]] float GetLowerLimit() const;

        /// Set the prismatic joint maximum motor force, typically in newtons
        void SetMaxMotorForce(float force) /*non-const*/ requires (!ForceConst);

        /// Get the prismatic joint maximum motor force, typically in newtons
        [[nodiscard]] float GetMaxMotorForce() const;

        /// Enable/disable a prismatic joint motor
        void EnableMotor(bool enableMotor) /*non-const*/ requires (!ForceConst);

        /// Is the prismatic joint motor enabled?
        [[nodiscard]] bool IsMotorEnabled() const;

        /// Get the prismatic joint current motor force, typically in newtons
        [[nodiscard]] float GetMotorForce() const;

        /// Set the prismatic joint motor speed, typically in meters per second
        void SetMotorSpeed(float motorSpeed) /*non-const*/ requires (!ForceConst);

        /// Get the prismatic joint motor speed, typically in meters per second
        [[nodiscard]] float GetMotorSpeed() const;

        /// Enable/disable the joint spring.
        void EnableSpring(bool enableSpring) /*non-const*/ requires (!ForceConst);

        /// Is the prismatic joint spring enabled or not?
        [[nodiscard]] bool IsSpringEnabled() const;

        /// Set the prismatic joint damping ratio (non-dimensional)
        void SetSpringDampingRatio(float dampingRatio) /*non-const*/ requires (!ForceConst);

        /// Get the prismatic spring damping ratio (non-dimensional)
        [[nodiscard]] float GetSpringDampingRatio() const;

        /// Set the prismatic joint stiffness in Hertz.
        /// This should usually be less than a quarter of the simulation rate. For example, if the simulation
        /// runs at 60Hz then the joint stiffness should be 15Hz or less.
        void SetSpringHertz(float hertz) /*non-const*/ requires (!ForceConst);

        /// Get the prismatic joint stiffness in Hertz
        [[nodiscard]] float GetSpringHertz() const;

        /// Get the prismatic joint upper limit
        [[nodiscard]] float GetUpperLimit() const;
    };

    /// Prismatic joint definition
    ///
    /// This requires defining a line of motion using an axis and an anchor point.
    /// The definition uses local anchor points and a local axis so that the initial
    /// configuration can violate the constraint slightly. The joint translation is zero
    /// when the local anchor points coincide in world space.
    /// @ingroup prismatic_joint
    class PrismaticJoint : public Joint, public BasicPrismaticJointInterface<PrismaticJoint, false>
    {
        template <typename, bool>
        friend class BasicPrismaticJointInterface;
        template <typename, bool>
        friend class BasicWorldInterface;

      public:
        static constexpr bool IsOwning = true;

        // Constructs a null (invalid) object.
        constexpr PrismaticJoint() noexcept {}

        // The constructor accepts either this or directly `b2PrismaticJointDef`.
        struct Params : b2PrismaticJointDef
        {
            Params() : b2PrismaticJointDef(b2DefaultPrismaticJointDef()) {}
        };

        // Downcast from a generic joint.
        // Triggers an assertion if this isn't the right joint kind.
        explicit PrismaticJoint(Joint&& other) noexcept
        {
            if (other.GetType() == b2_prismaticJoint)
                this->id = std::exchange(other.id, {});
            else
                BOX2CPP_ASSERT(false && "This joint is not a `PrismaticJoint`.");
        }
    };

    template <bool IsConstRef>
    class MaybeConstPrismaticJointRef : public MaybeConstJointRef<IsConstRef>, public BasicPrismaticJointInterface<PrismaticJointRef, IsConstRef>
    {
        template <typename, bool>
        friend class BasicPrismaticJointInterface;

      public:
        static constexpr bool IsOwning = false;
        static constexpr bool IsConst = IsConstRef;

        // Constructs a null (invalid) object.
        constexpr MaybeConstPrismaticJointRef() noexcept {}

        // Point to an existing handle.
        // Using a `same_as` template to prevent implicit madness. In particular, to prevent const-to-non-const conversions between non-owning wrappers.
        // Downcast from a generic joint reference (or owning joint).
        // Triggers an assertion if this isn't the right joint kind.
        explicit constexpr MaybeConstPrismaticJointRef(std::same_as<b2JointId> auto id) noexcept
        {
            if (b2Joint_GetType(id) == b2_prismaticJoint)
                this->id = id;
            else
                BOX2CPP_ASSERT(false && "This joint is not a `PrismaticJoint`.");
        }

        // Create from a non-reference.
        constexpr MaybeConstPrismaticJointRef(const PrismaticJoint& other) noexcept : MaybeConstPrismaticJointRef(other.Handle()) {}

        // Triggers an assertion if this isn't the right joint kind.
        explicit constexpr MaybeConstPrismaticJointRef(MaybeConstJointRef<IsConstRef> other) noexcept : MaybeConstPrismaticJointRef(other.Handle()) {}
        // Convert a non-const reference to a const reference.
        constexpr MaybeConstPrismaticJointRef(const MaybeConstPrismaticJointRef<!IsConstRef>& other) noexcept requires IsConstRef : MaybeConstPrismaticJointRef(other.Handle()) {}
    };

    template <typename D, bool ForceConst>
    class BasicRevoluteJointInterface
    {
      protected:
        BasicRevoluteJointInterface() = default;

      public:
        /// Get the revolute joint current angle in radians relative to the reference angle
        ///	@see b2RevoluteJointDef::referenceAngle
        [[nodiscard]] float GetAngle() const;

        /// Enable/disable the revolute joint limit
        void EnableLimit(bool enableLimit) /*non-const*/ requires (!ForceConst);

        /// Is the revolute joint limit enabled?
        [[nodiscard]] bool IsLimitEnabled() const;

        /// Set the revolute joint limits in radians
        void SetLimits(float lower, float upper) /*non-const*/ requires (!ForceConst);

        /// Get the revolute joint lower limit in radians
        [[nodiscard]] float GetLowerLimit() const;

        /// Set the revolute joint maximum motor torque, typically in newton-meters
        void SetMaxMotorTorque(float torque) /*non-const*/ requires (!ForceConst);

        /// Get the revolute joint maximum motor torque, typically in newton-meters
        [[nodiscard]] float GetMaxMotorTorque() const;

        /// Enable/disable a revolute joint motor
        void EnableMotor(bool enableMotor) /*non-const*/ requires (!ForceConst);

        /// Is the revolute joint motor enabled?
        [[nodiscard]] bool IsMotorEnabled() const;

        /// Set the revolute joint motor speed in radians per second
        void SetMotorSpeed(float motorSpeed) /*non-const*/ requires (!ForceConst);

        /// Get the revolute joint motor speed in radians per second
        [[nodiscard]] float GetMotorSpeed() const;

        /// Get the revolute joint current motor torque, typically in newton-meters
        [[nodiscard]] float GetMotorTorque() const;

        /// Enable/disable the revolute joint spring
        void EnableSpring(bool enableSpring) /*non-const*/ requires (!ForceConst);

        /// Set the revolute joint spring damping ratio, non-dimensional
        void SetSpringDampingRatio(float dampingRatio) /*non-const*/ requires (!ForceConst);

        /// Get the revolute joint spring damping ratio, non-dimensional
        [[nodiscard]] float GetSpringDampingRatio() const;

        /// Set the revolute joint spring stiffness in Hertz
        void SetSpringHertz(float hertz) /*non-const*/ requires (!ForceConst);

        /// Get the revolute joint spring stiffness in Hertz
        [[nodiscard]] float GetSpringHertz() const;

        /// Get the revolute joint upper limit in radians
        [[nodiscard]] float GetUpperLimit() const;
    };

    /// Revolute joint definition
    ///
    /// This requires defining an anchor point where the bodies are joined.
    /// The definition uses local anchor points so that the
    /// initial configuration can violate the constraint slightly. You also need to
    /// specify the initial relative angle for joint limits. This helps when saving
    /// and loading a game.
    /// The local anchor points are measured from the body's origin
    /// rather than the center of mass because:
    /// 1. you might not know where the center of mass will be
    /// 2. if you add/remove shapes from a body and recompute the mass, the joints will be broken
    /// @ingroup revolute_joint
    class RevoluteJoint : public Joint, public BasicRevoluteJointInterface<RevoluteJoint, false>
    {
        template <typename, bool>
        friend class BasicRevoluteJointInterface;
        template <typename, bool>
        friend class BasicWorldInterface;

      public:
        static constexpr bool IsOwning = true;

        // Constructs a null (invalid) object.
        constexpr RevoluteJoint() noexcept {}

        // The constructor accepts either this or directly `b2RevoluteJointDef`.
        struct Params : b2RevoluteJointDef
        {
            Params() : b2RevoluteJointDef(b2DefaultRevoluteJointDef()) {}
        };

        // Downcast from a generic joint.
        // Triggers an assertion if this isn't the right joint kind.
        explicit RevoluteJoint(Joint&& other) noexcept
        {
            if (other.GetType() == b2_revoluteJoint)
                this->id = std::exchange(other.id, {});
            else
                BOX2CPP_ASSERT(false && "This joint is not a `RevoluteJoint`.");
        }
    };

    template <bool IsConstRef>
    class MaybeConstRevoluteJointRef : public MaybeConstJointRef<IsConstRef>, public BasicRevoluteJointInterface<RevoluteJointRef, IsConstRef>
    {
        template <typename, bool>
        friend class BasicRevoluteJointInterface;

      public:
        static constexpr bool IsOwning = false;
        static constexpr bool IsConst = IsConstRef;

        // Constructs a null (invalid) object.
        constexpr MaybeConstRevoluteJointRef() noexcept {}

        // Point to an existing handle.
        // Using a `same_as` template to prevent implicit madness. In particular, to prevent const-to-non-const conversions between non-owning wrappers.
        // Downcast from a generic joint reference (or owning joint).
        // Triggers an assertion if this isn't the right joint kind.
        explicit constexpr MaybeConstRevoluteJointRef(std::same_as<b2JointId> auto id) noexcept
        {
            if (b2Joint_GetType(id) == b2_revoluteJoint)
                this->id = id;
            else
                BOX2CPP_ASSERT(false && "This joint is not a `RevoluteJoint`.");
        }

        // Create from a non-reference.
        constexpr MaybeConstRevoluteJointRef(const RevoluteJoint& other) noexcept : MaybeConstRevoluteJointRef(other.Handle()) {}

        // Triggers an assertion if this isn't the right joint kind.
        explicit constexpr MaybeConstRevoluteJointRef(MaybeConstJointRef<IsConstRef> other) noexcept : MaybeConstRevoluteJointRef(other.Handle()) {}
        // Convert a non-const reference to a const reference.
        constexpr MaybeConstRevoluteJointRef(const MaybeConstRevoluteJointRef<!IsConstRef>& other) noexcept requires IsConstRef : MaybeConstRevoluteJointRef(other.Handle()) {}
    };

    template <typename D, bool ForceConst>
    class BasicWeldJointInterface
    {
      protected:
        BasicWeldJointInterface() = default;

      public:
        /// Set weld joint angular damping ratio, non-dimensional
        void SetAngularDampingRatio(float dampingRatio) /*non-const*/ requires (!ForceConst);

        /// Get the weld joint angular damping ratio, non-dimensional
        [[nodiscard]] float GetAngularDampingRatio() const;

        /// Set the weld joint angular stiffness in Hertz. 0 is rigid.
        void SetAngularHertz(float hertz) /*non-const*/ requires (!ForceConst);

        /// Get the weld joint angular stiffness in Hertz
        [[nodiscard]] float GetAngularHertz() const;

        /// Set the weld joint linear damping ratio (non-dimensional)
        void SetLinearDampingRatio(float dampingRatio) /*non-const*/ requires (!ForceConst);

        /// Get the weld joint linear damping ratio (non-dimensional)
        [[nodiscard]] float GetLinearDampingRatio() const;

        /// Set the weld joint linear stiffness in Hertz. 0 is rigid.
        void SetLinearHertz(float hertz) /*non-const*/ requires (!ForceConst);

        /// Get the weld joint linear stiffness in Hertz
        [[nodiscard]] float GetLinearHertz() const;
    };

    /// Weld joint definition
    ///
    /// A weld joint connect to bodies together rigidly. This constraint provides springs to mimic
    ///	soft-body simulation.
    /// @note The approximate solver in Box2D cannot hold many bodies together rigidly
    /// @ingroup weld_joint
    class WeldJoint : public Joint, public BasicWeldJointInterface<WeldJoint, false>
    {
        template <typename, bool>
        friend class BasicWeldJointInterface;
        template <typename, bool>
        friend class BasicWorldInterface;

      public:
        static constexpr bool IsOwning = true;

        // Constructs a null (invalid) object.
        constexpr WeldJoint() noexcept {}

        // The constructor accepts either this or directly `b2WeldJointDef`.
        struct Params : b2WeldJointDef
        {
            Params() : b2WeldJointDef(b2DefaultWeldJointDef()) {}
        };

        // Downcast from a generic joint.
        // Triggers an assertion if this isn't the right joint kind.
        explicit WeldJoint(Joint&& other) noexcept
        {
            if (other.GetType() == b2_weldJoint)
                this->id = std::exchange(other.id, {});
            else
                BOX2CPP_ASSERT(false && "This joint is not a `WeldJoint`.");
        }
    };

    template <bool IsConstRef>
    class MaybeConstWeldJointRef : public MaybeConstJointRef<IsConstRef>, public BasicWeldJointInterface<WeldJointRef, IsConstRef>
    {
        template <typename, bool>
        friend class BasicWeldJointInterface;

      public:
        static constexpr bool IsOwning = false;
        static constexpr bool IsConst = IsConstRef;

        // Constructs a null (invalid) object.
        constexpr MaybeConstWeldJointRef() noexcept {}

        // Point to an existing handle.
        // Using a `same_as` template to prevent implicit madness. In particular, to prevent const-to-non-const conversions between non-owning wrappers.
        // Downcast from a generic joint reference (or owning joint).
        // Triggers an assertion if this isn't the right joint kind.
        explicit constexpr MaybeConstWeldJointRef(std::same_as<b2JointId> auto id) noexcept
        {
            if (b2Joint_GetType(id) == b2_weldJoint)
                this->id = id;
            else
                BOX2CPP_ASSERT(false && "This joint is not a `WeldJoint`.");
        }

        // Create from a non-reference.
        constexpr MaybeConstWeldJointRef(const WeldJoint& other) noexcept : MaybeConstWeldJointRef(other.Handle()) {}

        // Triggers an assertion if this isn't the right joint kind.
        explicit constexpr MaybeConstWeldJointRef(MaybeConstJointRef<IsConstRef> other) noexcept : MaybeConstWeldJointRef(other.Handle()) {}
        // Convert a non-const reference to a const reference.
        constexpr MaybeConstWeldJointRef(const MaybeConstWeldJointRef<!IsConstRef>& other) noexcept requires IsConstRef : MaybeConstWeldJointRef(other.Handle()) {}
    };

    template <typename D, bool ForceConst>
    class BasicWheelJointInterface
    {
      protected:
        BasicWheelJointInterface() = default;

      public:
        /// Enable/disable the wheel joint limit
        void EnableLimit(bool enableLimit) /*non-const*/ requires (!ForceConst);

        /// Is the wheel joint limit enabled?
        [[nodiscard]] bool IsLimitEnabled() const;

        /// Set the wheel joint limits
        void SetLimits(float lower, float upper) /*non-const*/ requires (!ForceConst);

        /// Get the wheel joint lower limit
        [[nodiscard]] float GetLowerLimit() const;

        /// Set the wheel joint maximum motor torque, typically in newton-meters
        void SetMaxMotorTorque(float torque) /*non-const*/ requires (!ForceConst);

        /// Get the wheel joint maximum motor torque, typically in newton-meters
        [[nodiscard]] float GetMaxMotorTorque() const;

        /// Enable/disable the wheel joint motor
        void EnableMotor(bool enableMotor) /*non-const*/ requires (!ForceConst);

        /// Is the wheel joint motor enabled?
        [[nodiscard]] bool IsMotorEnabled() const;

        /// Set the wheel joint motor speed in radians per second
        void SetMotorSpeed(float motorSpeed) /*non-const*/ requires (!ForceConst);

        /// Get the wheel joint motor speed in radians per second
        [[nodiscard]] float GetMotorSpeed() const;

        /// Get the wheel joint current motor torque, typically in newton-meters
        [[nodiscard]] float GetMotorTorque() const;

        /// Enable/disable the wheel joint spring
        void EnableSpring(bool enableSpring) /*non-const*/ requires (!ForceConst);

        /// Is the wheel joint spring enabled?
        [[nodiscard]] bool IsSpringEnabled() const;

        /// Set the wheel joint damping ratio, non-dimensional
        void SetSpringDampingRatio(float dampingRatio) /*non-const*/ requires (!ForceConst);

        /// Get the wheel joint damping ratio, non-dimensional
        [[nodiscard]] float GetSpringDampingRatio() const;

        /// Set the wheel joint stiffness in Hertz
        void SetSpringHertz(float hertz) /*non-const*/ requires (!ForceConst);

        /// Get the wheel joint stiffness in Hertz
        [[nodiscard]] float GetSpringHertz() const;

        /// Get the wheel joint upper limit
        [[nodiscard]] float GetUpperLimit() const;
    };

    /// Wheel joint definition
    ///
    /// This requires defining a line of motion using an axis and an anchor point.
    /// The definition uses local  anchor points and a local axis so that the initial
    /// configuration can violate the constraint slightly. The joint translation is zero
    /// when the local anchor points coincide in world space.
    /// @ingroup wheel_joint
    class WheelJoint : public Joint, public BasicWheelJointInterface<WheelJoint, false>
    {
        template <typename, bool>
        friend class BasicWheelJointInterface;
        template <typename, bool>
        friend class BasicWorldInterface;

      public:
        static constexpr bool IsOwning = true;

        // Constructs a null (invalid) object.
        constexpr WheelJoint() noexcept {}

        // The constructor accepts either this or directly `b2WheelJointDef`.
        struct Params : b2WheelJointDef
        {
            Params() : b2WheelJointDef(b2DefaultWheelJointDef()) {}
        };

        // Downcast from a generic joint.
        // Triggers an assertion if this isn't the right joint kind.
        explicit WheelJoint(Joint&& other) noexcept
        {
            if (other.GetType() == b2_wheelJoint)
                this->id = std::exchange(other.id, {});
            else
                BOX2CPP_ASSERT(false && "This joint is not a `WheelJoint`.");
        }
    };

    template <bool IsConstRef>
    class MaybeConstWheelJointRef : public MaybeConstJointRef<IsConstRef>, public BasicWheelJointInterface<WheelJointRef, IsConstRef>
    {
        template <typename, bool>
        friend class BasicWheelJointInterface;

      public:
        static constexpr bool IsOwning = false;
        static constexpr bool IsConst = IsConstRef;

        // Constructs a null (invalid) object.
        constexpr MaybeConstWheelJointRef() noexcept {}

        // Point to an existing handle.
        // Using a `same_as` template to prevent implicit madness. In particular, to prevent const-to-non-const conversions between non-owning wrappers.
        // Downcast from a generic joint reference (or owning joint).
        // Triggers an assertion if this isn't the right joint kind.
        explicit constexpr MaybeConstWheelJointRef(std::same_as<b2JointId> auto id) noexcept
        {
            if (b2Joint_GetType(id) == b2_wheelJoint)
                this->id = id;
            else
                BOX2CPP_ASSERT(false && "This joint is not a `WheelJoint`.");
        }

        // Create from a non-reference.
        constexpr MaybeConstWheelJointRef(const WheelJoint& other) noexcept : MaybeConstWheelJointRef(other.Handle()) {}

        // Triggers an assertion if this isn't the right joint kind.
        explicit constexpr MaybeConstWheelJointRef(MaybeConstJointRef<IsConstRef> other) noexcept : MaybeConstWheelJointRef(other.Handle()) {}
        // Convert a non-const reference to a const reference.
        constexpr MaybeConstWheelJointRef(const MaybeConstWheelJointRef<!IsConstRef>& other) noexcept requires IsConstRef : MaybeConstWheelJointRef(other.Handle()) {}
    };

    template <typename D, bool ForceConst>
    class BasicBodyInterface
    {
      protected:
        BasicBodyInterface() = default;

      public:
        [[nodiscard]] explicit operator bool() const { return B2_IS_NON_NULL(Handle()); }
        [[nodiscard]] const b2BodyId &Handle() const { return static_cast<const D &>(*this).id; }
        // Convert to a handle. Using a `same_as` template to prevent implicit madness. In particular, to prevent const-to-non-const conversions between non-owning wrappers.
        // I want this to return a const reference, but GCC 13 and earlier choke on that (fixed in 14). GCC bug: https://gcc.gnu.org/bugzilla/show_bug.cgi?id=61663
        template <std::same_as<b2BodyId> T> [[nodiscard]] operator T() const { return Handle(); }

        /// Create a chain shape
        ///	@see b2ChainDef for details
        [[nodiscard]] Chain CreateChain(Tags::OwningHandle, const std::derived_from<b2ChainDef> auto& def) /*non-const*/ requires (!ForceConst);
        ChainRef CreateChain(Tags::DestroyWithParent, const std::derived_from<b2ChainDef> auto& def) /*non-const*/ requires (!ForceConst);

        /// Create a capsule shape and attach it to a body. The shape definition and geometry are fully cloned.
        /// Contacts are not created until the next time step.
        ///	@return the shape id for accessing the shape
        [[nodiscard]] Shape CreateShape(Tags::OwningHandle, const std::derived_from<b2ShapeDef> auto& def, const b2Capsule& capsule) /*non-const*/ requires (!ForceConst);
        ShapeRef CreateShape(Tags::DestroyWithParent, const std::derived_from<b2ShapeDef> auto& def, const b2Capsule& capsule) /*non-const*/ requires (!ForceConst);

        /// Create a circle shape and attach it to a body. The shape definition and geometry are fully cloned.
        /// Contacts are not created until the next time step.
        ///	@return the shape id for accessing the shape
        [[nodiscard]] Shape CreateShape(Tags::OwningHandle, const std::derived_from<b2ShapeDef> auto& def, const b2Circle& circle) /*non-const*/ requires (!ForceConst);
        ShapeRef CreateShape(Tags::DestroyWithParent, const std::derived_from<b2ShapeDef> auto& def, const b2Circle& circle) /*non-const*/ requires (!ForceConst);

        /// Create a polygon shape and attach it to a body. The shape definition and geometry are fully cloned.
        /// Contacts are not created until the next time step.
        ///	@return the shape id for accessing the shape
        [[nodiscard]] Shape CreateShape(Tags::OwningHandle, const std::derived_from<b2ShapeDef> auto& def, const b2Polygon& polygon) /*non-const*/ requires (!ForceConst);
        ShapeRef CreateShape(Tags::DestroyWithParent, const std::derived_from<b2ShapeDef> auto& def, const b2Polygon& polygon) /*non-const*/ requires (!ForceConst);

        /// Create a line segment shape and attach it to a body. The shape definition and geometry are fully cloned.
        /// Contacts are not created until the next time step.
        ///	@return the shape id for accessing the shape
        [[nodiscard]] Shape CreateShape(Tags::OwningHandle, const std::derived_from<b2ShapeDef> auto& def, const b2Segment& segment) /*non-const*/ requires (!ForceConst);
        ShapeRef CreateShape(Tags::DestroyWithParent, const std::derived_from<b2ShapeDef> auto& def, const b2Segment& segment) /*non-const*/ requires (!ForceConst);

        /// Destroy a rigid body given an id. This destroys all shapes and joints attached to the body.
        ///	Do not keep references to the associated shapes and joints.
        void Destroy() /*non-const*/ requires (!ForceConst);

        /// Body identifier validation. Can be used to detect orphaned ids. Provides validation for up to 64K allocations.
        [[nodiscard]] bool IsValid() const;

        /// Enable a body by adding it to the simulation. This is expensive.
        void Enable() /*non-const*/ requires (!ForceConst);

        /// Returns true if this body is enabled
        [[nodiscard]] bool IsEnabled() const;

        /// Adjust the angular damping. Normally this is set in b2BodyDef before creation.
        void SetAngularDamping(float angularDamping) /*non-const*/ requires (!ForceConst);

        /// Get the current angular damping.
        [[nodiscard]] float GetAngularDamping() const;

        /// Set the angular velocity of a body in radians per second
        void SetAngularVelocity(float angularVelocity) /*non-const*/ requires (!ForceConst);

        /// Get the angular velocity of a body in radians per second
        [[nodiscard]] float GetAngularVelocity() const;

        /// Apply an angular impulse. The impulse is ignored if the body is not awake.
        /// This optionally wakes the body.
        ///	@param bodyId The body id
        /// @param impulse the angular impulse, typically in units of kg*m*m/s
        /// @param wake also wake up the body
        ///	@warning This should be used for one-shot impulses. If you need a steady force,
        /// use a force instead, which will work better with the sub-stepping solver.
        void ApplyAngularImpulse(float impulse, bool wake) /*non-const*/ requires (!ForceConst);

        /// Apply a force at a world point. If the force is not applied at the center of mass,
        /// it will generate a torque and affect the angular velocity. This optionally wakes up the body.
        ///	The force is ignored if the body is not awake.
        ///	@param bodyId The body id
        /// @param force The world force vector, typically in newtons (N)
        /// @param point The world position of the point of application
        /// @param wake Option to wake up the body
        void ApplyForce(b2Vec2 force, b2Vec2 point, bool wake) /*non-const*/ requires (!ForceConst);

        /// Apply a force to the center of mass. This optionally wakes up the body.
        ///	The force is ignored if the body is not awake.
        ///	@param bodyId The body id
        /// @param force the world force vector, usually in newtons (N).
        /// @param wake also wake up the body
        void ApplyForceToCenter(b2Vec2 force, bool wake) /*non-const*/ requires (!ForceConst);

        /// Apply an impulse at a point. This immediately modifies the velocity.
        /// It also modifies the angular velocity if the point of application
        /// is not at the center of mass. This optionally wakes the body.
        /// The impulse is ignored if the body is not awake.
        ///	@param bodyId The body id
        /// @param impulse the world impulse vector, typically in N*s or kg*m/s.
        /// @param point the world position of the point of application.
        /// @param wake also wake up the body
        ///	@warning This should be used for one-shot impulses. If you need a steady force,
        /// use a force instead, which will work better with the sub-stepping solver.
        void ApplyLinearImpulse(b2Vec2 impulse, b2Vec2 point, bool wake) /*non-const*/ requires (!ForceConst);

        /// Apply an impulse to the center of mass. This immediately modifies the velocity.
        /// The impulse is ignored if the body is not awake. This optionally wakes the body.
        ///	@param bodyId The body id
        /// @param impulse the world impulse vector, typically in N*s or kg*m/s.
        /// @param wake also wake up the body
        ///	@warning This should be used for one-shot impulses. If you need a steady force,
        /// use a force instead, which will work better with the sub-stepping solver.
        void ApplyLinearImpulseToCenter(b2Vec2 impulse, bool wake) /*non-const*/ requires (!ForceConst);

        /// This update the mass properties to the sum of the mass properties of the shapes.
        /// This normally does not need to be called unless you called SetMassData to override
        /// the mass and you later want to reset the mass.
        ///	You may also use this when automatic mass computation has been disabled.
        ///	You should call this regardless of body type.
        void ApplyMassFromShapes() /*non-const*/ requires (!ForceConst);

        /// Apply a torque. This affects the angular velocity without affecting the linear velocity.
        ///	This optionally wakes the body. The torque is ignored if the body is not awake.
        ///	@param bodyId The body id
        /// @param torque about the z-axis (out of the screen), typically in N*m.
        /// @param wake also wake up the body
        void ApplyTorque(float torque, bool wake) /*non-const*/ requires (!ForceConst);

        /// Set the automatic mass setting. Normally this is set in b2BodyDef before creation.
        ///	@see b2BodyDef::automaticMass
        void SetAutomaticMass(bool automaticMass) /*non-const*/ requires (!ForceConst);

        /// Get the automatic mass setting
        [[nodiscard]] bool GetAutomaticMass() const;

        /// Wake a body from sleep. This wakes the entire island the body is touching.
        ///	@warning Putting a body to sleep will put the entire island of bodies touching this body to sleep,
        ///	which can be expensive and possibly unintuitive.
        void SetAwake(bool awake) /*non-const*/ requires (!ForceConst);

        /// @return true if this body is awake
        [[nodiscard]] bool IsAwake() const;

        /// Set this body to be a bullet. A bullet does continuous collision detection
        /// against dynamic bodies (but not other bullets).
        void SetBullet(bool flag) /*non-const*/ requires (!ForceConst);

        /// Is this body a bullet?
        [[nodiscard]] bool IsBullet() const;

        /// Get the current world AABB that contains all the attached shapes. Note that this may not encompass the body origin.
        ///	If there are no shapes attached then the returned AABB is empty and centered on the body origin.
        [[nodiscard]] b2AABB ComputeAABB() const;

        /// Get the maximum capacity required for retrieving all the touching contacts on a body
        [[nodiscard]] int GetContactCapacity() const;

        /// Get the touching contact data for a body
        [[nodiscard]] int GetContactData(b2ContactData& contactData, int capacity) const;

        /// Disable a body by removing it completely from the simulation. This is expensive.
        void Disable() /*non-const*/ requires (!ForceConst);

        /// Set this body to have fixed rotation. This causes the mass to be reset in all cases.
        void SetFixedRotation(bool flag) /*non-const*/ requires (!ForceConst);

        /// Does this body have fixed rotation?
        [[nodiscard]] bool IsFixedRotation() const;

        /// Adjust the gravity scale. Normally this is set in b2BodyDef before creation.
        ///	@see b2BodyDef::gravityScale
        void SetGravityScale(float gravityScale) /*non-const*/ requires (!ForceConst);

        /// Get the current gravity scale
        [[nodiscard]] float GetGravityScale() const;

        /// Enable/disable hit events on all shapes
        ///	@see b2ShapeDef::enableHitEvents
        void EnableHitEvents(bool enableHitEvents) /*non-const*/ requires (!ForceConst);

        /// Get the inertia tensor of the body, typically in kg*m^2
        [[nodiscard]] float GetInertiaTensor() const;

        /// Get the number of joints on this body
        [[nodiscard]] int GetJointCount() const;

        /// Get the joint ids for all joints on this body, up to the provided capacity
        ///	@returns the number of joint ids stored in the user array
        [[nodiscard]] int GetJoints(b2JointId& jointArray, int capacity) const;

        /// Adjust the linear damping. Normally this is set in b2BodyDef before creation.
        void SetLinearDamping(float linearDamping) /*non-const*/ requires (!ForceConst);

        /// Get the current linear damping.
        [[nodiscard]] float GetLinearDamping() const;

        /// Set the linear velocity of a body. Typically in meters per second.
        void SetLinearVelocity(b2Vec2 linearVelocity) /*non-const*/ requires (!ForceConst);

        /// Get the linear velocity of a body's center of mass. Typically in meters per second.
        [[nodiscard]] b2Vec2 GetLinearVelocity() const;

        /// Get the center of mass position of the body in local space
        [[nodiscard]] b2Vec2 GetLocalCenterOfMass() const;

        /// Get a local point on a body given a world point
        [[nodiscard]] b2Vec2 GetLocalPoint(b2Vec2 worldPoint) const;

        /// Get a local vector on a body given a world vector
        [[nodiscard]] b2Vec2 GetLocalVector(b2Vec2 worldVector) const;

        /// Get the mass of the body, typically in kilograms
        [[nodiscard]] float GetMass() const;

        /// Override the body's mass properties. Normally this is computed automatically using the
        ///	shape geometry and density. This information is lost if a shape is added or removed or if the
        ///	body type changes.
        void SetMassData(b2MassData massData) /*non-const*/ requires (!ForceConst);

        /// Get the mass data for a body
        [[nodiscard]] b2MassData GetMassData() const;

        /// Get the world position of a body. This is the location of the body origin.
        [[nodiscard]] b2Vec2 GetPosition() const;

        /// Get the world rotation of a body as a cosine/sine pair (complex number)
        [[nodiscard]] b2Rot GetRotation() const;

        /// Get the number of shapes on this body
        [[nodiscard]] int GetShapeCount() const;

        /// Get the shape ids for all shapes on this body, up to the provided capacity.
        ///	@returns the number of shape ids stored in the user array
        [[nodiscard]] int GetShapes(b2ShapeId& shapeArray, int capacity) const;

        /// Enable or disable sleeping for this body. If sleeping is disabled the body will wake.
        void EnableSleep(bool enableSleep) /*non-const*/ requires (!ForceConst);

        /// Returns true if sleeping is enabled for this body
        [[nodiscard]] bool IsSleepEnabled() const;

        /// Set the sleep threshold, typically in meters per second
        void SetSleepThreshold(float sleepVelocity) /*non-const*/ requires (!ForceConst);

        /// Get the sleep threshold, typically in meters per second.
        [[nodiscard]] float GetSleepThreshold() const;

        /// Set the world transform of a body. This acts as a teleport and is fairly expensive.
        /// @note Generally you should create a body with then intended transform.
        ///	@see b2BodyDef::position and b2BodyDef::angle
        void SetTransform(b2Vec2 position, b2Rot rotation) /*non-const*/ requires (!ForceConst);

        /// Get the world transform of a body.
        [[nodiscard]] b2Transform GetTransform() const;

        /// Change the body type. This is an expensive operation. This automatically updates the mass
        ///	properties regardless of the automatic mass setting.
        void SetType(b2BodyType type) /*non-const*/ requires (!ForceConst);

        /// Get the body type: static, kinematic, or dynamic
        [[nodiscard]] b2BodyType GetType() const;

        /// Set the user data for a body
        void SetUserData(void* userData) /*non-const*/ requires (!ForceConst);

        /// Get the user data stored in a body
        [[nodiscard]] void* GetUserData() const;

        /// Get the center of mass position of the body in world space
        [[nodiscard]] b2Vec2 GetWorldCenterOfMass() const;

        /// Get a world point on a body given a local point
        [[nodiscard]] b2Vec2 GetWorldPoint(b2Vec2 localPoint) const;

        /// Get a world vector on a body given a local vector
        [[nodiscard]] b2Vec2 GetWorldVector(b2Vec2 localVector) const;
    };

    /// A body definition holds all the data needed to construct a rigid body.
    /// You can safely re-use body definitions. Shapes are added to a body after construction.
    ///	Body definitions are temporary objects used to bundle creation parameters.
    /// Must be initialized using b2DefaultBodyDef().
    /// @ingroup body
    class Body : public BasicBodyInterface<Body, false>
    {
        template <typename, bool>
        friend class BasicBodyInterface;
        template <typename, bool>
        friend class BasicWorldInterface;

      protected:
        b2BodyId id{};

      public:
        static constexpr bool IsOwning = true;

        // Constructs a null (invalid) object.
        constexpr Body() noexcept {}

        // The constructor accepts either this or directly `b2BodyDef`.
        struct Params : b2BodyDef
        {
            Params() : b2BodyDef(b2DefaultBodyDef()) {}
        };

        Body(Body&& other) noexcept { id = std::exchange(other.id, b2BodyId{}); }
        Body& operator=(Body other) noexcept { std::swap(id, other.id); return *this; }

        ~Body() { if (*this) Destroy(); }
    };

    template <bool IsConstRef>
    class MaybeConstBodyRef : public BasicBodyInterface<BodyRef, IsConstRef>
    {
        template <typename, bool>
        friend class BasicBodyInterface;

      protected:
        b2BodyId id{};

      public:
        static constexpr bool IsOwning = false;
        static constexpr bool IsConst = IsConstRef;

        // Constructs a null (invalid) object.
        constexpr MaybeConstBodyRef() noexcept {}

        // Point to an existing handle.
        // Using a `same_as` template to prevent implicit madness. In particular, to prevent const-to-non-const conversions between non-owning wrappers.
        constexpr MaybeConstBodyRef(std::same_as<b2BodyId> auto id) noexcept { this->id = id; }

        // Create from a non-reference.
        constexpr MaybeConstBodyRef(const Body& other) noexcept : MaybeConstBodyRef(other.Handle()) {}

        // Convert a non-const reference to a const reference.
        constexpr MaybeConstBodyRef(const MaybeConstBodyRef<!IsConstRef>& other) noexcept requires IsConstRef : MaybeConstBodyRef(other.Handle()) {}
    };

    template <typename D, bool ForceConst>
    class BasicWorldInterface
    {
      protected:
        BasicWorldInterface() = default;

      public:
        [[nodiscard]] explicit operator bool() const { return B2_IS_NON_NULL(Handle()); }
        [[nodiscard]] const b2WorldId &Handle() const { return static_cast<const D &>(*this).id; }
        // Convert to a handle. Using a `same_as` template to prevent implicit madness. In particular, to prevent const-to-non-const conversions between non-owning wrappers.
        // I want this to return a const reference, but GCC 13 and earlier choke on that (fixed in 14). GCC bug: https://gcc.gnu.org/bugzilla/show_bug.cgi?id=61663
        template <std::same_as<b2WorldId> T> [[nodiscard]] operator T() const { return Handle(); }

        /// Create a rigid body given a definition. No reference to the definition is retained. So you can create the definition
        ///	on the stack and pass it as a pointer.
        ///	@code{.c}
        ///	b2BodyDef bodyDef = b2DefaultBodyDef();
        ///	b2BodyId myBodyId = b2CreateBody(myWorldId, &bodyDef);
        ///	@endcode
        /// @warning This function is locked during callbacks.
        [[nodiscard]] Body CreateBody(Tags::OwningHandle, const std::derived_from<b2BodyDef> auto& def) /*non-const*/ requires (!ForceConst);
        BodyRef CreateBody(Tags::DestroyWithParent, const std::derived_from<b2BodyDef> auto& def) /*non-const*/ requires (!ForceConst);

        /// Create a distance joint
        ///	@see b2DistanceJointDef for details
        [[nodiscard]] DistanceJoint CreateJoint(Tags::OwningHandle, const std::derived_from<b2DistanceJointDef> auto& def) /*non-const*/ requires (!ForceConst);
        DistanceJointRef CreateJoint(Tags::DestroyWithParent, const std::derived_from<b2DistanceJointDef> auto& def) /*non-const*/ requires (!ForceConst);

        /// Create a motor joint
        ///	@see b2MotorJointDef for details
        [[nodiscard]] MotorJoint CreateJoint(Tags::OwningHandle, const std::derived_from<b2MotorJointDef> auto& def) /*non-const*/ requires (!ForceConst);
        MotorJointRef CreateJoint(Tags::DestroyWithParent, const std::derived_from<b2MotorJointDef> auto& def) /*non-const*/ requires (!ForceConst);

        /// Create a mouse joint
        ///	@see b2MouseJointDef for details
        [[nodiscard]] MouseJoint CreateJoint(Tags::OwningHandle, const std::derived_from<b2MouseJointDef> auto& def) /*non-const*/ requires (!ForceConst);
        MouseJointRef CreateJoint(Tags::DestroyWithParent, const std::derived_from<b2MouseJointDef> auto& def) /*non-const*/ requires (!ForceConst);

        /// Create a prismatic (slider) joint.
        ///	@see b2PrismaticJointDef for details
        [[nodiscard]] PrismaticJoint CreateJoint(Tags::OwningHandle, const std::derived_from<b2PrismaticJointDef> auto& def) /*non-const*/ requires (!ForceConst);
        PrismaticJointRef CreateJoint(Tags::DestroyWithParent, const std::derived_from<b2PrismaticJointDef> auto& def) /*non-const*/ requires (!ForceConst);

        /// Create a revolute joint
        ///	@see b2RevoluteJointDef for details
        [[nodiscard]] RevoluteJoint CreateJoint(Tags::OwningHandle, const std::derived_from<b2RevoluteJointDef> auto& def) /*non-const*/ requires (!ForceConst);
        RevoluteJointRef CreateJoint(Tags::DestroyWithParent, const std::derived_from<b2RevoluteJointDef> auto& def) /*non-const*/ requires (!ForceConst);

        /// Create a weld joint
        ///	@see b2WeldJointDef for details
        [[nodiscard]] WeldJoint CreateJoint(Tags::OwningHandle, const std::derived_from<b2WeldJointDef> auto& def) /*non-const*/ requires (!ForceConst);
        WeldJointRef CreateJoint(Tags::DestroyWithParent, const std::derived_from<b2WeldJointDef> auto& def) /*non-const*/ requires (!ForceConst);

        /// Create a wheel joint
        ///	@see b2WheelJointDef for details
        [[nodiscard]] WheelJoint CreateJoint(Tags::OwningHandle, const std::derived_from<b2WheelJointDef> auto& def) /*non-const*/ requires (!ForceConst);
        WheelJointRef CreateJoint(Tags::DestroyWithParent, const std::derived_from<b2WheelJointDef> auto& def) /*non-const*/ requires (!ForceConst);

        /// Destroy a world
        void Destroy() /*non-const*/ requires (!ForceConst);

        /// World id validation. Provides validation for up to 64K allocations.
        [[nodiscard]] bool IsValid() const;

        /// Get the body events for the current time step. The event data is transient. Do not store a reference to this data.
        [[nodiscard]] b2BodyEvents GetBodyEvents() const;

        /// Cast a capsule through the world. Similar to a cast ray except that a capsule is cast instead of a point.
        void Cast(const b2Capsule& capsule, b2Transform originTransform, b2Vec2 translation, b2QueryFilter filter, detail::FuncRef<b2CastResultFcn,false> fcn) /*non-const*/ requires (!ForceConst);
        void Cast(const b2Capsule& capsule, b2Transform originTransform, b2Vec2 translation, b2QueryFilter filter, detail::FuncRef<b2CastResultFcn,true> fcn) const;

        /// Cast a circle through the world. Similar to a cast ray except that a circle is cast instead of a point.
        void Cast(const b2Circle& circle, b2Transform originTransform, b2Vec2 translation, b2QueryFilter filter, detail::FuncRef<b2CastResultFcn,false> fcn) /*non-const*/ requires (!ForceConst);
        void Cast(const b2Circle& circle, b2Transform originTransform, b2Vec2 translation, b2QueryFilter filter, detail::FuncRef<b2CastResultFcn,true> fcn) const;

        /// Cast a polygon through the world. Similar to a cast ray except that a polygon is cast instead of a point.
        void Cast(const b2Polygon& polygon, b2Transform originTransform, b2Vec2 translation, b2QueryFilter filter, detail::FuncRef<b2CastResultFcn,false> fcn) /*non-const*/ requires (!ForceConst);
        void Cast(const b2Polygon& polygon, b2Transform originTransform, b2Vec2 translation, b2QueryFilter filter, detail::FuncRef<b2CastResultFcn,true> fcn) const;

        /// Cast a ray into the world to collect shapes in the path of the ray.
        /// Your callback function controls whether you get the closest point, any point, or n-points.
        /// The ray-cast ignores shapes that contain the starting point.
        ///	@param worldId The world to cast the ray against
        ///	@param origin The start point of the ray
        ///	@param translation The translation of the ray from the start point to the end point
        ///	@param filter Contains bit flags to filter unwanted shapes from the results
        /// @param fcn A user implemented callback function
        /// @param context A user context that is passed along to the callback function
        ///	@note The callback function may receive shapes in any order
        void CastRay(b2Vec2 origin, b2Vec2 translation, b2QueryFilter filter, detail::FuncRef<b2CastResultFcn,false> fcn) /*non-const*/ requires (!ForceConst);
        void CastRay(b2Vec2 origin, b2Vec2 translation, b2QueryFilter filter, detail::FuncRef<b2CastResultFcn,true> fcn) const;

        /// Cast a ray into the world to collect the closest hit. This is a convenience function.
        /// This is less general than b2World_CastRay() and does not allow for custom filtering.
        [[nodiscard]] b2RayResult CastRayClosest(b2Vec2 origin, b2Vec2 translation, b2QueryFilter filter) const;

        /// Get contact events for this current time step. The event data is transient. Do not store a reference to this data.
        [[nodiscard]] b2ContactEvents GetContactEvents() const;

        /// Adjust contact tuning parameters
        ///	@param worldId The world id
        /// @param hertz The contact stiffness (cycles per second)
        /// @param dampingRatio The contact bounciness with 1 being critical damping (non-dimensional)
        /// @param pushVelocity The maximum contact constraint push out velocity (meters per second)
        ///	@note Advanced feature
        void SetContactTuning(float hertz, float dampingRatio, float pushVelocity) /*non-const*/ requires (!ForceConst);

        /// Enable/disable continuous collision between dynamic and static bodies. Generally you should keep continuous
        /// collision enabled to prevent fast moving objects from going through static objects. The performance gain from
        ///	disabling continuous collision is minor.
        ///	@see b2WorldDef
        void EnableContinuous(bool flag) /*non-const*/ requires (!ForceConst);

        /// Get world counters and sizes
        [[nodiscard]] b2Counters GetCounters() const;

        /// Register the custom filter callback. This is optional.
        void SetCustomFilterCallback(b2CustomFilterFcn* fcn, void* context) /*non-const*/ requires (!ForceConst);

        /// Call this to draw shapes and other debug draw data
        void Draw(b2DebugDraw& draw) const;

        /// Dump memory stats to box2d_memory.txt
        void DumpMemoryStats() /*non-const*/ requires (!ForceConst);

        /// Apply a radial explosion
        ///	@param worldId The world id
        ///	@param position The center of the explosion
        ///	@param radius The radius of the explosion
        ///	@param impulse The impulse of the explosion, typically in kg * m / s or N * s.
        void Explode(b2Vec2 position, float radius, float impulse) /*non-const*/ requires (!ForceConst);

        /// Set the gravity vector for the entire world. Box2D has no concept of an up direction and this
        /// is left as a decision for the application. Typically in m/s^2.
        ///	@see b2WorldDef
        void SetGravity(b2Vec2 gravity) /*non-const*/ requires (!ForceConst);

        /// Get the gravity vector
        [[nodiscard]] b2Vec2 GetGravity() const;

        /// Adjust the hit event threshold. This controls the collision velocity needed to generate a b2ContactHitEvent.
        /// Typically in meters per second.
        ///	@see b2WorldDef::hitEventThreshold
        void SetHitEventThreshold(float value) /*non-const*/ requires (!ForceConst);

        /// Overlap test for all shapes that *potentially* overlap the provided AABB
        void Overlap(b2AABB aabb, b2QueryFilter filter, detail::FuncRef<b2OverlapResultFcn,false> fcn) /*non-const*/ requires (!ForceConst);
        void Overlap(b2AABB aabb, b2QueryFilter filter, detail::FuncRef<b2OverlapResultFcn,true> fcn) const;

        /// Overlap test for all shapes that overlap the provided capsule
        void Overlap(const b2Capsule& capsule, b2Transform transform, b2QueryFilter filter, detail::FuncRef<b2OverlapResultFcn,false> fcn) /*non-const*/ requires (!ForceConst);
        void Overlap(const b2Capsule& capsule, b2Transform transform, b2QueryFilter filter, detail::FuncRef<b2OverlapResultFcn,true> fcn) const;

        /// Overlap test for for all shapes that overlap the provided circle
        void Overlap(const b2Circle& circle, b2Transform transform, b2QueryFilter filter, detail::FuncRef<b2OverlapResultFcn,false> fcn) /*non-const*/ requires (!ForceConst);
        void Overlap(const b2Circle& circle, b2Transform transform, b2QueryFilter filter, detail::FuncRef<b2OverlapResultFcn,true> fcn) const;

        /// Overlap test for all shapes that overlap the provided polygon
        void Overlap(const b2Polygon& polygon, b2Transform transform, b2QueryFilter filter, detail::FuncRef<b2OverlapResultFcn,false> fcn) /*non-const*/ requires (!ForceConst);
        void Overlap(const b2Polygon& polygon, b2Transform transform, b2QueryFilter filter, detail::FuncRef<b2OverlapResultFcn,true> fcn) const;

        /// Register the pre-solve callback. This is optional.
        void SetPreSolveCallback(b2PreSolveFcn* fcn, void* context) /*non-const*/ requires (!ForceConst);

        /// Get the current world performance profile
        [[nodiscard]] b2Profile GetProfile() const;

        /// Adjust the restitution threshold. It is recommended not to make this value very small
        ///	because it will prevent bodies from sleeping. Typically in meters per second.
        ///	@see b2WorldDef
        void SetRestitutionThreshold(float value) /*non-const*/ requires (!ForceConst);

        /// Get sensor events for the current time step. The event data is transient. Do not store a reference to this data.
        [[nodiscard]] b2SensorEvents GetSensorEvents() const;

        /// Enable/disable sleep. If your application does not need sleeping, you can gain some performance
        ///	by disabling sleep completely at the world level.
        ///	@see b2WorldDef
        void EnableSleeping(bool flag) /*non-const*/ requires (!ForceConst);

        /// Simulate a world for one time step. This performs collision detection, integration, and constraint solution.
        /// @param worldId The world to simulate
        /// @param timeStep The amount of time to simulate, this should be a fixed number. Typically 1/60.
        /// @param subStepCount The number of sub-steps, increasing the sub-step count can increase accuracy. Typically 4.
        void Step(float timeStep, int subStepCount) /*non-const*/ requires (!ForceConst);

        /// Enable/disable constraint warm starting. Advanced feature for testing. Disabling
        ///	sleeping greatly reduces stability and provides no performance gain.
        void EnableWarmStarting(bool flag) /*non-const*/ requires (!ForceConst);
    };

    /// World definition used to create a simulation world.
    /// Must be initialized using b2DefaultWorldDef().
    /// @ingroup world
    class World : public BasicWorldInterface<World, false>
    {
        template <typename, bool>
        friend class BasicWorldInterface;

      protected:
        b2WorldId id{};

      public:
        static constexpr bool IsOwning = true;

        // Constructs a null (invalid) object.
        constexpr World() noexcept {}

        // The constructor accepts either this or directly `b2WorldDef`.
        struct Params : b2WorldDef
        {
            Params() : b2WorldDef(b2DefaultWorldDef()) {}
        };

        /// Create a world for rigid body simulation. A world contains bodies, shapes, and constraints. You make create
        ///	up to 128 worlds. Each world is completely independent and may be simulated in parallel.
        ///	@return the world id.
        World(const std::derived_from<b2WorldDef> auto &params) { id = b2CreateWorld(&params); }

        World(World&& other) noexcept { id = std::exchange(other.id, b2WorldId{}); }
        World& operator=(World other) noexcept { std::swap(id, other.id); return *this; }

        ~World() { if (*this) Destroy(); }
    };

    template <bool IsConstRef>
    class MaybeConstWorldRef : public BasicWorldInterface<WorldRef, IsConstRef>
    {
        template <typename, bool>
        friend class BasicWorldInterface;

      protected:
        b2WorldId id{};

      public:
        static constexpr bool IsOwning = false;
        static constexpr bool IsConst = IsConstRef;

        // Constructs a null (invalid) object.
        constexpr MaybeConstWorldRef() noexcept {}

        // Point to an existing handle.
        // Using a `same_as` template to prevent implicit madness. In particular, to prevent const-to-non-const conversions between non-owning wrappers.
        constexpr MaybeConstWorldRef(std::same_as<b2WorldId> auto id) noexcept { this->id = id; }

        // Create from a non-reference.
        constexpr MaybeConstWorldRef(const World& other) noexcept : MaybeConstWorldRef(other.Handle()) {}

        // Convert a non-const reference to a const reference.
        constexpr MaybeConstWorldRef(const MaybeConstWorldRef<!IsConstRef>& other) noexcept requires IsConstRef : MaybeConstWorldRef(other.Handle()) {}
    };

    /// The dynamic tree structure. This should be considered private data.
    /// It is placed here for performance reasons.
    class DynamicTree
    {
        b2DynamicTree value{};

      public:
        // Consturcts a null (invalid) object.
        constexpr DynamicTree() {}

        /// Constructing the tree initializes the node pool.
        DynamicTree(std::nullptr_t) : value(b2DynamicTree_Create()) {}

        DynamicTree(DynamicTree&& other) noexcept : value(other.value) { other.value = {}; }
        DynamicTree& operator=(DynamicTree&& other) noexcept
        {
            if (this == &other) return *this;
            if (*this) *this = {};
            value = other.value;
            other.value = {};
            return *this;
        }

        /// Destroy the tree, freeing the node pool.
        ~DynamicTree() { if (*this) b2DynamicTree_Destroy(&value); }

        [[nodiscard]] explicit operator bool() const { return bool( value.nodes ); }
        [[nodiscard]]       b2DynamicTree *RawTreePtr()       { return *this ? &value : nullptr; }
        [[nodiscard]] const b2DynamicTree *RawTreePtr() const { return *this ? &value : nullptr; }

        /// Get the AABB of a proxy
        [[nodiscard]] b2AABB GetAABB(int32_t proxyId) const;

        /// Get the ratio of the sum of the node areas to the root area.
        [[nodiscard]] float GetAreaRatio() const;

        /// Get the number of bytes used by this tree
        [[nodiscard]] int GetByteCount() const;

        /// Create a proxy. Provide an AABB and a userData value.
        [[nodiscard]] int32_t CreateProxy(b2AABB aabb, uint32_t categoryBits, int32_t userData);

        /// Destroy a proxy. This asserts if the id is invalid.
        void DestroyProxy(int32_t proxyId);

        /// Enlarge a proxy and enlarge ancestors as necessary.
        void EnlargeProxy(int32_t proxyId, b2AABB aabb);

        /// Compute the height of the binary tree in O(N) time. Should not be
        /// called often.
        [[nodiscard]] int GetHeight() const;

        /// Get the maximum balance of the tree. The balance is the difference in height of the two children of a node.
        [[nodiscard]] int GetMaxBalance() const;

        /// Move a proxy to a new AABB by removing and reinserting into the tree.
        void MoveProxy(int32_t proxyId, b2AABB aabb);

        /// Get the number of proxies created
        [[nodiscard]] int GetProxyCount() const;

        /// Query an AABB for overlapping proxies. The callback class
        /// is called for each proxy that overlaps the supplied AABB.
        void Query(b2AABB aabb, uint32_t maskBits, b2TreeQueryCallbackFcn* callback, void* context) const;

        /// Ray-cast against the proxies in the tree. This relies on the callback
        /// to perform a exact ray-cast in the case were the proxy contains a shape.
        /// The callback also performs the any collision filtering. This has performance
        /// roughly equal to k * log(n), where k is the number of collisions and n is the
        /// number of proxies in the tree.
        ///	Bit-wise filtering using mask bits can greatly improve performance in some scenarios.
        ///	@param tree the dynamic tree to ray cast
        /// @param input the ray-cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1)
        ///	@param maskBits filter bits: `bool accept = (maskBits & node->categoryBits) != 0;`
        /// @param callback a callback class that is called for each proxy that is hit by the ray
        ///	@param context user context that is passed to the callback
        void RayCast(const b2RayCastInput& input, uint32_t maskBits, b2TreeRayCastCallbackFcn* callback, void* context) const;

        /// Rebuild the tree while retaining subtrees that haven't changed. Returns the number of boxes sorted.
        int Rebuild(bool fullBuild);

        /// Build an optimal tree. Very expensive. For testing.
        void RebuildBottomUp();

        /// Ray-cast against the proxies in the tree. This relies on the callback
        /// to perform a exact ray-cast in the case were the proxy contains a shape.
        /// The callback also performs the any collision filtering. This has performance
        /// roughly equal to k * log(n), where k is the number of collisions and n is the
        /// number of proxies in the tree.
        ///	@param tree the dynamic tree to ray cast
        /// @param input the ray-cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1).
        ///	@param maskBits filter bits: `bool accept = (maskBits & node->categoryBits) != 0;`
        /// @param callback a callback class that is called for each proxy that is hit by the shape
        ///	@param context user context that is passed to the callback
        void ShapeCast(const b2ShapeCastInput& input, uint32_t maskBits, b2TreeShapeCastCallbackFcn* callback, void* context) const;

        /// Shift the world origin. Useful for large worlds.
        /// The shift formula is: position -= newOrigin
        /// @param tree the tree to shift
        /// @param newOrigin the new origin with respect to the old origin
        void ShiftOrigin(b2Vec2 newOrigin);

        /// Get proxy user data
        /// @return the proxy user data or 0 if the id is invalid
        [[nodiscard]] int32_t GetUserData(int32_t proxyId) const;

        /// Validate this tree. For testing.
        void Validate() const;
    };
} // namespace box2d

// Implementations:
namespace b2
{
    template <typename D, bool ForceConst> void BasicChainInterface<D, ForceConst>::Destroy() requires (!ForceConst) { if (*this) { b2DestroyChain(static_cast<const D &>(*this).Handle()); static_cast<D &>(*this).id = {}; } }
    template <typename D, bool ForceConst> bool BasicChainInterface<D, ForceConst>::IsValid() const { return b2Chain_IsValid(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicChainInterface<D, ForceConst>::SetFriction(float friction) requires (!ForceConst) { b2Chain_SetFriction(static_cast<const D &>(*this).Handle(), friction); }
    template <typename D, bool ForceConst> void BasicChainInterface<D, ForceConst>::SetRestitution(float restitution) requires (!ForceConst) { b2Chain_SetRestitution(static_cast<const D &>(*this).Handle(), restitution); }
    template <typename D, bool ForceConst> void BasicShapeInterface<D, ForceConst>::Destroy() requires (!ForceConst) { if (*this) { b2DestroyShape(static_cast<const D &>(*this).Handle()); static_cast<D &>(*this).id = {}; } }
    template <typename D, bool ForceConst> bool BasicShapeInterface<D, ForceConst>::IsValid() const { return b2Shape_IsValid(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicShapeInterface<D, ForceConst>::Set(const b2Capsule& capsule) requires (!ForceConst) { b2Shape_SetCapsule(static_cast<const D &>(*this).Handle(), &capsule); }
    template <typename D, bool ForceConst> void BasicShapeInterface<D, ForceConst>::Set(const b2Circle& circle) requires (!ForceConst) { b2Shape_SetCircle(static_cast<const D &>(*this).Handle(), &circle); }
    template <typename D, bool ForceConst> void BasicShapeInterface<D, ForceConst>::Set(const b2Polygon& polygon) requires (!ForceConst) { b2Shape_SetPolygon(static_cast<const D &>(*this).Handle(), &polygon); }
    template <typename D, bool ForceConst> void BasicShapeInterface<D, ForceConst>::Set(const b2Segment& segment) requires (!ForceConst) { b2Shape_SetSegment(static_cast<const D &>(*this).Handle(), &segment); }
    template <typename D, bool ForceConst> b2AABB BasicShapeInterface<D, ForceConst>::GetAABB() const { return b2Shape_GetAABB(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> BodyRef BasicShapeInterface<D, ForceConst>::GetBody() requires (!ForceConst) { return b2Shape_GetBody(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> BodyConstRef BasicShapeInterface<D, ForceConst>::GetBody() const { return b2Shape_GetBody(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> b2Capsule BasicShapeInterface<D, ForceConst>::GetCapsule() const { return b2Shape_GetCapsule(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> b2Circle BasicShapeInterface<D, ForceConst>::GetCircle() const { return b2Shape_GetCircle(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> b2Vec2 BasicShapeInterface<D, ForceConst>::GetClosestPoint(b2Vec2 target) const { return b2Shape_GetClosestPoint(static_cast<const D &>(*this).Handle(), target); }
    template <typename D, bool ForceConst> int BasicShapeInterface<D, ForceConst>::GetContactCapacity() const { return b2Shape_GetContactCapacity(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> int BasicShapeInterface<D, ForceConst>::GetContactData(b2ContactData& contactData, int capacity) const { return b2Shape_GetContactData(static_cast<const D &>(*this).Handle(), &contactData, capacity); }
    template <typename D, bool ForceConst> void BasicShapeInterface<D, ForceConst>::EnableContactEvents(bool flag) requires (!ForceConst) { b2Shape_EnableContactEvents(static_cast<const D &>(*this).Handle(), flag); }
    template <typename D, bool ForceConst> bool BasicShapeInterface<D, ForceConst>::AreContactEventsEnabled() const { return b2Shape_AreContactEventsEnabled(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicShapeInterface<D, ForceConst>::SetDensity(float density) requires (!ForceConst) { b2Shape_SetDensity(static_cast<const D &>(*this).Handle(), density); }
    template <typename D, bool ForceConst> float BasicShapeInterface<D, ForceConst>::GetDensity() const { return b2Shape_GetDensity(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicShapeInterface<D, ForceConst>::SetFilter(b2Filter filter) requires (!ForceConst) { b2Shape_SetFilter(static_cast<const D &>(*this).Handle(), filter); }
    template <typename D, bool ForceConst> b2Filter BasicShapeInterface<D, ForceConst>::GetFilter() const { return b2Shape_GetFilter(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicShapeInterface<D, ForceConst>::SetFriction(float friction) requires (!ForceConst) { b2Shape_SetFriction(static_cast<const D &>(*this).Handle(), friction); }
    template <typename D, bool ForceConst> float BasicShapeInterface<D, ForceConst>::GetFriction() const { return b2Shape_GetFriction(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicShapeInterface<D, ForceConst>::EnableHitEvents(bool flag) requires (!ForceConst) { b2Shape_EnableHitEvents(static_cast<const D &>(*this).Handle(), flag); }
    template <typename D, bool ForceConst> bool BasicShapeInterface<D, ForceConst>::AreHitEventsEnabled() const { return b2Shape_AreHitEventsEnabled(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> ChainRef BasicShapeInterface<D, ForceConst>::GetParentChain() requires (!ForceConst) { return b2Shape_GetParentChain(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> ChainConstRef BasicShapeInterface<D, ForceConst>::GetParentChain() const { return b2Shape_GetParentChain(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> b2Polygon BasicShapeInterface<D, ForceConst>::GetPolygon() const { return b2Shape_GetPolygon(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicShapeInterface<D, ForceConst>::EnablePreSolveEvents(bool flag) requires (!ForceConst) { b2Shape_EnablePreSolveEvents(static_cast<const D &>(*this).Handle(), flag); }
    template <typename D, bool ForceConst> bool BasicShapeInterface<D, ForceConst>::ArePreSolveEventsEnabled() const { return b2Shape_ArePreSolveEventsEnabled(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> b2CastOutput BasicShapeInterface<D, ForceConst>::RayCast(b2Vec2 origin, b2Vec2 translation) const { return b2Shape_RayCast(static_cast<const D &>(*this).Handle(), origin, translation); }
    template <typename D, bool ForceConst> void BasicShapeInterface<D, ForceConst>::SetRestitution(float restitution) requires (!ForceConst) { b2Shape_SetRestitution(static_cast<const D &>(*this).Handle(), restitution); }
    template <typename D, bool ForceConst> float BasicShapeInterface<D, ForceConst>::GetRestitution() const { return b2Shape_GetRestitution(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> b2Segment BasicShapeInterface<D, ForceConst>::GetSegment() const { return b2Shape_GetSegment(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> bool BasicShapeInterface<D, ForceConst>::IsSensor() const { return b2Shape_IsSensor(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicShapeInterface<D, ForceConst>::EnableSensorEvents(bool flag) requires (!ForceConst) { b2Shape_EnableSensorEvents(static_cast<const D &>(*this).Handle(), flag); }
    template <typename D, bool ForceConst> bool BasicShapeInterface<D, ForceConst>::AreSensorEventsEnabled() const { return b2Shape_AreSensorEventsEnabled(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> b2SmoothSegment BasicShapeInterface<D, ForceConst>::GetSmoothSegment() const { return b2Shape_GetSmoothSegment(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> bool BasicShapeInterface<D, ForceConst>::TestPoint(b2Vec2 point) const { return b2Shape_TestPoint(static_cast<const D &>(*this).Handle(), point); }
    template <typename D, bool ForceConst> b2ShapeType BasicShapeInterface<D, ForceConst>::GetType() const { return b2Shape_GetType(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicShapeInterface<D, ForceConst>::SetUserData(void* userData) requires (!ForceConst) { b2Shape_SetUserData(static_cast<const D &>(*this).Handle(), userData); }
    template <typename D, bool ForceConst> void* BasicShapeInterface<D, ForceConst>::GetUserData() const { return b2Shape_GetUserData(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicJointInterface<D, ForceConst>::Destroy() requires (!ForceConst) { if (*this) { b2DestroyJoint(static_cast<const D &>(*this).Handle()); static_cast<D &>(*this).id = {}; } }
    template <typename D, bool ForceConst> bool BasicJointInterface<D, ForceConst>::IsValid() const { return b2Joint_IsValid(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> BodyRef BasicJointInterface<D, ForceConst>::GetBodyA() requires (!ForceConst) { return b2Joint_GetBodyA(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> BodyConstRef BasicJointInterface<D, ForceConst>::GetBodyA() const { return b2Joint_GetBodyA(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> BodyRef BasicJointInterface<D, ForceConst>::GetBodyB() requires (!ForceConst) { return b2Joint_GetBodyB(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> BodyConstRef BasicJointInterface<D, ForceConst>::GetBodyB() const { return b2Joint_GetBodyB(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicJointInterface<D, ForceConst>::SetCollideConnected(bool shouldCollide) requires (!ForceConst) { b2Joint_SetCollideConnected(static_cast<const D &>(*this).Handle(), shouldCollide); }
    template <typename D, bool ForceConst> bool BasicJointInterface<D, ForceConst>::GetCollideConnected() const { return b2Joint_GetCollideConnected(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> b2Vec2 BasicJointInterface<D, ForceConst>::GetConstraintForce() const { return b2Joint_GetConstraintForce(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> float BasicJointInterface<D, ForceConst>::GetConstraintTorque() const { return b2Joint_GetConstraintTorque(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> b2Vec2 BasicJointInterface<D, ForceConst>::GetLocalAnchorA() const { return b2Joint_GetLocalAnchorA(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> b2Vec2 BasicJointInterface<D, ForceConst>::GetLocalAnchorB() const { return b2Joint_GetLocalAnchorB(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> b2JointType BasicJointInterface<D, ForceConst>::GetType() const { return b2Joint_GetType(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicJointInterface<D, ForceConst>::SetUserData(void* userData) requires (!ForceConst) { b2Joint_SetUserData(static_cast<const D &>(*this).Handle(), userData); }
    template <typename D, bool ForceConst> void* BasicJointInterface<D, ForceConst>::GetUserData() const { return b2Joint_GetUserData(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicJointInterface<D, ForceConst>::WakeBodies() requires (!ForceConst) { b2Joint_WakeBodies(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> float BasicDistanceJointInterface<D, ForceConst>::GetCurrentLength() const { return b2DistanceJoint_GetCurrentLength(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> float BasicDistanceJointInterface<D, ForceConst>::GetDampingRatio() const { return b2DistanceJoint_GetDampingRatio(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> float BasicDistanceJointInterface<D, ForceConst>::GetHertz() const { return b2DistanceJoint_GetHertz(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicDistanceJointInterface<D, ForceConst>::SetLength(float length) requires (!ForceConst) { b2DistanceJoint_SetLength(static_cast<const D &>(*this).Handle(), length); }
    template <typename D, bool ForceConst> float BasicDistanceJointInterface<D, ForceConst>::GetLength() const { return b2DistanceJoint_GetLength(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicDistanceJointInterface<D, ForceConst>::SetLengthRange(float minLength, float maxLength) requires (!ForceConst) { b2DistanceJoint_SetLengthRange(static_cast<const D &>(*this).Handle(), minLength, maxLength); }
    template <typename D, bool ForceConst> void BasicDistanceJointInterface<D, ForceConst>::EnableLimit(bool enableLimit) requires (!ForceConst) { b2DistanceJoint_EnableLimit(static_cast<const D &>(*this).Handle(), enableLimit); }
    template <typename D, bool ForceConst> bool BasicDistanceJointInterface<D, ForceConst>::IsLimitEnabled() const { return b2DistanceJoint_IsLimitEnabled(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> float BasicDistanceJointInterface<D, ForceConst>::GetMaxLength() const { return b2DistanceJoint_GetMaxLength(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicDistanceJointInterface<D, ForceConst>::SetMaxMotorForce(float force) requires (!ForceConst) { b2DistanceJoint_SetMaxMotorForce(static_cast<const D &>(*this).Handle(), force); }
    template <typename D, bool ForceConst> float BasicDistanceJointInterface<D, ForceConst>::GetMaxMotorForce() const { return b2DistanceJoint_GetMaxMotorForce(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> float BasicDistanceJointInterface<D, ForceConst>::GetMinLength() const { return b2DistanceJoint_GetMinLength(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicDistanceJointInterface<D, ForceConst>::EnableMotor(bool enableMotor) requires (!ForceConst) { b2DistanceJoint_EnableMotor(static_cast<const D &>(*this).Handle(), enableMotor); }
    template <typename D, bool ForceConst> bool BasicDistanceJointInterface<D, ForceConst>::IsMotorEnabled() const { return b2DistanceJoint_IsMotorEnabled(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> float BasicDistanceJointInterface<D, ForceConst>::GetMotorForce() const { return b2DistanceJoint_GetMotorForce(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicDistanceJointInterface<D, ForceConst>::SetMotorSpeed(float motorSpeed) requires (!ForceConst) { b2DistanceJoint_SetMotorSpeed(static_cast<const D &>(*this).Handle(), motorSpeed); }
    template <typename D, bool ForceConst> float BasicDistanceJointInterface<D, ForceConst>::GetMotorSpeed() const { return b2DistanceJoint_GetMotorSpeed(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicDistanceJointInterface<D, ForceConst>::EnableSpring(bool enableSpring) requires (!ForceConst) { b2DistanceJoint_EnableSpring(static_cast<const D &>(*this).Handle(), enableSpring); }
    template <typename D, bool ForceConst> bool BasicDistanceJointInterface<D, ForceConst>::IsSpringEnabled() const { return b2DistanceJoint_IsSpringEnabled(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicDistanceJointInterface<D, ForceConst>::SetSpringDampingRatio(float dampingRatio) requires (!ForceConst) { b2DistanceJoint_SetSpringDampingRatio(static_cast<const D &>(*this).Handle(), dampingRatio); }
    template <typename D, bool ForceConst> void BasicDistanceJointInterface<D, ForceConst>::SetSpringHertz(float hertz) requires (!ForceConst) { b2DistanceJoint_SetSpringHertz(static_cast<const D &>(*this).Handle(), hertz); }
    template <typename D, bool ForceConst> void BasicMotorJointInterface<D, ForceConst>::SetAngularOffset(float angularOffset) requires (!ForceConst) { b2MotorJoint_SetAngularOffset(static_cast<const D &>(*this).Handle(), angularOffset); }
    template <typename D, bool ForceConst> float BasicMotorJointInterface<D, ForceConst>::GetAngularOffset() const { return b2MotorJoint_GetAngularOffset(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicMotorJointInterface<D, ForceConst>::SetCorrectionFactor(float correctionFactor) requires (!ForceConst) { b2MotorJoint_SetCorrectionFactor(static_cast<const D &>(*this).Handle(), correctionFactor); }
    template <typename D, bool ForceConst> float BasicMotorJointInterface<D, ForceConst>::GetCorrectionFactor() const { return b2MotorJoint_GetCorrectionFactor(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicMotorJointInterface<D, ForceConst>::SetLinearOffset(b2Vec2 linearOffset) requires (!ForceConst) { b2MotorJoint_SetLinearOffset(static_cast<const D &>(*this).Handle(), linearOffset); }
    template <typename D, bool ForceConst> b2Vec2 BasicMotorJointInterface<D, ForceConst>::GetLinearOffset() const { return b2MotorJoint_GetLinearOffset(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicMotorJointInterface<D, ForceConst>::SetMaxForce(float maxForce) requires (!ForceConst) { b2MotorJoint_SetMaxForce(static_cast<const D &>(*this).Handle(), maxForce); }
    template <typename D, bool ForceConst> float BasicMotorJointInterface<D, ForceConst>::GetMaxForce() const { return b2MotorJoint_GetMaxForce(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicMotorJointInterface<D, ForceConst>::SetMaxTorque(float maxTorque) requires (!ForceConst) { b2MotorJoint_SetMaxTorque(static_cast<const D &>(*this).Handle(), maxTorque); }
    template <typename D, bool ForceConst> float BasicMotorJointInterface<D, ForceConst>::GetMaxTorque() const { return b2MotorJoint_GetMaxTorque(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicMouseJointInterface<D, ForceConst>::SetMaxForce(float maxForce) requires (!ForceConst) { b2MouseJoint_SetMaxForce(static_cast<const D &>(*this).Handle(), maxForce); }
    template <typename D, bool ForceConst> float BasicMouseJointInterface<D, ForceConst>::GetMaxForce() const { return b2MouseJoint_GetMaxForce(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicMouseJointInterface<D, ForceConst>::SetSpringDampingRatio(float dampingRatio) requires (!ForceConst) { b2MouseJoint_SetSpringDampingRatio(static_cast<const D &>(*this).Handle(), dampingRatio); }
    template <typename D, bool ForceConst> float BasicMouseJointInterface<D, ForceConst>::GetSpringDampingRatio() const { return b2MouseJoint_GetSpringDampingRatio(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicMouseJointInterface<D, ForceConst>::SetSpringHertz(float hertz) requires (!ForceConst) { b2MouseJoint_SetSpringHertz(static_cast<const D &>(*this).Handle(), hertz); }
    template <typename D, bool ForceConst> float BasicMouseJointInterface<D, ForceConst>::GetSpringHertz() const { return b2MouseJoint_GetSpringHertz(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicMouseJointInterface<D, ForceConst>::SetTarget(b2Vec2 target) requires (!ForceConst) { b2MouseJoint_SetTarget(static_cast<const D &>(*this).Handle(), target); }
    template <typename D, bool ForceConst> b2Vec2 BasicMouseJointInterface<D, ForceConst>::GetTarget() const { return b2MouseJoint_GetTarget(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicPrismaticJointInterface<D, ForceConst>::EnableLimit(bool enableLimit) requires (!ForceConst) { b2PrismaticJoint_EnableLimit(static_cast<const D &>(*this).Handle(), enableLimit); }
    template <typename D, bool ForceConst> bool BasicPrismaticJointInterface<D, ForceConst>::IsLimitEnabled() const { return b2PrismaticJoint_IsLimitEnabled(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicPrismaticJointInterface<D, ForceConst>::SetLimits(float lower, float upper) requires (!ForceConst) { b2PrismaticJoint_SetLimits(static_cast<const D &>(*this).Handle(), lower, upper); }
    template <typename D, bool ForceConst> float BasicPrismaticJointInterface<D, ForceConst>::GetLowerLimit() const { return b2PrismaticJoint_GetLowerLimit(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicPrismaticJointInterface<D, ForceConst>::SetMaxMotorForce(float force) requires (!ForceConst) { b2PrismaticJoint_SetMaxMotorForce(static_cast<const D &>(*this).Handle(), force); }
    template <typename D, bool ForceConst> float BasicPrismaticJointInterface<D, ForceConst>::GetMaxMotorForce() const { return b2PrismaticJoint_GetMaxMotorForce(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicPrismaticJointInterface<D, ForceConst>::EnableMotor(bool enableMotor) requires (!ForceConst) { b2PrismaticJoint_EnableMotor(static_cast<const D &>(*this).Handle(), enableMotor); }
    template <typename D, bool ForceConst> bool BasicPrismaticJointInterface<D, ForceConst>::IsMotorEnabled() const { return b2PrismaticJoint_IsMotorEnabled(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> float BasicPrismaticJointInterface<D, ForceConst>::GetMotorForce() const { return b2PrismaticJoint_GetMotorForce(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicPrismaticJointInterface<D, ForceConst>::SetMotorSpeed(float motorSpeed) requires (!ForceConst) { b2PrismaticJoint_SetMotorSpeed(static_cast<const D &>(*this).Handle(), motorSpeed); }
    template <typename D, bool ForceConst> float BasicPrismaticJointInterface<D, ForceConst>::GetMotorSpeed() const { return b2PrismaticJoint_GetMotorSpeed(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicPrismaticJointInterface<D, ForceConst>::EnableSpring(bool enableSpring) requires (!ForceConst) { b2PrismaticJoint_EnableSpring(static_cast<const D &>(*this).Handle(), enableSpring); }
    template <typename D, bool ForceConst> bool BasicPrismaticJointInterface<D, ForceConst>::IsSpringEnabled() const { return b2PrismaticJoint_IsSpringEnabled(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicPrismaticJointInterface<D, ForceConst>::SetSpringDampingRatio(float dampingRatio) requires (!ForceConst) { b2PrismaticJoint_SetSpringDampingRatio(static_cast<const D &>(*this).Handle(), dampingRatio); }
    template <typename D, bool ForceConst> float BasicPrismaticJointInterface<D, ForceConst>::GetSpringDampingRatio() const { return b2PrismaticJoint_GetSpringDampingRatio(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicPrismaticJointInterface<D, ForceConst>::SetSpringHertz(float hertz) requires (!ForceConst) { b2PrismaticJoint_SetSpringHertz(static_cast<const D &>(*this).Handle(), hertz); }
    template <typename D, bool ForceConst> float BasicPrismaticJointInterface<D, ForceConst>::GetSpringHertz() const { return b2PrismaticJoint_GetSpringHertz(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> float BasicPrismaticJointInterface<D, ForceConst>::GetUpperLimit() const { return b2PrismaticJoint_GetUpperLimit(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> float BasicRevoluteJointInterface<D, ForceConst>::GetAngle() const { return b2RevoluteJoint_GetAngle(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicRevoluteJointInterface<D, ForceConst>::EnableLimit(bool enableLimit) requires (!ForceConst) { b2RevoluteJoint_EnableLimit(static_cast<const D &>(*this).Handle(), enableLimit); }
    template <typename D, bool ForceConst> bool BasicRevoluteJointInterface<D, ForceConst>::IsLimitEnabled() const { return b2RevoluteJoint_IsLimitEnabled(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicRevoluteJointInterface<D, ForceConst>::SetLimits(float lower, float upper) requires (!ForceConst) { b2RevoluteJoint_SetLimits(static_cast<const D &>(*this).Handle(), lower, upper); }
    template <typename D, bool ForceConst> float BasicRevoluteJointInterface<D, ForceConst>::GetLowerLimit() const { return b2RevoluteJoint_GetLowerLimit(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicRevoluteJointInterface<D, ForceConst>::SetMaxMotorTorque(float torque) requires (!ForceConst) { b2RevoluteJoint_SetMaxMotorTorque(static_cast<const D &>(*this).Handle(), torque); }
    template <typename D, bool ForceConst> float BasicRevoluteJointInterface<D, ForceConst>::GetMaxMotorTorque() const { return b2RevoluteJoint_GetMaxMotorTorque(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicRevoluteJointInterface<D, ForceConst>::EnableMotor(bool enableMotor) requires (!ForceConst) { b2RevoluteJoint_EnableMotor(static_cast<const D &>(*this).Handle(), enableMotor); }
    template <typename D, bool ForceConst> bool BasicRevoluteJointInterface<D, ForceConst>::IsMotorEnabled() const { return b2RevoluteJoint_IsMotorEnabled(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicRevoluteJointInterface<D, ForceConst>::SetMotorSpeed(float motorSpeed) requires (!ForceConst) { b2RevoluteJoint_SetMotorSpeed(static_cast<const D &>(*this).Handle(), motorSpeed); }
    template <typename D, bool ForceConst> float BasicRevoluteJointInterface<D, ForceConst>::GetMotorSpeed() const { return b2RevoluteJoint_GetMotorSpeed(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> float BasicRevoluteJointInterface<D, ForceConst>::GetMotorTorque() const { return b2RevoluteJoint_GetMotorTorque(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicRevoluteJointInterface<D, ForceConst>::EnableSpring(bool enableSpring) requires (!ForceConst) { b2RevoluteJoint_EnableSpring(static_cast<const D &>(*this).Handle(), enableSpring); }
    template <typename D, bool ForceConst> void BasicRevoluteJointInterface<D, ForceConst>::SetSpringDampingRatio(float dampingRatio) requires (!ForceConst) { b2RevoluteJoint_SetSpringDampingRatio(static_cast<const D &>(*this).Handle(), dampingRatio); }
    template <typename D, bool ForceConst> float BasicRevoluteJointInterface<D, ForceConst>::GetSpringDampingRatio() const { return b2RevoluteJoint_GetSpringDampingRatio(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicRevoluteJointInterface<D, ForceConst>::SetSpringHertz(float hertz) requires (!ForceConst) { b2RevoluteJoint_SetSpringHertz(static_cast<const D &>(*this).Handle(), hertz); }
    template <typename D, bool ForceConst> float BasicRevoluteJointInterface<D, ForceConst>::GetSpringHertz() const { return b2RevoluteJoint_GetSpringHertz(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> float BasicRevoluteJointInterface<D, ForceConst>::GetUpperLimit() const { return b2RevoluteJoint_GetUpperLimit(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicWeldJointInterface<D, ForceConst>::SetAngularDampingRatio(float dampingRatio) requires (!ForceConst) { b2WeldJoint_SetAngularDampingRatio(static_cast<const D &>(*this).Handle(), dampingRatio); }
    template <typename D, bool ForceConst> float BasicWeldJointInterface<D, ForceConst>::GetAngularDampingRatio() const { return b2WeldJoint_GetAngularDampingRatio(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicWeldJointInterface<D, ForceConst>::SetAngularHertz(float hertz) requires (!ForceConst) { b2WeldJoint_SetAngularHertz(static_cast<const D &>(*this).Handle(), hertz); }
    template <typename D, bool ForceConst> float BasicWeldJointInterface<D, ForceConst>::GetAngularHertz() const { return b2WeldJoint_GetAngularHertz(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicWeldJointInterface<D, ForceConst>::SetLinearDampingRatio(float dampingRatio) requires (!ForceConst) { b2WeldJoint_SetLinearDampingRatio(static_cast<const D &>(*this).Handle(), dampingRatio); }
    template <typename D, bool ForceConst> float BasicWeldJointInterface<D, ForceConst>::GetLinearDampingRatio() const { return b2WeldJoint_GetLinearDampingRatio(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicWeldJointInterface<D, ForceConst>::SetLinearHertz(float hertz) requires (!ForceConst) { b2WeldJoint_SetLinearHertz(static_cast<const D &>(*this).Handle(), hertz); }
    template <typename D, bool ForceConst> float BasicWeldJointInterface<D, ForceConst>::GetLinearHertz() const { return b2WeldJoint_GetLinearHertz(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicWheelJointInterface<D, ForceConst>::EnableLimit(bool enableLimit) requires (!ForceConst) { b2WheelJoint_EnableLimit(static_cast<const D &>(*this).Handle(), enableLimit); }
    template <typename D, bool ForceConst> bool BasicWheelJointInterface<D, ForceConst>::IsLimitEnabled() const { return b2WheelJoint_IsLimitEnabled(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicWheelJointInterface<D, ForceConst>::SetLimits(float lower, float upper) requires (!ForceConst) { b2WheelJoint_SetLimits(static_cast<const D &>(*this).Handle(), lower, upper); }
    template <typename D, bool ForceConst> float BasicWheelJointInterface<D, ForceConst>::GetLowerLimit() const { return b2WheelJoint_GetLowerLimit(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicWheelJointInterface<D, ForceConst>::SetMaxMotorTorque(float torque) requires (!ForceConst) { b2WheelJoint_SetMaxMotorTorque(static_cast<const D &>(*this).Handle(), torque); }
    template <typename D, bool ForceConst> float BasicWheelJointInterface<D, ForceConst>::GetMaxMotorTorque() const { return b2WheelJoint_GetMaxMotorTorque(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicWheelJointInterface<D, ForceConst>::EnableMotor(bool enableMotor) requires (!ForceConst) { b2WheelJoint_EnableMotor(static_cast<const D &>(*this).Handle(), enableMotor); }
    template <typename D, bool ForceConst> bool BasicWheelJointInterface<D, ForceConst>::IsMotorEnabled() const { return b2WheelJoint_IsMotorEnabled(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicWheelJointInterface<D, ForceConst>::SetMotorSpeed(float motorSpeed) requires (!ForceConst) { b2WheelJoint_SetMotorSpeed(static_cast<const D &>(*this).Handle(), motorSpeed); }
    template <typename D, bool ForceConst> float BasicWheelJointInterface<D, ForceConst>::GetMotorSpeed() const { return b2WheelJoint_GetMotorSpeed(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> float BasicWheelJointInterface<D, ForceConst>::GetMotorTorque() const { return b2WheelJoint_GetMotorTorque(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicWheelJointInterface<D, ForceConst>::EnableSpring(bool enableSpring) requires (!ForceConst) { b2WheelJoint_EnableSpring(static_cast<const D &>(*this).Handle(), enableSpring); }
    template <typename D, bool ForceConst> bool BasicWheelJointInterface<D, ForceConst>::IsSpringEnabled() const { return b2WheelJoint_IsSpringEnabled(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicWheelJointInterface<D, ForceConst>::SetSpringDampingRatio(float dampingRatio) requires (!ForceConst) { b2WheelJoint_SetSpringDampingRatio(static_cast<const D &>(*this).Handle(), dampingRatio); }
    template <typename D, bool ForceConst> float BasicWheelJointInterface<D, ForceConst>::GetSpringDampingRatio() const { return b2WheelJoint_GetSpringDampingRatio(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicWheelJointInterface<D, ForceConst>::SetSpringHertz(float hertz) requires (!ForceConst) { b2WheelJoint_SetSpringHertz(static_cast<const D &>(*this).Handle(), hertz); }
    template <typename D, bool ForceConst> float BasicWheelJointInterface<D, ForceConst>::GetSpringHertz() const { return b2WheelJoint_GetSpringHertz(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> float BasicWheelJointInterface<D, ForceConst>::GetUpperLimit() const { return b2WheelJoint_GetUpperLimit(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> Chain BasicBodyInterface<D, ForceConst>::CreateChain(Tags::OwningHandle, const std::derived_from<b2ChainDef> auto& def) requires (!ForceConst) { Chain ret; ret.id = b2CreateChain(static_cast<const D &>(*this).Handle(), &def); return ret; }
    template <typename D, bool ForceConst> ChainRef BasicBodyInterface<D, ForceConst>::CreateChain(Tags::DestroyWithParent, const std::derived_from<b2ChainDef> auto& def) requires (!ForceConst) { return b2CreateChain(static_cast<const D &>(*this).Handle(), &def); }
    template <typename D, bool ForceConst> Shape BasicBodyInterface<D, ForceConst>::CreateShape(Tags::OwningHandle, const std::derived_from<b2ShapeDef> auto& def, const b2Capsule& capsule) requires (!ForceConst) { Shape ret; ret.id = b2CreateCapsuleShape(static_cast<const D &>(*this).Handle(), &def, &capsule); return ret; }
    template <typename D, bool ForceConst> ShapeRef BasicBodyInterface<D, ForceConst>::CreateShape(Tags::DestroyWithParent, const std::derived_from<b2ShapeDef> auto& def, const b2Capsule& capsule) requires (!ForceConst) { return b2CreateCapsuleShape(static_cast<const D &>(*this).Handle(), &def, &capsule); }
    template <typename D, bool ForceConst> Shape BasicBodyInterface<D, ForceConst>::CreateShape(Tags::OwningHandle, const std::derived_from<b2ShapeDef> auto& def, const b2Circle& circle) requires (!ForceConst) { Shape ret; ret.id = b2CreateCircleShape(static_cast<const D &>(*this).Handle(), &def, &circle); return ret; }
    template <typename D, bool ForceConst> ShapeRef BasicBodyInterface<D, ForceConst>::CreateShape(Tags::DestroyWithParent, const std::derived_from<b2ShapeDef> auto& def, const b2Circle& circle) requires (!ForceConst) { return b2CreateCircleShape(static_cast<const D &>(*this).Handle(), &def, &circle); }
    template <typename D, bool ForceConst> Shape BasicBodyInterface<D, ForceConst>::CreateShape(Tags::OwningHandle, const std::derived_from<b2ShapeDef> auto& def, const b2Polygon& polygon) requires (!ForceConst) { Shape ret; ret.id = b2CreatePolygonShape(static_cast<const D &>(*this).Handle(), &def, &polygon); return ret; }
    template <typename D, bool ForceConst> ShapeRef BasicBodyInterface<D, ForceConst>::CreateShape(Tags::DestroyWithParent, const std::derived_from<b2ShapeDef> auto& def, const b2Polygon& polygon) requires (!ForceConst) { return b2CreatePolygonShape(static_cast<const D &>(*this).Handle(), &def, &polygon); }
    template <typename D, bool ForceConst> Shape BasicBodyInterface<D, ForceConst>::CreateShape(Tags::OwningHandle, const std::derived_from<b2ShapeDef> auto& def, const b2Segment& segment) requires (!ForceConst) { Shape ret; ret.id = b2CreateSegmentShape(static_cast<const D &>(*this).Handle(), &def, &segment); return ret; }
    template <typename D, bool ForceConst> ShapeRef BasicBodyInterface<D, ForceConst>::CreateShape(Tags::DestroyWithParent, const std::derived_from<b2ShapeDef> auto& def, const b2Segment& segment) requires (!ForceConst) { return b2CreateSegmentShape(static_cast<const D &>(*this).Handle(), &def, &segment); }
    template <typename D, bool ForceConst> void BasicBodyInterface<D, ForceConst>::Destroy() requires (!ForceConst) { if (*this) { b2DestroyBody(static_cast<const D &>(*this).Handle()); static_cast<D &>(*this).id = {}; } }
    template <typename D, bool ForceConst> bool BasicBodyInterface<D, ForceConst>::IsValid() const { return b2Body_IsValid(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicBodyInterface<D, ForceConst>::Enable() requires (!ForceConst) { b2Body_Enable(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> bool BasicBodyInterface<D, ForceConst>::IsEnabled() const { return b2Body_IsEnabled(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicBodyInterface<D, ForceConst>::SetAngularDamping(float angularDamping) requires (!ForceConst) { b2Body_SetAngularDamping(static_cast<const D &>(*this).Handle(), angularDamping); }
    template <typename D, bool ForceConst> float BasicBodyInterface<D, ForceConst>::GetAngularDamping() const { return b2Body_GetAngularDamping(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicBodyInterface<D, ForceConst>::SetAngularVelocity(float angularVelocity) requires (!ForceConst) { b2Body_SetAngularVelocity(static_cast<const D &>(*this).Handle(), angularVelocity); }
    template <typename D, bool ForceConst> float BasicBodyInterface<D, ForceConst>::GetAngularVelocity() const { return b2Body_GetAngularVelocity(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicBodyInterface<D, ForceConst>::ApplyAngularImpulse(float impulse, bool wake) requires (!ForceConst) { b2Body_ApplyAngularImpulse(static_cast<const D &>(*this).Handle(), impulse, wake); }
    template <typename D, bool ForceConst> void BasicBodyInterface<D, ForceConst>::ApplyForce(b2Vec2 force, b2Vec2 point, bool wake) requires (!ForceConst) { b2Body_ApplyForce(static_cast<const D &>(*this).Handle(), force, point, wake); }
    template <typename D, bool ForceConst> void BasicBodyInterface<D, ForceConst>::ApplyForceToCenter(b2Vec2 force, bool wake) requires (!ForceConst) { b2Body_ApplyForceToCenter(static_cast<const D &>(*this).Handle(), force, wake); }
    template <typename D, bool ForceConst> void BasicBodyInterface<D, ForceConst>::ApplyLinearImpulse(b2Vec2 impulse, b2Vec2 point, bool wake) requires (!ForceConst) { b2Body_ApplyLinearImpulse(static_cast<const D &>(*this).Handle(), impulse, point, wake); }
    template <typename D, bool ForceConst> void BasicBodyInterface<D, ForceConst>::ApplyLinearImpulseToCenter(b2Vec2 impulse, bool wake) requires (!ForceConst) { b2Body_ApplyLinearImpulseToCenter(static_cast<const D &>(*this).Handle(), impulse, wake); }
    template <typename D, bool ForceConst> void BasicBodyInterface<D, ForceConst>::ApplyMassFromShapes() requires (!ForceConst) { b2Body_ApplyMassFromShapes(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicBodyInterface<D, ForceConst>::ApplyTorque(float torque, bool wake) requires (!ForceConst) { b2Body_ApplyTorque(static_cast<const D &>(*this).Handle(), torque, wake); }
    template <typename D, bool ForceConst> void BasicBodyInterface<D, ForceConst>::SetAutomaticMass(bool automaticMass) requires (!ForceConst) { b2Body_SetAutomaticMass(static_cast<const D &>(*this).Handle(), automaticMass); }
    template <typename D, bool ForceConst> bool BasicBodyInterface<D, ForceConst>::GetAutomaticMass() const { return b2Body_GetAutomaticMass(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicBodyInterface<D, ForceConst>::SetAwake(bool awake) requires (!ForceConst) { b2Body_SetAwake(static_cast<const D &>(*this).Handle(), awake); }
    template <typename D, bool ForceConst> bool BasicBodyInterface<D, ForceConst>::IsAwake() const { return b2Body_IsAwake(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicBodyInterface<D, ForceConst>::SetBullet(bool flag) requires (!ForceConst) { b2Body_SetBullet(static_cast<const D &>(*this).Handle(), flag); }
    template <typename D, bool ForceConst> bool BasicBodyInterface<D, ForceConst>::IsBullet() const { return b2Body_IsBullet(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> b2AABB BasicBodyInterface<D, ForceConst>::ComputeAABB() const { return b2Body_ComputeAABB(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> int BasicBodyInterface<D, ForceConst>::GetContactCapacity() const { return b2Body_GetContactCapacity(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> int BasicBodyInterface<D, ForceConst>::GetContactData(b2ContactData& contactData, int capacity) const { return b2Body_GetContactData(static_cast<const D &>(*this).Handle(), &contactData, capacity); }
    template <typename D, bool ForceConst> void BasicBodyInterface<D, ForceConst>::Disable() requires (!ForceConst) { b2Body_Disable(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicBodyInterface<D, ForceConst>::SetFixedRotation(bool flag) requires (!ForceConst) { b2Body_SetFixedRotation(static_cast<const D &>(*this).Handle(), flag); }
    template <typename D, bool ForceConst> bool BasicBodyInterface<D, ForceConst>::IsFixedRotation() const { return b2Body_IsFixedRotation(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicBodyInterface<D, ForceConst>::SetGravityScale(float gravityScale) requires (!ForceConst) { b2Body_SetGravityScale(static_cast<const D &>(*this).Handle(), gravityScale); }
    template <typename D, bool ForceConst> float BasicBodyInterface<D, ForceConst>::GetGravityScale() const { return b2Body_GetGravityScale(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicBodyInterface<D, ForceConst>::EnableHitEvents(bool enableHitEvents) requires (!ForceConst) { b2Body_EnableHitEvents(static_cast<const D &>(*this).Handle(), enableHitEvents); }
    template <typename D, bool ForceConst> float BasicBodyInterface<D, ForceConst>::GetInertiaTensor() const { return b2Body_GetInertiaTensor(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> int BasicBodyInterface<D, ForceConst>::GetJointCount() const { return b2Body_GetJointCount(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> int BasicBodyInterface<D, ForceConst>::GetJoints(b2JointId& jointArray, int capacity) const { return b2Body_GetJoints(static_cast<const D &>(*this).Handle(), &jointArray, capacity); }
    template <typename D, bool ForceConst> void BasicBodyInterface<D, ForceConst>::SetLinearDamping(float linearDamping) requires (!ForceConst) { b2Body_SetLinearDamping(static_cast<const D &>(*this).Handle(), linearDamping); }
    template <typename D, bool ForceConst> float BasicBodyInterface<D, ForceConst>::GetLinearDamping() const { return b2Body_GetLinearDamping(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicBodyInterface<D, ForceConst>::SetLinearVelocity(b2Vec2 linearVelocity) requires (!ForceConst) { b2Body_SetLinearVelocity(static_cast<const D &>(*this).Handle(), linearVelocity); }
    template <typename D, bool ForceConst> b2Vec2 BasicBodyInterface<D, ForceConst>::GetLinearVelocity() const { return b2Body_GetLinearVelocity(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> b2Vec2 BasicBodyInterface<D, ForceConst>::GetLocalCenterOfMass() const { return b2Body_GetLocalCenterOfMass(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> b2Vec2 BasicBodyInterface<D, ForceConst>::GetLocalPoint(b2Vec2 worldPoint) const { return b2Body_GetLocalPoint(static_cast<const D &>(*this).Handle(), worldPoint); }
    template <typename D, bool ForceConst> b2Vec2 BasicBodyInterface<D, ForceConst>::GetLocalVector(b2Vec2 worldVector) const { return b2Body_GetLocalVector(static_cast<const D &>(*this).Handle(), worldVector); }
    template <typename D, bool ForceConst> float BasicBodyInterface<D, ForceConst>::GetMass() const { return b2Body_GetMass(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicBodyInterface<D, ForceConst>::SetMassData(b2MassData massData) requires (!ForceConst) { b2Body_SetMassData(static_cast<const D &>(*this).Handle(), massData); }
    template <typename D, bool ForceConst> b2MassData BasicBodyInterface<D, ForceConst>::GetMassData() const { return b2Body_GetMassData(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> b2Vec2 BasicBodyInterface<D, ForceConst>::GetPosition() const { return b2Body_GetPosition(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> b2Rot BasicBodyInterface<D, ForceConst>::GetRotation() const { return b2Body_GetRotation(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> int BasicBodyInterface<D, ForceConst>::GetShapeCount() const { return b2Body_GetShapeCount(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> int BasicBodyInterface<D, ForceConst>::GetShapes(b2ShapeId& shapeArray, int capacity) const { return b2Body_GetShapes(static_cast<const D &>(*this).Handle(), &shapeArray, capacity); }
    template <typename D, bool ForceConst> void BasicBodyInterface<D, ForceConst>::EnableSleep(bool enableSleep) requires (!ForceConst) { b2Body_EnableSleep(static_cast<const D &>(*this).Handle(), enableSleep); }
    template <typename D, bool ForceConst> bool BasicBodyInterface<D, ForceConst>::IsSleepEnabled() const { return b2Body_IsSleepEnabled(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicBodyInterface<D, ForceConst>::SetSleepThreshold(float sleepVelocity) requires (!ForceConst) { b2Body_SetSleepThreshold(static_cast<const D &>(*this).Handle(), sleepVelocity); }
    template <typename D, bool ForceConst> float BasicBodyInterface<D, ForceConst>::GetSleepThreshold() const { return b2Body_GetSleepThreshold(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicBodyInterface<D, ForceConst>::SetTransform(b2Vec2 position, b2Rot rotation) requires (!ForceConst) { b2Body_SetTransform(static_cast<const D &>(*this).Handle(), position, rotation); }
    template <typename D, bool ForceConst> b2Transform BasicBodyInterface<D, ForceConst>::GetTransform() const { return b2Body_GetTransform(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicBodyInterface<D, ForceConst>::SetType(b2BodyType type) requires (!ForceConst) { b2Body_SetType(static_cast<const D &>(*this).Handle(), type); }
    template <typename D, bool ForceConst> b2BodyType BasicBodyInterface<D, ForceConst>::GetType() const { return b2Body_GetType(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicBodyInterface<D, ForceConst>::SetUserData(void* userData) requires (!ForceConst) { b2Body_SetUserData(static_cast<const D &>(*this).Handle(), userData); }
    template <typename D, bool ForceConst> void* BasicBodyInterface<D, ForceConst>::GetUserData() const { return b2Body_GetUserData(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> b2Vec2 BasicBodyInterface<D, ForceConst>::GetWorldCenterOfMass() const { return b2Body_GetWorldCenterOfMass(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> b2Vec2 BasicBodyInterface<D, ForceConst>::GetWorldPoint(b2Vec2 localPoint) const { return b2Body_GetWorldPoint(static_cast<const D &>(*this).Handle(), localPoint); }
    template <typename D, bool ForceConst> b2Vec2 BasicBodyInterface<D, ForceConst>::GetWorldVector(b2Vec2 localVector) const { return b2Body_GetWorldVector(static_cast<const D &>(*this).Handle(), localVector); }
    template <typename D, bool ForceConst> Body BasicWorldInterface<D, ForceConst>::CreateBody(Tags::OwningHandle, const std::derived_from<b2BodyDef> auto& def) requires (!ForceConst) { Body ret; ret.id = b2CreateBody(static_cast<const D &>(*this).Handle(), &def); return ret; }
    template <typename D, bool ForceConst> BodyRef BasicWorldInterface<D, ForceConst>::CreateBody(Tags::DestroyWithParent, const std::derived_from<b2BodyDef> auto& def) requires (!ForceConst) { return b2CreateBody(static_cast<const D &>(*this).Handle(), &def); }
    template <typename D, bool ForceConst> DistanceJoint BasicWorldInterface<D, ForceConst>::CreateJoint(Tags::OwningHandle, const std::derived_from<b2DistanceJointDef> auto& def) requires (!ForceConst) { DistanceJoint ret; ret.id = b2CreateDistanceJoint(static_cast<const D &>(*this).Handle(), &def); return ret; }
    template <typename D, bool ForceConst> DistanceJointRef BasicWorldInterface<D, ForceConst>::CreateJoint(Tags::DestroyWithParent, const std::derived_from<b2DistanceJointDef> auto& def) requires (!ForceConst) { return (DistanceJointRef)b2CreateDistanceJoint(static_cast<const D &>(*this).Handle(), &def); }
    template <typename D, bool ForceConst> MotorJoint BasicWorldInterface<D, ForceConst>::CreateJoint(Tags::OwningHandle, const std::derived_from<b2MotorJointDef> auto& def) requires (!ForceConst) { MotorJoint ret; ret.id = b2CreateMotorJoint(static_cast<const D &>(*this).Handle(), &def); return ret; }
    template <typename D, bool ForceConst> MotorJointRef BasicWorldInterface<D, ForceConst>::CreateJoint(Tags::DestroyWithParent, const std::derived_from<b2MotorJointDef> auto& def) requires (!ForceConst) { return (MotorJointRef)b2CreateMotorJoint(static_cast<const D &>(*this).Handle(), &def); }
    template <typename D, bool ForceConst> MouseJoint BasicWorldInterface<D, ForceConst>::CreateJoint(Tags::OwningHandle, const std::derived_from<b2MouseJointDef> auto& def) requires (!ForceConst) { MouseJoint ret; ret.id = b2CreateMouseJoint(static_cast<const D &>(*this).Handle(), &def); return ret; }
    template <typename D, bool ForceConst> MouseJointRef BasicWorldInterface<D, ForceConst>::CreateJoint(Tags::DestroyWithParent, const std::derived_from<b2MouseJointDef> auto& def) requires (!ForceConst) { return (MouseJointRef)b2CreateMouseJoint(static_cast<const D &>(*this).Handle(), &def); }
    template <typename D, bool ForceConst> PrismaticJoint BasicWorldInterface<D, ForceConst>::CreateJoint(Tags::OwningHandle, const std::derived_from<b2PrismaticJointDef> auto& def) requires (!ForceConst) { PrismaticJoint ret; ret.id = b2CreatePrismaticJoint(static_cast<const D &>(*this).Handle(), &def); return ret; }
    template <typename D, bool ForceConst> PrismaticJointRef BasicWorldInterface<D, ForceConst>::CreateJoint(Tags::DestroyWithParent, const std::derived_from<b2PrismaticJointDef> auto& def) requires (!ForceConst) { return (PrismaticJointRef)b2CreatePrismaticJoint(static_cast<const D &>(*this).Handle(), &def); }
    template <typename D, bool ForceConst> RevoluteJoint BasicWorldInterface<D, ForceConst>::CreateJoint(Tags::OwningHandle, const std::derived_from<b2RevoluteJointDef> auto& def) requires (!ForceConst) { RevoluteJoint ret; ret.id = b2CreateRevoluteJoint(static_cast<const D &>(*this).Handle(), &def); return ret; }
    template <typename D, bool ForceConst> RevoluteJointRef BasicWorldInterface<D, ForceConst>::CreateJoint(Tags::DestroyWithParent, const std::derived_from<b2RevoluteJointDef> auto& def) requires (!ForceConst) { return (RevoluteJointRef)b2CreateRevoluteJoint(static_cast<const D &>(*this).Handle(), &def); }
    template <typename D, bool ForceConst> WeldJoint BasicWorldInterface<D, ForceConst>::CreateJoint(Tags::OwningHandle, const std::derived_from<b2WeldJointDef> auto& def) requires (!ForceConst) { WeldJoint ret; ret.id = b2CreateWeldJoint(static_cast<const D &>(*this).Handle(), &def); return ret; }
    template <typename D, bool ForceConst> WeldJointRef BasicWorldInterface<D, ForceConst>::CreateJoint(Tags::DestroyWithParent, const std::derived_from<b2WeldJointDef> auto& def) requires (!ForceConst) { return (WeldJointRef)b2CreateWeldJoint(static_cast<const D &>(*this).Handle(), &def); }
    template <typename D, bool ForceConst> WheelJoint BasicWorldInterface<D, ForceConst>::CreateJoint(Tags::OwningHandle, const std::derived_from<b2WheelJointDef> auto& def) requires (!ForceConst) { WheelJoint ret; ret.id = b2CreateWheelJoint(static_cast<const D &>(*this).Handle(), &def); return ret; }
    template <typename D, bool ForceConst> WheelJointRef BasicWorldInterface<D, ForceConst>::CreateJoint(Tags::DestroyWithParent, const std::derived_from<b2WheelJointDef> auto& def) requires (!ForceConst) { return (WheelJointRef)b2CreateWheelJoint(static_cast<const D &>(*this).Handle(), &def); }
    template <typename D, bool ForceConst> void BasicWorldInterface<D, ForceConst>::Destroy() requires (!ForceConst) { if (*this) { b2DestroyWorld(static_cast<const D &>(*this).Handle()); static_cast<D &>(*this).id = {}; } }
    template <typename D, bool ForceConst> bool BasicWorldInterface<D, ForceConst>::IsValid() const { return b2World_IsValid(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> b2BodyEvents BasicWorldInterface<D, ForceConst>::GetBodyEvents() const { return b2World_GetBodyEvents(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicWorldInterface<D, ForceConst>::Cast(const b2Capsule& capsule, b2Transform originTransform, b2Vec2 translation, b2QueryFilter filter, detail::FuncRef<b2CastResultFcn,false> fcn) requires (!ForceConst) { b2World_CastCapsule(static_cast<const D &>(*this).Handle(), &capsule, originTransform, translation, filter, fcn.GetFunc(), fcn.GetContext()); }
    template <typename D, bool ForceConst> void BasicWorldInterface<D, ForceConst>::Cast(const b2Capsule& capsule, b2Transform originTransform, b2Vec2 translation, b2QueryFilter filter, detail::FuncRef<b2CastResultFcn,true> fcn) const { b2World_CastCapsule(static_cast<const D &>(*this).Handle(), &capsule, originTransform, translation, filter, fcn.GetFunc(), fcn.GetContext()); }
    template <typename D, bool ForceConst> void BasicWorldInterface<D, ForceConst>::Cast(const b2Circle& circle, b2Transform originTransform, b2Vec2 translation, b2QueryFilter filter, detail::FuncRef<b2CastResultFcn,false> fcn) requires (!ForceConst) { b2World_CastCircle(static_cast<const D &>(*this).Handle(), &circle, originTransform, translation, filter, fcn.GetFunc(), fcn.GetContext()); }
    template <typename D, bool ForceConst> void BasicWorldInterface<D, ForceConst>::Cast(const b2Circle& circle, b2Transform originTransform, b2Vec2 translation, b2QueryFilter filter, detail::FuncRef<b2CastResultFcn,true> fcn) const { b2World_CastCircle(static_cast<const D &>(*this).Handle(), &circle, originTransform, translation, filter, fcn.GetFunc(), fcn.GetContext()); }
    template <typename D, bool ForceConst> void BasicWorldInterface<D, ForceConst>::Cast(const b2Polygon& polygon, b2Transform originTransform, b2Vec2 translation, b2QueryFilter filter, detail::FuncRef<b2CastResultFcn,false> fcn) requires (!ForceConst) { b2World_CastPolygon(static_cast<const D &>(*this).Handle(), &polygon, originTransform, translation, filter, fcn.GetFunc(), fcn.GetContext()); }
    template <typename D, bool ForceConst> void BasicWorldInterface<D, ForceConst>::Cast(const b2Polygon& polygon, b2Transform originTransform, b2Vec2 translation, b2QueryFilter filter, detail::FuncRef<b2CastResultFcn,true> fcn) const { b2World_CastPolygon(static_cast<const D &>(*this).Handle(), &polygon, originTransform, translation, filter, fcn.GetFunc(), fcn.GetContext()); }
    template <typename D, bool ForceConst> void BasicWorldInterface<D, ForceConst>::CastRay(b2Vec2 origin, b2Vec2 translation, b2QueryFilter filter, detail::FuncRef<b2CastResultFcn,false> fcn) requires (!ForceConst) { b2World_CastRay(static_cast<const D &>(*this).Handle(), origin, translation, filter, fcn.GetFunc(), fcn.GetContext()); }
    template <typename D, bool ForceConst> void BasicWorldInterface<D, ForceConst>::CastRay(b2Vec2 origin, b2Vec2 translation, b2QueryFilter filter, detail::FuncRef<b2CastResultFcn,true> fcn) const { b2World_CastRay(static_cast<const D &>(*this).Handle(), origin, translation, filter, fcn.GetFunc(), fcn.GetContext()); }
    template <typename D, bool ForceConst> b2RayResult BasicWorldInterface<D, ForceConst>::CastRayClosest(b2Vec2 origin, b2Vec2 translation, b2QueryFilter filter) const { return b2World_CastRayClosest(static_cast<const D &>(*this).Handle(), origin, translation, filter); }
    template <typename D, bool ForceConst> b2ContactEvents BasicWorldInterface<D, ForceConst>::GetContactEvents() const { return b2World_GetContactEvents(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicWorldInterface<D, ForceConst>::SetContactTuning(float hertz, float dampingRatio, float pushVelocity) requires (!ForceConst) { b2World_SetContactTuning(static_cast<const D &>(*this).Handle(), hertz, dampingRatio, pushVelocity); }
    template <typename D, bool ForceConst> void BasicWorldInterface<D, ForceConst>::EnableContinuous(bool flag) requires (!ForceConst) { b2World_EnableContinuous(static_cast<const D &>(*this).Handle(), flag); }
    template <typename D, bool ForceConst> b2Counters BasicWorldInterface<D, ForceConst>::GetCounters() const { return b2World_GetCounters(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicWorldInterface<D, ForceConst>::SetCustomFilterCallback(b2CustomFilterFcn* fcn, void* context) requires (!ForceConst) { b2World_SetCustomFilterCallback(static_cast<const D &>(*this).Handle(), fcn, context); }
    template <typename D, bool ForceConst> void BasicWorldInterface<D, ForceConst>::Draw(b2DebugDraw& draw) const { b2World_Draw(static_cast<const D &>(*this).Handle(), &draw); }
    template <typename D, bool ForceConst> void BasicWorldInterface<D, ForceConst>::DumpMemoryStats() requires (!ForceConst) { b2World_DumpMemoryStats(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicWorldInterface<D, ForceConst>::Explode(b2Vec2 position, float radius, float impulse) requires (!ForceConst) { b2World_Explode(static_cast<const D &>(*this).Handle(), position, radius, impulse); }
    template <typename D, bool ForceConst> void BasicWorldInterface<D, ForceConst>::SetGravity(b2Vec2 gravity) requires (!ForceConst) { b2World_SetGravity(static_cast<const D &>(*this).Handle(), gravity); }
    template <typename D, bool ForceConst> b2Vec2 BasicWorldInterface<D, ForceConst>::GetGravity() const { return b2World_GetGravity(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicWorldInterface<D, ForceConst>::SetHitEventThreshold(float value) requires (!ForceConst) { b2World_SetHitEventThreshold(static_cast<const D &>(*this).Handle(), value); }
    template <typename D, bool ForceConst> void BasicWorldInterface<D, ForceConst>::Overlap(b2AABB aabb, b2QueryFilter filter, detail::FuncRef<b2OverlapResultFcn,false> fcn) requires (!ForceConst) { b2World_OverlapAABB(static_cast<const D &>(*this).Handle(), aabb, filter, fcn.GetFunc(), fcn.GetContext()); }
    template <typename D, bool ForceConst> void BasicWorldInterface<D, ForceConst>::Overlap(b2AABB aabb, b2QueryFilter filter, detail::FuncRef<b2OverlapResultFcn,true> fcn) const { b2World_OverlapAABB(static_cast<const D &>(*this).Handle(), aabb, filter, fcn.GetFunc(), fcn.GetContext()); }
    template <typename D, bool ForceConst> void BasicWorldInterface<D, ForceConst>::Overlap(const b2Capsule& capsule, b2Transform transform, b2QueryFilter filter, detail::FuncRef<b2OverlapResultFcn,false> fcn) requires (!ForceConst) { b2World_OverlapCapsule(static_cast<const D &>(*this).Handle(), &capsule, transform, filter, fcn.GetFunc(), fcn.GetContext()); }
    template <typename D, bool ForceConst> void BasicWorldInterface<D, ForceConst>::Overlap(const b2Capsule& capsule, b2Transform transform, b2QueryFilter filter, detail::FuncRef<b2OverlapResultFcn,true> fcn) const { b2World_OverlapCapsule(static_cast<const D &>(*this).Handle(), &capsule, transform, filter, fcn.GetFunc(), fcn.GetContext()); }
    template <typename D, bool ForceConst> void BasicWorldInterface<D, ForceConst>::Overlap(const b2Circle& circle, b2Transform transform, b2QueryFilter filter, detail::FuncRef<b2OverlapResultFcn,false> fcn) requires (!ForceConst) { b2World_OverlapCircle(static_cast<const D &>(*this).Handle(), &circle, transform, filter, fcn.GetFunc(), fcn.GetContext()); }
    template <typename D, bool ForceConst> void BasicWorldInterface<D, ForceConst>::Overlap(const b2Circle& circle, b2Transform transform, b2QueryFilter filter, detail::FuncRef<b2OverlapResultFcn,true> fcn) const { b2World_OverlapCircle(static_cast<const D &>(*this).Handle(), &circle, transform, filter, fcn.GetFunc(), fcn.GetContext()); }
    template <typename D, bool ForceConst> void BasicWorldInterface<D, ForceConst>::Overlap(const b2Polygon& polygon, b2Transform transform, b2QueryFilter filter, detail::FuncRef<b2OverlapResultFcn,false> fcn) requires (!ForceConst) { b2World_OverlapPolygon(static_cast<const D &>(*this).Handle(), &polygon, transform, filter, fcn.GetFunc(), fcn.GetContext()); }
    template <typename D, bool ForceConst> void BasicWorldInterface<D, ForceConst>::Overlap(const b2Polygon& polygon, b2Transform transform, b2QueryFilter filter, detail::FuncRef<b2OverlapResultFcn,true> fcn) const { b2World_OverlapPolygon(static_cast<const D &>(*this).Handle(), &polygon, transform, filter, fcn.GetFunc(), fcn.GetContext()); }
    template <typename D, bool ForceConst> void BasicWorldInterface<D, ForceConst>::SetPreSolveCallback(b2PreSolveFcn* fcn, void* context) requires (!ForceConst) { b2World_SetPreSolveCallback(static_cast<const D &>(*this).Handle(), fcn, context); }
    template <typename D, bool ForceConst> b2Profile BasicWorldInterface<D, ForceConst>::GetProfile() const { return b2World_GetProfile(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicWorldInterface<D, ForceConst>::SetRestitutionThreshold(float value) requires (!ForceConst) { b2World_SetRestitutionThreshold(static_cast<const D &>(*this).Handle(), value); }
    template <typename D, bool ForceConst> b2SensorEvents BasicWorldInterface<D, ForceConst>::GetSensorEvents() const { return b2World_GetSensorEvents(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicWorldInterface<D, ForceConst>::EnableSleeping(bool flag) requires (!ForceConst) { b2World_EnableSleeping(static_cast<const D &>(*this).Handle(), flag); }
    template <typename D, bool ForceConst> void BasicWorldInterface<D, ForceConst>::Step(float timeStep, int subStepCount) requires (!ForceConst) { b2World_Step(static_cast<const D &>(*this).Handle(), timeStep, subStepCount); }
    template <typename D, bool ForceConst> void BasicWorldInterface<D, ForceConst>::EnableWarmStarting(bool flag) requires (!ForceConst) { b2World_EnableWarmStarting(static_cast<const D &>(*this).Handle(), flag); }
    inline b2AABB DynamicTree::GetAABB(int32_t proxyId) const { return b2DynamicTree_GetAABB(&value, proxyId); }
    inline float DynamicTree::GetAreaRatio() const { return b2DynamicTree_GetAreaRatio(&value); }
    inline int DynamicTree::GetByteCount() const { return b2DynamicTree_GetByteCount(&value); }
    inline int32_t DynamicTree::CreateProxy(b2AABB aabb, uint32_t categoryBits, int32_t userData) { return b2DynamicTree_CreateProxy(&value, aabb, categoryBits, userData); }
    inline void DynamicTree::DestroyProxy(int32_t proxyId) { b2DynamicTree_DestroyProxy(&value, proxyId); }
    inline void DynamicTree::EnlargeProxy(int32_t proxyId, b2AABB aabb) { b2DynamicTree_EnlargeProxy(&value, proxyId, aabb); }
    inline int DynamicTree::GetHeight() const { return b2DynamicTree_GetHeight(&value); }
    inline int DynamicTree::GetMaxBalance() const { return b2DynamicTree_GetMaxBalance(&value); }
    inline void DynamicTree::MoveProxy(int32_t proxyId, b2AABB aabb) { b2DynamicTree_MoveProxy(&value, proxyId, aabb); }
    inline int DynamicTree::GetProxyCount() const { return b2DynamicTree_GetProxyCount(&value); }
    inline void DynamicTree::Query(b2AABB aabb, uint32_t maskBits, b2TreeQueryCallbackFcn* callback, void* context) const { b2DynamicTree_Query(&value, aabb, maskBits, callback, context); }
    inline void DynamicTree::RayCast(const b2RayCastInput& input, uint32_t maskBits, b2TreeRayCastCallbackFcn* callback, void* context) const { b2DynamicTree_RayCast(&value, &input, maskBits, callback, context); }
    inline int DynamicTree::Rebuild(bool fullBuild) { return b2DynamicTree_Rebuild(&value, fullBuild); }
    inline void DynamicTree::RebuildBottomUp() { b2DynamicTree_RebuildBottomUp(&value); }
    inline void DynamicTree::ShapeCast(const b2ShapeCastInput& input, uint32_t maskBits, b2TreeShapeCastCallbackFcn* callback, void* context) const { b2DynamicTree_ShapeCast(&value, &input, maskBits, callback, context); }
    inline void DynamicTree::ShiftOrigin(b2Vec2 newOrigin) { b2DynamicTree_ShiftOrigin(&value, newOrigin); }
    inline int32_t DynamicTree::GetUserData(int32_t proxyId) const { return b2DynamicTree_GetUserData(&value, proxyId); }
    inline void DynamicTree::Validate() const { b2DynamicTree_Validate(&value); }
} // namespace box2d
