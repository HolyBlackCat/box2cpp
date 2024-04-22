#pragma once

#include <box2d/box2d.h>
#include <box2d/dynamic_tree.h>

#include <cstddef>
#include <concepts>
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
    template <bool IsConstRef> class MaybeConstWheelJointRef;
    using WheelJointRef = MaybeConstWheelJointRef<false>;
    using WheelJointConstRef = MaybeConstWheelJointRef<true>;
    template <bool IsConstRef> class MaybeConstWeldJointRef;
    using WeldJointRef = MaybeConstWeldJointRef<false>;
    using WeldJointConstRef = MaybeConstWeldJointRef<true>;

    template <typename D, bool ForceConst>
    class BasicChainInterface
    {
      protected:
        BasicChainInterface() = default;

      public:
        [[nodiscard]] explicit operator bool() const { return B2_IS_NON_NULL(Handle()); }
        [[nodiscard]] const b2ChainId &Handle() const { return static_cast<const D &>(*this).id; }
        // Convert to a handle. Using a `same_as` template to prevent implicit madness. In particular, to prevent const-to-non-const conversions between non-owning wrappers.
        template <std::same_as<b2ChainId> T> [[nodiscard]] operator const T &() const { return Handle(); }

        /// Destroy a chain shape
        void Destroy() /*non-const*/ requires (!ForceConst);

        /// Set the restitution (bounciness) on a chain. Normally this is specified in b2ChainDef.
        void SetRestitution(float restitution) /*non-const*/ requires (!ForceConst);

        /// Chain identifier validation. Provides validation for up to 64K allocations.
        [[nodiscard]] bool IsValid() const;

        /// Set the friction of a chain. Normally this is set in b2ChainDef.
        void SetFriction(float friction) /*non-const*/ requires (!ForceConst);
    };

    /// Used to create a chain of edges. This is designed to eliminate ghost collisions with some limitations.
    ///	- DO NOT use chain shapes unless you understand the limitations. This is an advanced feature!
    ///	- chains are one-sided
    ///	- chains have no mass and should be used on static bodies
    ///	- the front side of the chain points the right of the point sequence
    ///	- chains are either a loop or open
    /// - a chain must have at least 4 points
    ///	- the distance between any two points must be greater than b2_linearSlop
    ///	- a chain shape should not self intersect (this is not validated)
    ///	- an open chain shape has NO COLLISION on the first and final edge
    ///	- you may overlap two open chains on their first three and/or last three points to get smooth collision
    ///	- a chain shape creates multiple hidden shapes on the body
    /// https://en.wikipedia.org/wiki/Polygonal_chain
    class Chain : public BasicChainInterface<Chain, false>
    {
        template <typename, bool>
        friend class BasicChainInterface;
        template <typename, bool>
        friend class BasicBodyInterface;


      protected:
        b2ChainId id = b2_nullChainId;

      public:
        static constexpr bool IsOwning = true;

        // Constructs a null (invalid) object.
        constexpr Chain() noexcept {}

        // The constructor accepts either this or directly `b2ChainDef`.
        struct Params : b2ChainDef
        {
            Params() : b2ChainDef(b2DefaultChainDef()) {}
        };

        Chain(Chain&& other) noexcept { id = std::exchange(other.id, b2_nullChainId); }
        Chain& operator=(Chain other) noexcept { std::swap(id, other.id); return *this; }

        ~Chain() { if (*this) Destroy(); }
    };

    template <bool IsConstRef>
    class MaybeConstChainRef : public BasicChainInterface<ChainRef, IsConstRef>
    {
        template <typename, bool>
        friend class BasicChainInterface;

      protected:
        b2ChainId id = b2_nullChainId;

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

        // Const a non-const reference to a const reference.
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
        template <std::same_as<b2ShapeId> T> [[nodiscard]] operator const T &() const { return Handle(); }

        /// Destroy any shape type
        void Destroy() /*non-const*/ requires (!ForceConst);

        /// Get the user data for a shape. This is useful when you get a shape id
        ///	from an event or query.
        [[nodiscard]] void* GetUserData() const;

        /// Ray cast a shape directly
        [[nodiscard]] b2CastOutput RayCast(b2Vec2 origin, b2Vec2 translation) const;

        /// Access the line segment geometry of a shape. Asserts the type is correct.
        [[nodiscard]] b2Segment GetSegment() const;

        /// @return are contact events enabled?
        [[nodiscard]] bool AreContactEventsEnabled() const;

        /// Allows you to change a shape to be a capsule or update the current capsule.
        void SetCapsule(const b2Capsule& capsule) /*non-const*/ requires (!ForceConst);

        /// @return are sensor events enabled?
        [[nodiscard]] bool AreSensorEventsEnabled() const;

        /// Set the current filter. This is almost as expensive as recreating the shape.
        void SetFilter(b2Filter filter) /*non-const*/ requires (!ForceConst);

        /// Access the circle geometry of a shape. Asserts the type is correct.
        [[nodiscard]] b2Circle GetCircle() const;

        /// Get the maximum capacity required for retrieving all the touching contacts on a shape
        [[nodiscard]] int GetContactCapacity() const;

        /// Set the density on a shape. Normally this is specified in b2ShapeDef.
        ///	This will not update the mass properties on the parent body until you
        /// call b2Body_ResetMassData.
        void SetDensity(float density) /*non-const*/ requires (!ForceConst);

        /// Access the capsule geometry of a shape. Asserts the type is correct.
        [[nodiscard]] b2Capsule GetCapsule() const;

        /// Get the touching contact data for a shape. The provided shapeId will be either shapeIdA or shapeIdB on the contact data.
        [[nodiscard]] int GetContactData(b2ContactData& contactData, int capacity) const;

        /// Get the current world AABB
        [[nodiscard]] b2AABB GetAABB() const;

        /// Get the density on a shape.
        [[nodiscard]] float GetDensity() const;

        /// Allows you to change a shape to be a segment or update the current segment.
        void SetPolygon(const b2Polygon& polygon) /*non-const*/ requires (!ForceConst);

        /// Enable contact events for this shape. Only applies to kinematic and dynamic bodies. Ignored for sensors.
        void EnableContactEvents(bool flag) /*non-const*/ requires (!ForceConst);

        /// Get the current filter
        [[nodiscard]] b2Filter GetFilter() const;

        /// Enable pre-solve contact events for this shape. Only applies to dynamic bodies. These are expensive
        ///	and must be carefully handled due to multi-threading. Ignored for sensors.
        void EnablePreSolveEvents(bool flag) /*non-const*/ requires (!ForceConst);

        /// Get the restitution on a shape.
        [[nodiscard]] float GetRestitution() const;

        /// Access the convex polygon geometry of a shape. Asserts the type is correct.
        [[nodiscard]] b2Polygon GetPolygon() const;

        /// Test a point for overlap with a shape
        [[nodiscard]] bool TestPoint(b2Vec2 point) const;

        /// Is this shape a sensor? See b2ShapeDef.
        [[nodiscard]] bool IsSensor() const;

        /// Access the smooth line segment geometry of a shape. These come from chain shapes.
        /// Asserts the type is correct.
        [[nodiscard]] b2SmoothSegment GetSmoothSegment() const;

        /// @return are pre-solve events enabled?
        [[nodiscard]] bool ArePreSolveEventsEnabled() const;

        /// Get the body that a shape is attached to
        [[nodiscard]] BodyRef GetBody() const;

        /// Set the friction on a shape. Normally this is specified in b2ShapeDef.
        void SetFriction(float friction) /*non-const*/ requires (!ForceConst);

        /// Allows you to change a shape to be a circle or update the current circle.
        /// This does not modify the mass properties.
        void SetCircle(const b2Circle& circle) /*non-const*/ requires (!ForceConst);

        /// If the type is b2_smoothSegmentShape then you can get the parent chain id.
        /// If the shape is not a smooth segment then this will return b2_nullChainId.
        [[nodiscard]] ChainRef GetParentChain() const;

        /// Set the restitution (bounciness) on a shape. Normally this is specified in b2ShapeDef.
        void SetRestitution(float restitution) /*non-const*/ requires (!ForceConst);

        /// Set the user data for a shape.
        void SetUserData(void* userData) /*non-const*/ requires (!ForceConst);

        /// Get the type of a shape.
        [[nodiscard]] b2ShapeType GetType() const;

        /// Get the friction on a shape.
        [[nodiscard]] float GetFriction() const;

        /// Allows you to change a shape to be a segment or update the current segment.
        void SetSegment(const b2Segment& segment) /*non-const*/ requires (!ForceConst);

        /// Shape identifier validation. Provides validation for up to 64K allocations.
        [[nodiscard]] bool IsValid() const;

        /// Enable sensor events for this shape. Only applies to kinematic and dynamic bodies. Ignored for sensors.
        void EnableSensorEvents(bool flag) /*non-const*/ requires (!ForceConst);
    };

    /// Used to create a shape
    class Shape : public BasicShapeInterface<Shape, false>
    {
        template <typename, bool>
        friend class BasicShapeInterface;
        template <typename, bool>
        friend class BasicBodyInterface;


      protected:
        b2ShapeId id = b2_nullShapeId;

      public:
        static constexpr bool IsOwning = true;

        // Constructs a null (invalid) object.
        constexpr Shape() noexcept {}

        // The constructor accepts either this or directly `b2ShapeDef`.
        struct Params : b2ShapeDef
        {
            Params() : b2ShapeDef(b2DefaultShapeDef()) {}
        };

        Shape(Shape&& other) noexcept { id = std::exchange(other.id, b2_nullShapeId); }
        Shape& operator=(Shape other) noexcept { std::swap(id, other.id); return *this; }

        ~Shape() { if (*this) Destroy(); }
    };

    template <bool IsConstRef>
    class MaybeConstShapeRef : public BasicShapeInterface<ShapeRef, IsConstRef>
    {
        template <typename, bool>
        friend class BasicShapeInterface;

      protected:
        b2ShapeId id = b2_nullShapeId;

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

        // Const a non-const reference to a const reference.
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
        template <std::same_as<b2JointId> T> [[nodiscard]] operator const T &() const { return Handle(); }

        /// Destroy any joint type
        void Destroy() /*non-const*/ requires (!ForceConst);

        /// Get local anchor on bodyA
        [[nodiscard]] b2Vec2 GetLocalAnchorA() const;

        /// Get local anchor on bodyB
        [[nodiscard]] b2Vec2 GetLocalAnchorB() const;

        /// Wake the bodies connect to this joint
        void WakeBodies() /*non-const*/ requires (!ForceConst);

        /// Set the user data on a joint
        void SetUserData(void* userData) /*non-const*/ requires (!ForceConst);

        /// Get the user data on a joint
        [[nodiscard]] void* GetUserData() const;

        /// Get body A on a joint
        [[nodiscard]] BodyRef GetBodyA() const;

        /// Get body B on a joint
        [[nodiscard]] BodyRef GetBodyB() const;

        /// Toggle collision between connected bodies
        void SetCollideConnected(bool shouldCollide) /*non-const*/ requires (!ForceConst);

        /// Is collision allowed between connected bodies?
        [[nodiscard]] bool GetCollideConnected() const;

        /// Joint identifier validation. Provides validation for up to 64K allocations.
        [[nodiscard]] bool IsValid() const;

        /// Get the joint type
        [[nodiscard]] b2JointType GetType() const;
    };

    class Joint : public BasicJointInterface<Joint, false>
    {
        template <typename, bool>
        friend class BasicJointInterface;

      protected:
        b2JointId id = b2_nullJointId;

      public:
        static constexpr bool IsOwning = true;

        // Constructs a null (invalid) object.
        constexpr Joint() noexcept {}

        Joint(Joint&& other) noexcept { id = std::exchange(other.id, b2_nullJointId); }
        Joint& operator=(Joint other) noexcept { std::swap(id, other.id); return *this; }

        ~Joint() { if (*this) Destroy(); }
    };

    template <bool IsConstRef>
    class MaybeConstJointRef : public BasicJointInterface<JointRef, IsConstRef>
    {
        template <typename, bool>
        friend class BasicJointInterface;

      protected:
        b2JointId id = b2_nullJointId;

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

        // Const a non-const reference to a const reference.
        constexpr MaybeConstJointRef(const MaybeConstJointRef<!IsConstRef>& other) noexcept requires IsConstRef : MaybeConstJointRef(other.Handle()) {}
    };

    template <typename D, bool ForceConst>
    class BasicDistanceJointInterface
    {
      protected:
        BasicDistanceJointInterface() = default;

      public:
        /// Adjust the softness parameters of a distance joint
        void SetTuning(float hertz, float dampingRatio) /*non-const*/ requires (!ForceConst);

        /// Get the current length of a distance joint
        [[nodiscard]] float GetCurrentLength() const;

        /// Get the minimum distance joint length
        [[nodiscard]] float GetMinLength() const;

        /// Get the constraint force on a distance joint
        [[nodiscard]] float GetConstraintForce(float timeStep) const;

        /// Set the minimum and maximum length parameters of a distance joint
        void SetLengthRange(float minLength, float maxLength) /*non-const*/ requires (!ForceConst);

        /// Set the rest length of a distance joint
        void SetLength(float length) /*non-const*/ requires (!ForceConst);

        /// Get the Hertz of a distance joint
        [[nodiscard]] float GetHertz() const;

        /// Get the maximum distance joint length
        [[nodiscard]] float GetMaxLength() const;

        /// Get the damping ratio of a distance joint
        [[nodiscard]] float GetDampingRatio() const;

        /// Get the rest length of a distance joint
        [[nodiscard]] float GetLength() const;
    };

    /// Distance joint definition. This requires defining an anchor point on both
    /// bodies and the non-zero distance of the distance joint. The definition uses
    /// local anchor points so that the initial configuration can violate the
    /// constraint slightly. This helps when saving and loading a game.
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
    };

    template <bool IsConstRef>
    class MaybeConstDistanceJointRef : public JointRef, public BasicDistanceJointInterface<DistanceJointRef, IsConstRef>
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

        // Const a non-const reference to a const reference.
        constexpr MaybeConstDistanceJointRef(const MaybeConstDistanceJointRef<!IsConstRef>& other) noexcept requires IsConstRef : MaybeConstDistanceJointRef(other.Handle()) {}
    };

    template <typename D, bool ForceConst>
    class BasicMotorJointInterface
    {
      protected:
        BasicMotorJointInterface() = default;

      public:
        /// Get the current constraint force for a motor joint
        [[nodiscard]] b2Vec2 GetConstraintForce() const;

        /// Get the current constraint torque for a motor joint
        [[nodiscard]] float GetConstraintTorque() const;

        /// @return the angular offset target for a motor joint in radians
        [[nodiscard]] float GetAngularOffset() const;

        /// Set the maximum force for a motor joint
        void SetMaxForce(float maxForce) /*non-const*/ requires (!ForceConst);

        /// @return the maximum force for a motor joint
        [[nodiscard]] float GetMaxForce() const;

        /// Set/Get the linear offset target for a motor joint
        void SetLinearOffset(b2Vec2 linearOffset) /*non-const*/ requires (!ForceConst);

        /// Set the correction factor for a motor joint
        void SetCorrectionFactor(float correctionFactor) /*non-const*/ requires (!ForceConst);

        /// @return the correction factor for a motor joint
        [[nodiscard]] float GetCorrectionFactor() const;

        /// Set the maximum torque for a motor joint
        void SetMaxTorque(float maxTorque) /*non-const*/ requires (!ForceConst);

        /// @return the linear offset target for a motor joint
        [[nodiscard]] b2Vec2 GetLinearOffset() const;

        /// Set the angular offset target for a motor joint in radians
        void SetAngularOffset(float angularOffset) /*non-const*/ requires (!ForceConst);

        /// @return the maximum torque for a motor joint
        [[nodiscard]] float GetMaxTorque() const;
    };

    /// A motor joint is used to control the relative motion
    /// between two bodies. A typical usage is to control the movement
    /// of a dynamic body with respect to the ground.
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
    };

    template <bool IsConstRef>
    class MaybeConstMotorJointRef : public JointRef, public BasicMotorJointInterface<MotorJointRef, IsConstRef>
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

        // Const a non-const reference to a const reference.
        constexpr MaybeConstMotorJointRef(const MaybeConstMotorJointRef<!IsConstRef>& other) noexcept requires IsConstRef : MaybeConstMotorJointRef(other.Handle()) {}
    };

    template <typename D, bool ForceConst>
    class BasicMouseJointInterface
    {
      protected:
        BasicMouseJointInterface() = default;

      public:
        /// @return the target for a mouse joint
        [[nodiscard]] b2Vec2 GetTarget() const;

        /// Get the Hertz of a mouse joint
        [[nodiscard]] float GetHertz() const;

        /// Get the damping ratio of a mouse joint
        [[nodiscard]] float GetDampingRatio() const;

        /// Set the target for a mouse joint
        void SetTarget(b2Vec2 target) /*non-const*/ requires (!ForceConst);

        /// Adjust the softness parameters of a mouse joint
        void SetTuning(float hertz, float dampingRatio) /*non-const*/ requires (!ForceConst);
    };

    /// A mouse joint is used to make a point on a body track a
    /// specified world point. This a soft constraint and allows the constraint to stretch without
    /// applying huge forces. This also applies rotation constraint heuristic to improve control.
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
    };

    template <bool IsConstRef>
    class MaybeConstMouseJointRef : public JointRef, public BasicMouseJointInterface<MouseJointRef, IsConstRef>
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

        // Const a non-const reference to a const reference.
        constexpr MaybeConstMouseJointRef(const MaybeConstMouseJointRef<!IsConstRef>& other) noexcept requires IsConstRef : MaybeConstMouseJointRef(other.Handle()) {}
    };

    template <typename D, bool ForceConst>
    class BasicPrismaticJointInterface
    {
      protected:
        BasicPrismaticJointInterface() = default;

      public:
        /// @return is the prismatic joint motor enabled
        [[nodiscard]] bool IsMotorEnabled() const;

        /// @return is the prismatic joint limit enabled
        [[nodiscard]] bool IsLimitEnabled() const;

        /// Enable/disable a prismatic joint motor
        void EnableMotor(bool enableMotor) /*non-const*/ requires (!ForceConst);

        /// Enable/disable a prismatic joint limit
        void EnableLimit(bool enableLimit) /*non-const*/ requires (!ForceConst);

        /// @return the maximum force for a prismatic joint motor
        [[nodiscard]] float GetMaxMotorForce() const;

        /// Get the upper joint limit in length units (meters).
        [[nodiscard]] float GetUpperLimit() const;

        /// Set the motor speed for a prismatic joint
        void SetMotorSpeed(float motorSpeed) /*non-const*/ requires (!ForceConst);

        /// Set the joint limits in length units (meters).
        void SetLimits(float lower, float upper) /*non-const*/ requires (!ForceConst);

        /// @return the motor speed for a prismatic joint
        [[nodiscard]] float GetMotorSpeed() const;

        /// Get the lower joint limit in length units (meters).
        [[nodiscard]] float GetLowerLimit() const;

        /// Get the current constraint torque for a prismatic joint
        [[nodiscard]] float GetConstraintTorque() const;

        /// Get the current constraint force for a prismatic joint
        [[nodiscard]] b2Vec2 GetConstraintForce() const;

        /// Get the current motor force for a prismatic joint
        [[nodiscard]] float GetMotorForce() const;

        /// Set the maximum force for a prismatic joint motor
        void SetMaxMotorForce(float force) /*non-const*/ requires (!ForceConst);
    };

    /// Prismatic joint definition. This requires defining a line of
    /// motion using an axis and an anchor point. The definition uses local
    /// anchor points and a local axis so that the initial configuration
    /// can violate the constraint slightly. The joint translation is zero
    /// when the local anchor points coincide in world space.
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
    };

    template <bool IsConstRef>
    class MaybeConstPrismaticJointRef : public JointRef, public BasicPrismaticJointInterface<PrismaticJointRef, IsConstRef>
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

        // Const a non-const reference to a const reference.
        constexpr MaybeConstPrismaticJointRef(const MaybeConstPrismaticJointRef<!IsConstRef>& other) noexcept requires IsConstRef : MaybeConstPrismaticJointRef(other.Handle()) {}
    };

    template <typename D, bool ForceConst>
    class BasicRevoluteJointInterface
    {
      protected:
        BasicRevoluteJointInterface() = default;

      public:
        /// Set the maximum torque for a revolute joint motor
        void SetMaxMotorTorque(float torque) /*non-const*/ requires (!ForceConst);

        /// Get the upper joint limit in radians.
        [[nodiscard]] float GetUpperLimit() const;

        /// @return the motor speed for a revolute joint in radians per second
        [[nodiscard]] float GetMotorSpeed() const;

        /// Set the motor speed for a revolute joint in radians per second
        void SetMotorSpeed(float motorSpeed) /*non-const*/ requires (!ForceConst);

        /// Get the lower joint limit in radians.
        [[nodiscard]] float GetLowerLimit() const;

        /// Set the joint limits in radians.
        void SetLimits(float lower, float upper) /*non-const*/ requires (!ForceConst);

        /// Get the current motor torque for a revolute joint
        [[nodiscard]] float GetMotorTorque() const;

        /// Get the current constraint torque for a revolute joint
        [[nodiscard]] float GetConstraintTorque() const;

        /// Get the current constraint force for a revolute joint
        [[nodiscard]] b2Vec2 GetConstraintForce() const;

        /// @return is the revolute joint motor enabled
        [[nodiscard]] bool IsMotorEnabled() const;

        /// @return is the revolute joint limit enabled
        [[nodiscard]] bool IsLimitEnabled() const;

        /// Enable/disable a revolute joint motor.
        void EnableMotor(bool enableMotor) /*non-const*/ requires (!ForceConst);

        /// Enable/disable a revolute joint limit.
        void EnableLimit(bool enableLimit) /*non-const*/ requires (!ForceConst);

        /// @return the maximum torque for a revolute joint motor
        [[nodiscard]] float GetMaxMotorTorque() const;
    };

    /// Revolute joint definition. This requires defining an anchor point where the
    /// sims are joined. The definition uses local anchor points so that the
    /// initial configuration can violate the constraint slightly. You also need to
    /// specify the initial relative angle for joint limits. This helps when saving
    /// and loading a game.
    /// The local anchor points are measured from the body's origin
    /// rather than the center of mass because:
    /// 1. you might not know where the center of mass will be.
    /// 2. if you add/remove shapes from a body and recompute the mass,
    ///    the joints will be broken.
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
    };

    template <bool IsConstRef>
    class MaybeConstRevoluteJointRef : public JointRef, public BasicRevoluteJointInterface<RevoluteJointRef, IsConstRef>
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

        // Const a non-const reference to a const reference.
        constexpr MaybeConstRevoluteJointRef(const MaybeConstRevoluteJointRef<!IsConstRef>& other) noexcept requires IsConstRef : MaybeConstRevoluteJointRef(other.Handle()) {}
    };

    template <typename D, bool ForceConst>
    class BasicWheelJointInterface
    {
      protected:
        BasicWheelJointInterface() = default;

      public:
        /// Set the wheel joint damping ratio (non-dimensional)
        void SetSpringDampingRatio(float dampingRatio) /*non-const*/ requires (!ForceConst);

        /// Get the upper joint limit in length units (meters).
        [[nodiscard]] float GetUpperLimit() const;

        /// Get the current wheel joint constraint torque
        [[nodiscard]] float GetConstraintTorque() const;

        /// Set the wheel joint stiffness in Hertz
        void SetSpringHertz(float hertz) /*non-const*/ requires (!ForceConst);

        /// @return the wheel joint motor speed in radians per second
        [[nodiscard]] float GetMotorSpeed() const;

        /// Get the wheel joint current motor torque
        [[nodiscard]] float GetMotorTorque() const;

        /// Set the wheel joint motor speed in radians per second
        void SetMotorSpeed(float motorSpeed) /*non-const*/ requires (!ForceConst);

        /// Get the lower joint limit in length units (meters).
        [[nodiscard]] float GetLowerLimit() const;

        /// Set the wheel joint maximum motor torque
        void SetMaxMotorTorque(float torque) /*non-const*/ requires (!ForceConst);

        /// @return is the wheel joint motor enabled
        [[nodiscard]] bool IsMotorEnabled() const;

        /// @return is the wheel joint limit enabled
        [[nodiscard]] bool IsLimitEnabled() const;

        /// @return the wheel joint damping ratio (non-dimensional)
        [[nodiscard]] float GetSpringDampingRatio() const;

        /// Enable/disable the wheel joint limit.
        void EnableLimit(bool enableLimit) /*non-const*/ requires (!ForceConst);

        /// Set the joint limits in length units (meters).
        void SetLimits(float lower, float upper) /*non-const*/ requires (!ForceConst);

        /// Get the current wheel joint constraint force
        [[nodiscard]] b2Vec2 GetConstraintForce() const;

        /// Enable/disable the wheel joint motor
        void EnableMotor(bool enableMotor) /*non-const*/ requires (!ForceConst);

        /// @return the wheel joint stiffness in Hertz
        [[nodiscard]] float GetSpringHertz() const;

        /// @return the wheel joint maximum motor torque
        [[nodiscard]] float GetMaxMotorTorque() const;
    };

    /// Wheel joint definition. This requires defining a line of
    /// motion using an axis and an anchor point. The definition uses local
    /// anchor points and a local axis so that the initial configuration
    /// can violate the constraint slightly. The joint translation is zero
    /// when the local anchor points coincide in world space. Using local
    /// anchors and a local axis helps when saving and loading a game.
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
    };

    template <bool IsConstRef>
    class MaybeConstWheelJointRef : public JointRef, public BasicWheelJointInterface<WheelJointRef, IsConstRef>
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

        // Const a non-const reference to a const reference.
        constexpr MaybeConstWheelJointRef(const MaybeConstWheelJointRef<!IsConstRef>& other) noexcept requires IsConstRef : MaybeConstWheelJointRef(other.Handle()) {}
    };

    template <typename D, bool ForceConst>
    class BasicWeldJointInterface
    {
      protected:
        BasicWeldJointInterface() = default;

      public:
        /// Set weld joint angular stiffness in Hertz. 0 is rigid.
        void SetAngularHertz(float hertz) /*non-const*/ requires (!ForceConst);

        /// Set weld joint angular damping ratio (non-dimensional)
        void SetAngularDampingRatio(float dampingRatio) /*non-const*/ requires (!ForceConst);

        /// @return the weld joint angular stiffness in Hertz.
        [[nodiscard]] float GetAngularHertz() const;

        /// @return the weld joint linear stiffness in Hertz.
        [[nodiscard]] float GetLinearHertz() const;

        /// @return the weld joint angular damping ratio (non-dimensional)
        [[nodiscard]] float GetAngularDampingRatio() const;

        /// @return the weld joint linear damping ratio (non-dimensional)
        [[nodiscard]] float GetLinearDampingRatio() const;

        /// Set weld joint linear stiffness in Hertz. 0 is rigid.
        void SetLinearHertz(float hertz) /*non-const*/ requires (!ForceConst);

        /// Set weld joint linear damping ratio (non-dimensional)
        void SetLinearDampingRatio(float dampingRatio) /*non-const*/ requires (!ForceConst);
    };

    /// A weld joint connect to sims together rigidly. This constraint can be made soft to mimic
    ///	soft-body simulation.
    /// @warning the approximate solver in Box2D cannot hold many sims together rigidly
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
    };

    template <bool IsConstRef>
    class MaybeConstWeldJointRef : public JointRef, public BasicWeldJointInterface<WeldJointRef, IsConstRef>
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

        // Const a non-const reference to a const reference.
        constexpr MaybeConstWeldJointRef(const MaybeConstWeldJointRef<!IsConstRef>& other) noexcept requires IsConstRef : MaybeConstWeldJointRef(other.Handle()) {}
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
        template <std::same_as<b2BodyId> T> [[nodiscard]] operator const T &() const { return Handle(); }

        /// Create a polygon shape and attach it to a body. The shape definition and geometry are fully cloned.
        /// Contacts are not created until the next time step.
        ///	@return the shape id for accessing the shape
        [[nodiscard]] Shape CreatePolygonShape(Tags::OwningHandle, const std::derived_from<b2ShapeDef> auto& def, const b2Polygon& polygon) /*non-const*/ requires (!ForceConst);
        ShapeRef CreatePolygonShape(Tags::DestroyWithParent, const std::derived_from<b2ShapeDef> auto& def, const b2Polygon& polygon) /*non-const*/ requires (!ForceConst);

        /// Create a circle shape and attach it to a body. The shape definition and geometry are fully cloned.
        /// Contacts are not created until the next time step.
        ///	@return the shape id for accessing the shape
        [[nodiscard]] Shape CreateCircleShape(Tags::OwningHandle, const std::derived_from<b2ShapeDef> auto& def, const b2Circle& circle) /*non-const*/ requires (!ForceConst);
        ShapeRef CreateCircleShape(Tags::DestroyWithParent, const std::derived_from<b2ShapeDef> auto& def, const b2Circle& circle) /*non-const*/ requires (!ForceConst);

        /// Create a line segment shape and attach it to a body. The shape definition and geometry are fully cloned.
        /// Contacts are not created until the next time step.
        ///	@return the shape id for accessing the shape
        [[nodiscard]] Shape CreateSegmentShape(Tags::OwningHandle, const std::derived_from<b2ShapeDef> auto& def, const b2Segment& segment) /*non-const*/ requires (!ForceConst);
        ShapeRef CreateSegmentShape(Tags::DestroyWithParent, const std::derived_from<b2ShapeDef> auto& def, const b2Segment& segment) /*non-const*/ requires (!ForceConst);

        /// Create a capsule shape and attach it to a body. The shape definition and geometry are fully cloned.
        /// Contacts are not created until the next time step.
        ///	@return the shape id for accessing the shape
        [[nodiscard]] Shape CreateCapsuleShape(Tags::OwningHandle, const std::derived_from<b2ShapeDef> auto& def, const b2Capsule& capsule) /*non-const*/ requires (!ForceConst);
        ShapeRef CreateCapsuleShape(Tags::DestroyWithParent, const std::derived_from<b2ShapeDef> auto& def, const b2Capsule& capsule) /*non-const*/ requires (!ForceConst);

        /// Create a chain shape
        ///	@see b2ChainDef for details
        [[nodiscard]] Chain CreateChain(Tags::OwningHandle, const std::derived_from<b2ChainDef> auto& def) /*non-const*/ requires (!ForceConst);
        ChainRef CreateChain(Tags::DestroyWithParent, const std::derived_from<b2ChainDef> auto& def) /*non-const*/ requires (!ForceConst);

        /// Destroy a rigid body given an id. This destroys all shapes and joints attached to the body.
        ///	Do not keep references to the associated shapes and joints.
        /// @warning This function is locked during callbacks.
        void Destroy() /*non-const*/ requires (!ForceConst);

        /// Get the inertia tensor of the body. In 2D this is a single number. (kilograms * meters^2)
        [[nodiscard]] float GetInertiaTensor() const;

        /// Apply an impulse at a point. This immediately modifies the velocity.
        /// It also modifies the angular velocity if the point of application
        /// is not at the center of mass. This wakes up the body.
        /// This should be used for one-shot impulses. If you need a steady force,
        /// use a force instead, which will work better with the sub-stepping solver.
        /// @param impulse the world impulse vector, usually in N-seconds or kg-m/s.
        /// @param point the world position of the point of application.
        /// @param wake also wake up the body
        void ApplyLinearImpulse(b2Vec2 impulse, b2Vec2 point, bool wake) /*non-const*/ requires (!ForceConst);

        /// Get a world point on a body given a local point
        [[nodiscard]] b2Vec2 GetWorldPoint(b2Vec2 localPoint) const;

        /// Is this body a bullet?
        [[nodiscard]] bool IsBullet() const;

        /// Adjust the gravity scale. Normally this is set in b2BodyDef before creation.
        void SetGravityScale(float gravityScale) /*non-const*/ requires (!ForceConst);

        /// Get the number of joints on this body
        [[nodiscard]] int GetJointCount() const;

        /// Get the shape ids for all shapes on this body, up to the provided capacity.
        ///	@returns the number of shape ids stored in the user array
        [[nodiscard]] int GetShapes(b2ShapeId& shapeArray, int capacity) const;

        /// Change the body type. This is an expensive operation.
        void SetType(b2BodyType type) /*non-const*/ requires (!ForceConst);

        /// Get the maximum capacity required for retrieving all the touching contacts on a body
        [[nodiscard]] int GetContactCapacity() const;

        /// Get the current gravity scale.
        [[nodiscard]] float GetGravityScale() const;

        /// Get the center of mass position of the body in local space.
        [[nodiscard]] b2Vec2 GetLocalCenterOfMass() const;

        /// Get the center of mass position of the body in world space.
        [[nodiscard]] b2Vec2 GetWorldCenterOfMass() const;

        /// Set this body to have fixed rotation. This causes the mass to be reset.
        void SetFixedRotation(bool flag) /*non-const*/ requires (!ForceConst);

        /// Adjust the angular damping. Normally this is set in b2BodyDef before creation.
        void SetAngularDamping(float angularDamping) /*non-const*/ requires (!ForceConst);

        /// Apply an angular impulse.
        /// This should be used for one-shot impulses. If you need a steady force,
        /// use a force instead, which will work better with the sub-stepping solver.
        /// @param impulse the angular impulse in units of
        /// kg*m*m/s
        /// @param wake also wake up the body
        void ApplyAngularImpulse(float impulse, bool wake) /*non-const*/ requires (!ForceConst);

        /// Get a local vector on a body given a world vector
        [[nodiscard]] b2Vec2 GetLocalVector(b2Vec2 worldVector) const;

        /// Override the body's mass properties. Normally this is computed automatically using the
        ///	shape geometry and density. This information is lost if a shape is added or removed or if the
        ///	body type changes.
        void SetMassData(b2MassData massData) /*non-const*/ requires (!ForceConst);

        /// Get the number of shapes on this body
        [[nodiscard]] int GetShapeCount() const;

        /// Adjust the linear damping. Normally this is set in b2BodyDef before creation.
        void SetLinearDamping(float linearDamping) /*non-const*/ requires (!ForceConst);

        /// Disable a body by removing it completely from the simulation
        void Disable() /*non-const*/ requires (!ForceConst);

        /// Set the linear velocity of a body
        void SetLinearVelocity(b2Vec2 linearVelocity) /*non-const*/ requires (!ForceConst);

        /// Get a local point on a body given a world point
        [[nodiscard]] b2Vec2 GetLocalPoint(b2Vec2 worldPoint) const;

        /// Get the mass data for a body.
        [[nodiscard]] b2MassData GetMassData() const;

        /// Set the world transform of a body. This acts as a teleport and is fairly expensive.
        void SetTransform(b2Vec2 position, float angle) /*non-const*/ requires (!ForceConst);

        /// Get the body angle in radians in the range [-pi, pi]
        [[nodiscard]] float GetAngle() const;

        /// Set the angular velocity of a body in radians per second
        void SetAngularVelocity(float angularVelocity) /*non-const*/ requires (!ForceConst);

        /// Get the joint ids for all joints on this body, up to the provided capacity
        ///	@returns the number of joint ids stored in the user array
        [[nodiscard]] int GetJoints(b2JointId& jointArray, int capacity) const;

        /// Enable or disable sleeping this body. If sleeping is disabled the body will wake.
        void EnableSleep(bool enableSleep) /*non-const*/ requires (!ForceConst);

        /// Apply a force at a world point. If the force is not
        /// applied at the center of mass, it will generate a torque and
        /// affect the angular velocity. This wakes up the body.
        /// @param force the world force vector, usually in Newtons (N).
        /// @param point the world position of the point of application.
        /// @param wake also wake up the body
        void ApplyForce(b2Vec2 force, b2Vec2 point, bool wake) /*non-const*/ requires (!ForceConst);

        /// Get the current angular damping.
        [[nodiscard]] float GetAngularDamping() const;

        /// Does this body have fixed rotation?
        [[nodiscard]] bool IsFixedRotation() const;

        /// This resets the mass properties to the sum of the mass properties of the fixtures.
        /// This normally does not need to be called unless you called SetMassData to override
        /// the mass and you later want to reset the mass.
        void ResetMassData() /*non-const*/ requires (!ForceConst);

        /// Set the user data for a body
        void SetUserData(void* userData) /*non-const*/ requires (!ForceConst);

        /// Get the body type: static, kinematic, or dynamic
        [[nodiscard]] b2BodyType GetType() const;

        /// @return is sleeping enabled for this body?
        [[nodiscard]] bool IsSleepEnabled() const;

        /// Get the angular velocity of a body in radians per second
        [[nodiscard]] float GetAngularVelocity() const;

        /// Body identifier validation. Provides validation for up to 64K allocations.
        [[nodiscard]] bool IsValid() const;

        /// Enable a body by adding it to the simulation
        void Enable() /*non-const*/ requires (!ForceConst);

        /// Apply an impulse to the center of mass. This immediately modifies the velocity.
        /// This should be used for one-shot impulses. If you need a steady force,
        /// use a force instead, which will work better with the sub-stepping solver.
        /// @param impulse the world impulse vector, usually in N-seconds or kg-m/s.
        /// @param wake also wake up the body
        void ApplyLinearImpulseToCenter(b2Vec2 impulse, bool wake) /*non-const*/ requires (!ForceConst);

        /// Wake a body from sleep. This wakes the entire island the body is touching.
        ///	Putting a body to sleep will put the entire island of bodies touching this body to sleep,
        ///	which can be expensive.
        void SetAwake(bool awake) /*non-const*/ requires (!ForceConst);

        /// Get the user data stored in a body
        [[nodiscard]] void* GetUserData() const;

        /// Apply a torque. This affects the angular velocity
        /// without affecting the linear velocity of the center of mass.
        /// @param torque about the z-axis (out of the screen), usually in N-m.
        /// @param wake also wake up the body
        void ApplyTorque(float torque, bool wake) /*non-const*/ requires (!ForceConst);

        /// Get the world rotation of a body as a sine/cosine pair.
        [[nodiscard]] b2Rot GetRotation() const;

        /// Get the current world AABB that contains all the attached shapes. Note that this may not emcompass the body origin.
        ///	If there are no shapes attached then the returned AABB is empty and centered on the body origin.
        [[nodiscard]] b2AABB ComputeAABB() const;

        /// Get the linear velocity of a body's center of mass
        [[nodiscard]] b2Vec2 GetLinearVelocity() const;

        /// Get the touching contact data for a body
        [[nodiscard]] int GetContactData(b2ContactData& contactData, int capacity) const;

        /// Is this body enabled?
        [[nodiscard]] bool IsEnabled() const;

        /// Apply a force to the center of mass. This wakes up the body.
        /// @param force the world force vector, usually in Newtons (N).
        /// @param wake also wake up the body
        void ApplyForceToCenter(b2Vec2 force, bool wake) /*non-const*/ requires (!ForceConst);

        /// Is this body awake?
        [[nodiscard]] bool IsAwake() const;

        /// Get the mass of the body (kilograms)
        [[nodiscard]] float GetMass() const;

        /// Get a world vector on a body given a local vector
        [[nodiscard]] b2Vec2 GetWorldVector(b2Vec2 localVector) const;

        /// Set this body to be a bullet. A bullet does continuous collision detection
        /// against dynamic bodies (but not other bullets).
        void SetBullet(bool flag) /*non-const*/ requires (!ForceConst);

        /// Get the world position of a body. This is the location of the body origin.
        [[nodiscard]] b2Vec2 GetPosition() const;

        /// Get the world transform of a body.
        [[nodiscard]] b2Transform GetTransform() const;

        /// Get the current linear damping.
        [[nodiscard]] float GetLinearDamping() const;
    };

    /// A body definition holds all the data needed to construct a rigid body.
    /// You can safely re-use body definitions. Shapes are added to a body after construction.
    class Body : public BasicBodyInterface<Body, false>
    {
        template <typename, bool>
        friend class BasicBodyInterface;
        template <typename, bool>
        friend class BasicWorldInterface;


      protected:
        b2BodyId id = b2_nullBodyId;

      public:
        static constexpr bool IsOwning = true;

        // Constructs a null (invalid) object.
        constexpr Body() noexcept {}

        // The constructor accepts either this or directly `b2BodyDef`.
        struct Params : b2BodyDef
        {
            Params() : b2BodyDef(b2DefaultBodyDef()) {}
        };

        Body(Body&& other) noexcept { id = std::exchange(other.id, b2_nullBodyId); }
        Body& operator=(Body other) noexcept { std::swap(id, other.id); return *this; }

        ~Body() { if (*this) Destroy(); }
    };

    template <bool IsConstRef>
    class MaybeConstBodyRef : public BasicBodyInterface<BodyRef, IsConstRef>
    {
        template <typename, bool>
        friend class BasicBodyInterface;

      protected:
        b2BodyId id = b2_nullBodyId;

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

        // Const a non-const reference to a const reference.
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
        template <std::same_as<b2WorldId> T> [[nodiscard]] operator const T &() const { return Handle(); }

        /// Create a weld joint
        ///	@see b2WeldJointDef for details
        [[nodiscard]] WeldJoint CreateWeldJoint(Tags::OwningHandle, const std::derived_from<b2WeldJointDef> auto& def) /*non-const*/ requires (!ForceConst);
        WeldJointRef CreateWeldJoint(Tags::DestroyWithParent, const std::derived_from<b2WeldJointDef> auto& def) /*non-const*/ requires (!ForceConst);

        /// Create a rigid body given a definition. No reference to the definition is retained.
        /// @warning This function is locked during callbacks.
        [[nodiscard]] Body CreateBody(Tags::OwningHandle, const std::derived_from<b2BodyDef> auto& def) /*non-const*/ requires (!ForceConst);
        BodyRef CreateBody(Tags::DestroyWithParent, const std::derived_from<b2BodyDef> auto& def) /*non-const*/ requires (!ForceConst);

        /// Create a distance joint
        ///	@see b2DistanceJointDef for details
        [[nodiscard]] DistanceJoint CreateDistanceJoint(Tags::OwningHandle, const std::derived_from<b2DistanceJointDef> auto& def) /*non-const*/ requires (!ForceConst);
        DistanceJointRef CreateDistanceJoint(Tags::DestroyWithParent, const std::derived_from<b2DistanceJointDef> auto& def) /*non-const*/ requires (!ForceConst);

        /// Create a mouse joint
        ///	@see b2MouseJointDef for details
        [[nodiscard]] MouseJoint CreateMouseJoint(Tags::OwningHandle, const std::derived_from<b2MouseJointDef> auto& def) /*non-const*/ requires (!ForceConst);
        MouseJointRef CreateMouseJoint(Tags::DestroyWithParent, const std::derived_from<b2MouseJointDef> auto& def) /*non-const*/ requires (!ForceConst);

        /// Create a wheel joint
        ///	@see b2WheelJointDef for details
        [[nodiscard]] WheelJoint CreateWheelJoint(Tags::OwningHandle, const std::derived_from<b2WheelJointDef> auto& def) /*non-const*/ requires (!ForceConst);
        WheelJointRef CreateWheelJoint(Tags::DestroyWithParent, const std::derived_from<b2WheelJointDef> auto& def) /*non-const*/ requires (!ForceConst);

        /// Create a motor joint
        ///	@see b2MotorJointDef for details
        [[nodiscard]] MotorJoint CreateMotorJoint(Tags::OwningHandle, const std::derived_from<b2MotorJointDef> auto& def) /*non-const*/ requires (!ForceConst);
        MotorJointRef CreateMotorJoint(Tags::DestroyWithParent, const std::derived_from<b2MotorJointDef> auto& def) /*non-const*/ requires (!ForceConst);

        /// Create a revolute (hinge) joint
        ///	@see b2RevoluteJointDef for details
        [[nodiscard]] RevoluteJoint CreateRevoluteJoint(Tags::OwningHandle, const std::derived_from<b2RevoluteJointDef> auto& def) /*non-const*/ requires (!ForceConst);
        RevoluteJointRef CreateRevoluteJoint(Tags::DestroyWithParent, const std::derived_from<b2RevoluteJointDef> auto& def) /*non-const*/ requires (!ForceConst);

        /// Create a prismatic (slider) joint
        ///	@see b2PrismaticJointDef for details
        [[nodiscard]] PrismaticJoint CreatePrismaticJoint(Tags::OwningHandle, const std::derived_from<b2PrismaticJointDef> auto& def) /*non-const*/ requires (!ForceConst);
        PrismaticJointRef CreatePrismaticJoint(Tags::DestroyWithParent, const std::derived_from<b2PrismaticJointDef> auto& def) /*non-const*/ requires (!ForceConst);

        /// Destroy a world.
        void Destroy() /*non-const*/ requires (!ForceConst);

        /// @return the gravity vector
        [[nodiscard]] b2Vec2 GetGravity() const;

        /// Overlap test for all shapes that overlap the provided capsule.
        void OverlapCapsule(const b2Capsule& capsule, b2Transform transform, b2QueryFilter filter, b2OverlapResultFcn* fcn, void* context) const;

        /// Get counters and sizes
        [[nodiscard]] b2Counters GetCounters() const;

        /// Get sensor events for the current time step. The event data is transient. Do not store a reference to this data.
        [[nodiscard]] b2SensorEvents GetSensorEvents() const;

        /// Register the pre-solve callback. This is optional.
        void SetPreSolveCallback(b2PreSolveFcn* fcn, void* context) /*non-const*/ requires (!ForceConst);

        /// Simulate a world for one time step. This performs collision detection, integration, and constraint solution.
        /// @param worldId the world to simulate
        /// @param timeStep the amount of time to simulate, this should be a fixed number. Typically 1/60.
        /// @param subStepCount the number of sub-steps, increasing the sub-step count can increase accuracy. Typically 4.
        void Step(float timeStep, int subStepCount) /*non-const*/ requires (!ForceConst);

        /// Ray-cast closest hit. Convenience function. This is less general than b2World_RayCast and does not allow for custom filtering.
        [[nodiscard]] b2RayResult RayCastClosest(b2Vec2 origin, b2Vec2 translation, b2QueryFilter filter) const;

        /// Cast a capsule through the world. Similar to a ray-cast except that a capsule is cast instead of a point.
        void CapsuleCast(const b2Capsule& capsule, b2Transform originTransform, b2Vec2 translation, b2QueryFilter filter, b2CastResultFcn* fcn, void* context) const;

        /// Adjust the restitution threshold. Advanced feature for testing.
        void SetRestitutionThreshold(float value) /*non-const*/ requires (!ForceConst);

        /// Cast a capsule through the world. Similar to a ray-cast except that a polygon is cast instead of a point.
        void PolygonCast(const b2Polygon& polygon, b2Transform originTransform, b2Vec2 translation, b2QueryFilter filter, b2CastResultFcn* fcn, void* context) const;

        /// Overlap test for all shapes that overlap the provided polygon.
        void OverlapPolygon(const b2Polygon& polygon, b2Transform transform, b2QueryFilter filter, b2OverlapResultFcn* fcn, void* context) const;

        /// Cast a circle through the world. Similar to a ray-cast except that a circle is cast instead of a point.
        void CircleCast(const b2Circle& circle, b2Transform originTransform, b2Vec2 translation, b2QueryFilter filter, b2CastResultFcn* fcn, void* context) const;

        /// Overlap test for for all shapes that overlap the provided circle.
        void OverlapCircle(const b2Circle& circle, b2Transform transform, b2QueryFilter filter, b2OverlapResultFcn* fcn, void* context) const;

        /// Adjust contact tuning parameters:
        /// - hertz is the contact stiffness (cycles per second)
        /// - damping ratio is the contact bounciness with 1 being critical damping (non-dimensional)
        /// - push velocity is the maximum contact constraint push out velocity (meters per second)
        ///	Advanced feature
        void SetContactTuning(float hertz, float dampingRatio, float pushVelocity) /*non-const*/ requires (!ForceConst);

        /// Get the current profile
        [[nodiscard]] b2Profile GetProfile() const;

        /// Call this to draw shapes and other debug draw data. This is intentionally non-const.
        void Draw(b2DebugDraw& draw) const;

        /// Enable/disable continuous collision. Advanced feature for testing.
        void EnableContinuous(bool flag) /*non-const*/ requires (!ForceConst);

        /// Enable/disable sleep. Advanced feature for testing.
        void EnableSleeping(bool flag) /*non-const*/ requires (!ForceConst);

        /// World identifier validation. Provides validation for up to 64K allocations.
        [[nodiscard]] bool IsValid() const;

        /// Overlap test for all shapes that *potentially* overlap the provided AABB.
        void OverlapAABB(b2AABB aabb, b2QueryFilter filter, b2OverlapResultFcn* fcn, void* context) const;

        /// Get contact events for this current time step. The event data is transient. Do not store a reference to this data.
        [[nodiscard]] b2ContactEvents GetContactEvents() const;

        /// Get the body events for the current time step. The event data is transient. Do not store a reference to this data.
        [[nodiscard]] b2BodyEvents GetBodyEvents() const;

        /// Ray-cast the world for all shapes in the path of the ray. Your callback
        /// controls whether you get the closest point, any point, or n-points.
        /// The ray-cast ignores shapes that contain the starting point.
        /// @param callback a user implemented callback class.
        /// @param point1 the ray starting point
        /// @param point2 the ray ending point
        void RayCast(b2Vec2 origin, b2Vec2 translation, b2QueryFilter filter, b2CastResultFcn* fcn, void* context) const;

        /// Enable/disable constraint warm starting. Advanced feature for testing.
        void EnableWarmStarting(bool flag) /*non-const*/ requires (!ForceConst);

        /// Set the gravity vector for the entire world. Typically in m/s^2
        void SetGravity(b2Vec2 gravity) /*non-const*/ requires (!ForceConst);
    };

    /// World definition used to create a simulation world. Must be initialized using b2DefaultWorldDef.
    class World : public BasicWorldInterface<World, false>
    {
        template <typename, bool>
        friend class BasicWorldInterface;

      protected:
        b2WorldId id = b2_nullWorldId;

      public:
        static constexpr bool IsOwning = true;

        // Constructs a null (invalid) object.
        constexpr World() noexcept {}

        // The constructor accepts either this or directly `b2WorldDef`.
        struct Params : b2WorldDef
        {
            Params() : b2WorldDef(b2DefaultWorldDef()) {}
        };

        /// Create a world for rigid body simulation. This contains all the bodies, shapes, and constraints.
        World(const std::derived_from<b2WorldDef> auto &params) { id = b2CreateWorld(&params); }

        World(World&& other) noexcept { id = std::exchange(other.id, b2_nullWorldId); }
        World& operator=(World other) noexcept { std::swap(id, other.id); return *this; }

        ~World() { if (*this) Destroy(); }
    };

    template <bool IsConstRef>
    class MaybeConstWorldRef : public BasicWorldInterface<WorldRef, IsConstRef>
    {
        template <typename, bool>
        friend class BasicWorldInterface;

      protected:
        b2WorldId id = b2_nullWorldId;

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

        // Const a non-const reference to a const reference.
        constexpr MaybeConstWorldRef(const MaybeConstWorldRef<!IsConstRef>& other) noexcept requires IsConstRef : MaybeConstWorldRef(other.Handle()) {}
    };

    /// A dynamic AABB tree broad-phase, inspired by Nathanael Presson's btDbvt.
    /// A dynamic tree arranges data in a binary tree to accelerate
    /// queries such as AABB queries and ray casts. Leaf nodes are proxies
    /// with an AABB. These are used to hold a user collision object, such as a reference to a b2Shape.
    /// Nodes are pooled and relocatable, so I use node indices rather than pointers.
    ///	The dynamic tree is made available for advanced users that would like to use it to organize
    ///	spatial game data besides rigid bodies.
    class DynamicTree
    {
        b2DynamicTree value{};

      public:
        // Consturcts a null (invalid) object.
        constexpr DynamicTree() {}

        /// Constructing the tree initializes the node pool.
        DynamicTree(std::nullptr_t) : value(b2DynamicTree_Create()) {}

        DynamicTree(const DynamicTree& other) : DynamicTree() { *this = other; }
        DynamicTree(DynamicTree&& other) noexcept : value(other.value) { other.value = {}; }
        // Clone one tree to another, reusing storage in the outTree if possible
        DynamicTree& operator=(const DynamicTree& other)
        {
            if (this == &other) {}
            else if (!other) *this = {};
            else
            {
                if (!*this) *this = nullptr;
                b2DynamicTree_Clone(&value, &other.value);
            }
            return *this;
        }
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

        /// Get the ratio of the sum of the node areas to the root area.
        [[nodiscard]] float GetAreaRatio() const;

        /// Ray-cast against the proxies in the tree. This relies on the callback
        /// to perform a exact ray-cast in the case were the proxy contains a shape.
        /// The callback also performs the any collision filtering. This has performance
        /// roughly equal to k * log(n), where k is the number of collisions and n is the
        /// number of proxies in the tree.
        /// @param input the ray-cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1).
        /// @param callback a callback class that is called for each proxy that is hit by the ray.
        void RayCast(const b2RayCastInput& input, uint32_t maskBits, b2TreeRayCastCallbackFcn* callback, void* context) const;

        /// Compute the height of the binary tree in O(N) time. Should not be
        /// called often.
        [[nodiscard]] int32_t GetHeight() const;

        /// Move a proxy to a new AABB by removing and reinserting into the tree.
        void MoveProxy(int32_t proxyId, b2AABB aabb);

        /// Shift the world origin. Useful for large worlds.
        /// The shift formula is: position -= newOrigin
        /// @param newOrigin the new origin with respect to the old origin
        void ShiftOrigin(b2Vec2 newOrigin);

        /// Validate this tree. For testing.
        void Validate() const;

        /// Get the AABB of a proxy
        [[nodiscard]] b2AABB GetAABB(int32_t proxyId) const;

        /// Build an optimal tree. Very expensive. For testing.
        void RebuildBottomUp();

        /// Ray-cast against the proxies in the tree. This relies on the callback
        /// to perform a exact ray-cast in the case were the proxy contains a shape.
        /// The callback also performs the any collision filtering. This has performance
        /// roughly equal to k * log(n), where k is the number of collisions and n is the
        /// number of proxies in the tree.
        /// @param input the ray-cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1).
        /// @param callback a callback class that is called for each proxy that is hit by the ray.
        void ShapeCast(const b2ShapeCastInput& input, uint32_t maskBits, b2TreeShapeCastCallbackFcn* callback, void* context) const;

        /// Get the number of proxies created
        [[nodiscard]] int32_t GetProxyCount() const;

        /// Create a proxy. Provide a tight fitting AABB and a userData value.
        [[nodiscard]] int32_t CreateProxy(b2AABB aabb, uint32_t categoryBits, int32_t userData);

        /// Get proxy user data
        /// @return the proxy user data or 0 if the id is invalid
        [[nodiscard]] int32_t GetUserData(int32_t proxyId) const;

        /// Query an AABB for overlapping proxies. The callback class
        /// is called for each proxy that overlaps the supplied AABB.
        void QueryFiltered(b2AABB aabb, uint32_t maskBits, b2TreeQueryCallbackFcn* callback, void* context) const;

        /// Get the maximum balance of the tree. The balance is the difference in height of the two children of a node.
        [[nodiscard]] int32_t GetMaxBalance() const;

        /// Query an AABB for overlapping proxies. The callback class
        /// is called for each proxy that overlaps the supplied AABB.
        void Query(b2AABB aabb, b2TreeQueryCallbackFcn* callback, void* context) const;

        /// Destroy a proxy. This asserts if the id is invalid.
        void DestroyProxy(int32_t proxyId);

        /// Rebuild the tree while retaining subtrees that haven't changed. Returns the number of boxes sorted.
        int32_t Rebuild(bool fullBuild);

        /// Enlarge a proxy and enlarge ancestors as necessary.
        void EnlargeProxy(int32_t proxyId, b2AABB aabb);
    };
} // namespace box2d

// Implementations:
namespace b2
{
    template <typename D, bool ForceConst> void BasicChainInterface<D, ForceConst>::Destroy() requires (!ForceConst) { if (*this) { b2DestroyChain(static_cast<const D &>(*this).Handle()); static_cast<D &>(*this).id = b2_nullChainId; } }
    template <typename D, bool ForceConst> void BasicChainInterface<D, ForceConst>::SetRestitution(float restitution) requires (!ForceConst) { b2Chain_SetRestitution(static_cast<const D &>(*this).Handle(), restitution); }
    template <typename D, bool ForceConst> bool BasicChainInterface<D, ForceConst>::IsValid() const { return b2Chain_IsValid(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicChainInterface<D, ForceConst>::SetFriction(float friction) requires (!ForceConst) { b2Chain_SetFriction(static_cast<const D &>(*this).Handle(), friction); }
    template <typename D, bool ForceConst> void BasicShapeInterface<D, ForceConst>::Destroy() requires (!ForceConst) { if (*this) { b2DestroyShape(static_cast<const D &>(*this).Handle()); static_cast<D &>(*this).id = b2_nullShapeId; } }
    template <typename D, bool ForceConst> void* BasicShapeInterface<D, ForceConst>::GetUserData() const { return b2Shape_GetUserData(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> b2CastOutput BasicShapeInterface<D, ForceConst>::RayCast(b2Vec2 origin, b2Vec2 translation) const { return b2Shape_RayCast(static_cast<const D &>(*this).Handle(), origin, translation); }
    template <typename D, bool ForceConst> b2Segment BasicShapeInterface<D, ForceConst>::GetSegment() const { return b2Shape_GetSegment(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> bool BasicShapeInterface<D, ForceConst>::AreContactEventsEnabled() const { return b2Shape_AreContactEventsEnabled(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicShapeInterface<D, ForceConst>::SetCapsule(const b2Capsule& capsule) requires (!ForceConst) { b2Shape_SetCapsule(static_cast<const D &>(*this).Handle(), &capsule); }
    template <typename D, bool ForceConst> bool BasicShapeInterface<D, ForceConst>::AreSensorEventsEnabled() const { return b2Shape_AreSensorEventsEnabled(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicShapeInterface<D, ForceConst>::SetFilter(b2Filter filter) requires (!ForceConst) { b2Shape_SetFilter(static_cast<const D &>(*this).Handle(), filter); }
    template <typename D, bool ForceConst> b2Circle BasicShapeInterface<D, ForceConst>::GetCircle() const { return b2Shape_GetCircle(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> int BasicShapeInterface<D, ForceConst>::GetContactCapacity() const { return b2Shape_GetContactCapacity(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicShapeInterface<D, ForceConst>::SetDensity(float density) requires (!ForceConst) { b2Shape_SetDensity(static_cast<const D &>(*this).Handle(), density); }
    template <typename D, bool ForceConst> b2Capsule BasicShapeInterface<D, ForceConst>::GetCapsule() const { return b2Shape_GetCapsule(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> int BasicShapeInterface<D, ForceConst>::GetContactData(b2ContactData& contactData, int capacity) const { return b2Shape_GetContactData(static_cast<const D &>(*this).Handle(), &contactData, capacity); }
    template <typename D, bool ForceConst> b2AABB BasicShapeInterface<D, ForceConst>::GetAABB() const { return b2Shape_GetAABB(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> float BasicShapeInterface<D, ForceConst>::GetDensity() const { return b2Shape_GetDensity(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicShapeInterface<D, ForceConst>::SetPolygon(const b2Polygon& polygon) requires (!ForceConst) { b2Shape_SetPolygon(static_cast<const D &>(*this).Handle(), &polygon); }
    template <typename D, bool ForceConst> void BasicShapeInterface<D, ForceConst>::EnableContactEvents(bool flag) requires (!ForceConst) { b2Shape_EnableContactEvents(static_cast<const D &>(*this).Handle(), flag); }
    template <typename D, bool ForceConst> b2Filter BasicShapeInterface<D, ForceConst>::GetFilter() const { return b2Shape_GetFilter(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicShapeInterface<D, ForceConst>::EnablePreSolveEvents(bool flag) requires (!ForceConst) { b2Shape_EnablePreSolveEvents(static_cast<const D &>(*this).Handle(), flag); }
    template <typename D, bool ForceConst> float BasicShapeInterface<D, ForceConst>::GetRestitution() const { return b2Shape_GetRestitution(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> b2Polygon BasicShapeInterface<D, ForceConst>::GetPolygon() const { return b2Shape_GetPolygon(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> bool BasicShapeInterface<D, ForceConst>::TestPoint(b2Vec2 point) const { return b2Shape_TestPoint(static_cast<const D &>(*this).Handle(), point); }
    template <typename D, bool ForceConst> bool BasicShapeInterface<D, ForceConst>::IsSensor() const { return b2Shape_IsSensor(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> b2SmoothSegment BasicShapeInterface<D, ForceConst>::GetSmoothSegment() const { return b2Shape_GetSmoothSegment(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> bool BasicShapeInterface<D, ForceConst>::ArePreSolveEventsEnabled() const { return b2Shape_ArePreSolveEventsEnabled(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> BodyRef BasicShapeInterface<D, ForceConst>::GetBody() const { return b2Shape_GetBody(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicShapeInterface<D, ForceConst>::SetFriction(float friction) requires (!ForceConst) { b2Shape_SetFriction(static_cast<const D &>(*this).Handle(), friction); }
    template <typename D, bool ForceConst> void BasicShapeInterface<D, ForceConst>::SetCircle(const b2Circle& circle) requires (!ForceConst) { b2Shape_SetCircle(static_cast<const D &>(*this).Handle(), &circle); }
    template <typename D, bool ForceConst> ChainRef BasicShapeInterface<D, ForceConst>::GetParentChain() const { return b2Shape_GetParentChain(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicShapeInterface<D, ForceConst>::SetRestitution(float restitution) requires (!ForceConst) { b2Shape_SetRestitution(static_cast<const D &>(*this).Handle(), restitution); }
    template <typename D, bool ForceConst> void BasicShapeInterface<D, ForceConst>::SetUserData(void* userData) requires (!ForceConst) { b2Shape_SetUserData(static_cast<const D &>(*this).Handle(), userData); }
    template <typename D, bool ForceConst> b2ShapeType BasicShapeInterface<D, ForceConst>::GetType() const { return b2Shape_GetType(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> float BasicShapeInterface<D, ForceConst>::GetFriction() const { return b2Shape_GetFriction(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicShapeInterface<D, ForceConst>::SetSegment(const b2Segment& segment) requires (!ForceConst) { b2Shape_SetSegment(static_cast<const D &>(*this).Handle(), &segment); }
    template <typename D, bool ForceConst> bool BasicShapeInterface<D, ForceConst>::IsValid() const { return b2Shape_IsValid(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicShapeInterface<D, ForceConst>::EnableSensorEvents(bool flag) requires (!ForceConst) { b2Shape_EnableSensorEvents(static_cast<const D &>(*this).Handle(), flag); }
    template <typename D, bool ForceConst> void BasicJointInterface<D, ForceConst>::Destroy() requires (!ForceConst) { if (*this) { b2DestroyJoint(static_cast<const D &>(*this).Handle()); static_cast<D &>(*this).id = b2_nullJointId; } }
    template <typename D, bool ForceConst> b2Vec2 BasicJointInterface<D, ForceConst>::GetLocalAnchorA() const { return b2Joint_GetLocalAnchorA(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> b2Vec2 BasicJointInterface<D, ForceConst>::GetLocalAnchorB() const { return b2Joint_GetLocalAnchorB(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicJointInterface<D, ForceConst>::WakeBodies() requires (!ForceConst) { b2Joint_WakeBodies(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicJointInterface<D, ForceConst>::SetUserData(void* userData) requires (!ForceConst) { b2Joint_SetUserData(static_cast<const D &>(*this).Handle(), userData); }
    template <typename D, bool ForceConst> void* BasicJointInterface<D, ForceConst>::GetUserData() const { return b2Joint_GetUserData(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> BodyRef BasicJointInterface<D, ForceConst>::GetBodyA() const { return b2Joint_GetBodyA(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> BodyRef BasicJointInterface<D, ForceConst>::GetBodyB() const { return b2Joint_GetBodyB(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicJointInterface<D, ForceConst>::SetCollideConnected(bool shouldCollide) requires (!ForceConst) { b2Joint_SetCollideConnected(static_cast<const D &>(*this).Handle(), shouldCollide); }
    template <typename D, bool ForceConst> bool BasicJointInterface<D, ForceConst>::GetCollideConnected() const { return b2Joint_GetCollideConnected(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> bool BasicJointInterface<D, ForceConst>::IsValid() const { return b2Joint_IsValid(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> b2JointType BasicJointInterface<D, ForceConst>::GetType() const { return b2Joint_GetType(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicDistanceJointInterface<D, ForceConst>::SetTuning(float hertz, float dampingRatio) requires (!ForceConst) { b2DistanceJoint_SetTuning(static_cast<const D &>(*this).Handle(), hertz, dampingRatio); }
    template <typename D, bool ForceConst> float BasicDistanceJointInterface<D, ForceConst>::GetCurrentLength() const { return b2DistanceJoint_GetCurrentLength(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> float BasicDistanceJointInterface<D, ForceConst>::GetMinLength() const { return b2DistanceJoint_GetMinLength(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> float BasicDistanceJointInterface<D, ForceConst>::GetConstraintForce(float timeStep) const { return b2DistanceJoint_GetConstraintForce(static_cast<const D &>(*this).Handle(), timeStep); }
    template <typename D, bool ForceConst> void BasicDistanceJointInterface<D, ForceConst>::SetLengthRange(float minLength, float maxLength) requires (!ForceConst) { b2DistanceJoint_SetLengthRange(static_cast<const D &>(*this).Handle(), minLength, maxLength); }
    template <typename D, bool ForceConst> void BasicDistanceJointInterface<D, ForceConst>::SetLength(float length) requires (!ForceConst) { b2DistanceJoint_SetLength(static_cast<const D &>(*this).Handle(), length); }
    template <typename D, bool ForceConst> float BasicDistanceJointInterface<D, ForceConst>::GetHertz() const { return b2DistanceJoint_GetHertz(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> float BasicDistanceJointInterface<D, ForceConst>::GetMaxLength() const { return b2DistanceJoint_GetMaxLength(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> float BasicDistanceJointInterface<D, ForceConst>::GetDampingRatio() const { return b2DistanceJoint_GetDampingRatio(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> float BasicDistanceJointInterface<D, ForceConst>::GetLength() const { return b2DistanceJoint_GetLength(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> b2Vec2 BasicMotorJointInterface<D, ForceConst>::GetConstraintForce() const { return b2MotorJoint_GetConstraintForce(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> float BasicMotorJointInterface<D, ForceConst>::GetConstraintTorque() const { return b2MotorJoint_GetConstraintTorque(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> float BasicMotorJointInterface<D, ForceConst>::GetAngularOffset() const { return b2MotorJoint_GetAngularOffset(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicMotorJointInterface<D, ForceConst>::SetMaxForce(float maxForce) requires (!ForceConst) { b2MotorJoint_SetMaxForce(static_cast<const D &>(*this).Handle(), maxForce); }
    template <typename D, bool ForceConst> float BasicMotorJointInterface<D, ForceConst>::GetMaxForce() const { return b2MotorJoint_GetMaxForce(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicMotorJointInterface<D, ForceConst>::SetLinearOffset(b2Vec2 linearOffset) requires (!ForceConst) { b2MotorJoint_SetLinearOffset(static_cast<const D &>(*this).Handle(), linearOffset); }
    template <typename D, bool ForceConst> void BasicMotorJointInterface<D, ForceConst>::SetCorrectionFactor(float correctionFactor) requires (!ForceConst) { b2MotorJoint_SetCorrectionFactor(static_cast<const D &>(*this).Handle(), correctionFactor); }
    template <typename D, bool ForceConst> float BasicMotorJointInterface<D, ForceConst>::GetCorrectionFactor() const { return b2MotorJoint_GetCorrectionFactor(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicMotorJointInterface<D, ForceConst>::SetMaxTorque(float maxTorque) requires (!ForceConst) { b2MotorJoint_SetMaxTorque(static_cast<const D &>(*this).Handle(), maxTorque); }
    template <typename D, bool ForceConst> b2Vec2 BasicMotorJointInterface<D, ForceConst>::GetLinearOffset() const { return b2MotorJoint_GetLinearOffset(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicMotorJointInterface<D, ForceConst>::SetAngularOffset(float angularOffset) requires (!ForceConst) { b2MotorJoint_SetAngularOffset(static_cast<const D &>(*this).Handle(), angularOffset); }
    template <typename D, bool ForceConst> float BasicMotorJointInterface<D, ForceConst>::GetMaxTorque() const { return b2MotorJoint_GetMaxTorque(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> b2Vec2 BasicMouseJointInterface<D, ForceConst>::GetTarget() const { return b2MouseJoint_GetTarget(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> float BasicMouseJointInterface<D, ForceConst>::GetHertz() const { return b2MouseJoint_GetHertz(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> float BasicMouseJointInterface<D, ForceConst>::GetDampingRatio() const { return b2MouseJoint_GetDampingRatio(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicMouseJointInterface<D, ForceConst>::SetTarget(b2Vec2 target) requires (!ForceConst) { b2MouseJoint_SetTarget(static_cast<const D &>(*this).Handle(), target); }
    template <typename D, bool ForceConst> void BasicMouseJointInterface<D, ForceConst>::SetTuning(float hertz, float dampingRatio) requires (!ForceConst) { b2MouseJoint_SetTuning(static_cast<const D &>(*this).Handle(), hertz, dampingRatio); }
    template <typename D, bool ForceConst> bool BasicPrismaticJointInterface<D, ForceConst>::IsMotorEnabled() const { return b2PrismaticJoint_IsMotorEnabled(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> bool BasicPrismaticJointInterface<D, ForceConst>::IsLimitEnabled() const { return b2PrismaticJoint_IsLimitEnabled(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicPrismaticJointInterface<D, ForceConst>::EnableMotor(bool enableMotor) requires (!ForceConst) { b2PrismaticJoint_EnableMotor(static_cast<const D &>(*this).Handle(), enableMotor); }
    template <typename D, bool ForceConst> void BasicPrismaticJointInterface<D, ForceConst>::EnableLimit(bool enableLimit) requires (!ForceConst) { b2PrismaticJoint_EnableLimit(static_cast<const D &>(*this).Handle(), enableLimit); }
    template <typename D, bool ForceConst> float BasicPrismaticJointInterface<D, ForceConst>::GetMaxMotorForce() const { return b2PrismaticJoint_GetMaxMotorForce(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> float BasicPrismaticJointInterface<D, ForceConst>::GetUpperLimit() const { return b2PrismaticJoint_GetUpperLimit(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicPrismaticJointInterface<D, ForceConst>::SetMotorSpeed(float motorSpeed) requires (!ForceConst) { b2PrismaticJoint_SetMotorSpeed(static_cast<const D &>(*this).Handle(), motorSpeed); }
    template <typename D, bool ForceConst> void BasicPrismaticJointInterface<D, ForceConst>::SetLimits(float lower, float upper) requires (!ForceConst) { b2PrismaticJoint_SetLimits(static_cast<const D &>(*this).Handle(), lower, upper); }
    template <typename D, bool ForceConst> float BasicPrismaticJointInterface<D, ForceConst>::GetMotorSpeed() const { return b2PrismaticJoint_GetMotorSpeed(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> float BasicPrismaticJointInterface<D, ForceConst>::GetLowerLimit() const { return b2PrismaticJoint_GetLowerLimit(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> float BasicPrismaticJointInterface<D, ForceConst>::GetConstraintTorque() const { return b2PrismaticJoint_GetConstraintTorque(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> b2Vec2 BasicPrismaticJointInterface<D, ForceConst>::GetConstraintForce() const { return b2PrismaticJoint_GetConstraintForce(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> float BasicPrismaticJointInterface<D, ForceConst>::GetMotorForce() const { return b2PrismaticJoint_GetMotorForce(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicPrismaticJointInterface<D, ForceConst>::SetMaxMotorForce(float force) requires (!ForceConst) { b2PrismaticJoint_SetMaxMotorForce(static_cast<const D &>(*this).Handle(), force); }
    template <typename D, bool ForceConst> void BasicRevoluteJointInterface<D, ForceConst>::SetMaxMotorTorque(float torque) requires (!ForceConst) { b2RevoluteJoint_SetMaxMotorTorque(static_cast<const D &>(*this).Handle(), torque); }
    template <typename D, bool ForceConst> float BasicRevoluteJointInterface<D, ForceConst>::GetUpperLimit() const { return b2RevoluteJoint_GetUpperLimit(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> float BasicRevoluteJointInterface<D, ForceConst>::GetMotorSpeed() const { return b2RevoluteJoint_GetMotorSpeed(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicRevoluteJointInterface<D, ForceConst>::SetMotorSpeed(float motorSpeed) requires (!ForceConst) { b2RevoluteJoint_SetMotorSpeed(static_cast<const D &>(*this).Handle(), motorSpeed); }
    template <typename D, bool ForceConst> float BasicRevoluteJointInterface<D, ForceConst>::GetLowerLimit() const { return b2RevoluteJoint_GetLowerLimit(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicRevoluteJointInterface<D, ForceConst>::SetLimits(float lower, float upper) requires (!ForceConst) { b2RevoluteJoint_SetLimits(static_cast<const D &>(*this).Handle(), lower, upper); }
    template <typename D, bool ForceConst> float BasicRevoluteJointInterface<D, ForceConst>::GetMotorTorque() const { return b2RevoluteJoint_GetMotorTorque(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> float BasicRevoluteJointInterface<D, ForceConst>::GetConstraintTorque() const { return b2RevoluteJoint_GetConstraintTorque(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> b2Vec2 BasicRevoluteJointInterface<D, ForceConst>::GetConstraintForce() const { return b2RevoluteJoint_GetConstraintForce(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> bool BasicRevoluteJointInterface<D, ForceConst>::IsMotorEnabled() const { return b2RevoluteJoint_IsMotorEnabled(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> bool BasicRevoluteJointInterface<D, ForceConst>::IsLimitEnabled() const { return b2RevoluteJoint_IsLimitEnabled(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicRevoluteJointInterface<D, ForceConst>::EnableMotor(bool enableMotor) requires (!ForceConst) { b2RevoluteJoint_EnableMotor(static_cast<const D &>(*this).Handle(), enableMotor); }
    template <typename D, bool ForceConst> void BasicRevoluteJointInterface<D, ForceConst>::EnableLimit(bool enableLimit) requires (!ForceConst) { b2RevoluteJoint_EnableLimit(static_cast<const D &>(*this).Handle(), enableLimit); }
    template <typename D, bool ForceConst> float BasicRevoluteJointInterface<D, ForceConst>::GetMaxMotorTorque() const { return b2RevoluteJoint_GetMaxMotorTorque(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicWheelJointInterface<D, ForceConst>::SetSpringDampingRatio(float dampingRatio) requires (!ForceConst) { b2WheelJoint_SetSpringDampingRatio(static_cast<const D &>(*this).Handle(), dampingRatio); }
    template <typename D, bool ForceConst> float BasicWheelJointInterface<D, ForceConst>::GetUpperLimit() const { return b2WheelJoint_GetUpperLimit(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> float BasicWheelJointInterface<D, ForceConst>::GetConstraintTorque() const { return b2WheelJoint_GetConstraintTorque(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicWheelJointInterface<D, ForceConst>::SetSpringHertz(float hertz) requires (!ForceConst) { b2WheelJoint_SetSpringHertz(static_cast<const D &>(*this).Handle(), hertz); }
    template <typename D, bool ForceConst> float BasicWheelJointInterface<D, ForceConst>::GetMotorSpeed() const { return b2WheelJoint_GetMotorSpeed(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> float BasicWheelJointInterface<D, ForceConst>::GetMotorTorque() const { return b2WheelJoint_GetMotorTorque(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicWheelJointInterface<D, ForceConst>::SetMotorSpeed(float motorSpeed) requires (!ForceConst) { b2WheelJoint_SetMotorSpeed(static_cast<const D &>(*this).Handle(), motorSpeed); }
    template <typename D, bool ForceConst> float BasicWheelJointInterface<D, ForceConst>::GetLowerLimit() const { return b2WheelJoint_GetLowerLimit(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicWheelJointInterface<D, ForceConst>::SetMaxMotorTorque(float torque) requires (!ForceConst) { b2WheelJoint_SetMaxMotorTorque(static_cast<const D &>(*this).Handle(), torque); }
    template <typename D, bool ForceConst> bool BasicWheelJointInterface<D, ForceConst>::IsMotorEnabled() const { return b2WheelJoint_IsMotorEnabled(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> bool BasicWheelJointInterface<D, ForceConst>::IsLimitEnabled() const { return b2WheelJoint_IsLimitEnabled(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> float BasicWheelJointInterface<D, ForceConst>::GetSpringDampingRatio() const { return b2WheelJoint_GetSpringDampingRatio(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicWheelJointInterface<D, ForceConst>::EnableLimit(bool enableLimit) requires (!ForceConst) { b2WheelJoint_EnableLimit(static_cast<const D &>(*this).Handle(), enableLimit); }
    template <typename D, bool ForceConst> void BasicWheelJointInterface<D, ForceConst>::SetLimits(float lower, float upper) requires (!ForceConst) { b2WheelJoint_SetLimits(static_cast<const D &>(*this).Handle(), lower, upper); }
    template <typename D, bool ForceConst> b2Vec2 BasicWheelJointInterface<D, ForceConst>::GetConstraintForce() const { return b2WheelJoint_GetConstraintForce(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicWheelJointInterface<D, ForceConst>::EnableMotor(bool enableMotor) requires (!ForceConst) { b2WheelJoint_EnableMotor(static_cast<const D &>(*this).Handle(), enableMotor); }
    template <typename D, bool ForceConst> float BasicWheelJointInterface<D, ForceConst>::GetSpringHertz() const { return b2WheelJoint_GetSpringHertz(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> float BasicWheelJointInterface<D, ForceConst>::GetMaxMotorTorque() const { return b2WheelJoint_GetMaxMotorTorque(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicWeldJointInterface<D, ForceConst>::SetAngularHertz(float hertz) requires (!ForceConst) { b2WeldJoint_SetAngularHertz(static_cast<const D &>(*this).Handle(), hertz); }
    template <typename D, bool ForceConst> void BasicWeldJointInterface<D, ForceConst>::SetAngularDampingRatio(float dampingRatio) requires (!ForceConst) { b2WeldJoint_SetAngularDampingRatio(static_cast<const D &>(*this).Handle(), dampingRatio); }
    template <typename D, bool ForceConst> float BasicWeldJointInterface<D, ForceConst>::GetAngularHertz() const { return b2WeldJoint_GetAngularHertz(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> float BasicWeldJointInterface<D, ForceConst>::GetLinearHertz() const { return b2WeldJoint_GetLinearHertz(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> float BasicWeldJointInterface<D, ForceConst>::GetAngularDampingRatio() const { return b2WeldJoint_GetAngularDampingRatio(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> float BasicWeldJointInterface<D, ForceConst>::GetLinearDampingRatio() const { return b2WeldJoint_GetLinearDampingRatio(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicWeldJointInterface<D, ForceConst>::SetLinearHertz(float hertz) requires (!ForceConst) { b2WeldJoint_SetLinearHertz(static_cast<const D &>(*this).Handle(), hertz); }
    template <typename D, bool ForceConst> void BasicWeldJointInterface<D, ForceConst>::SetLinearDampingRatio(float dampingRatio) requires (!ForceConst) { b2WeldJoint_SetLinearDampingRatio(static_cast<const D &>(*this).Handle(), dampingRatio); }
    template <typename D, bool ForceConst> Shape BasicBodyInterface<D, ForceConst>::CreatePolygonShape(Tags::OwningHandle, const std::derived_from<b2ShapeDef> auto& def, const b2Polygon& polygon) requires (!ForceConst) { Shape ret; ret.id = b2CreatePolygonShape(static_cast<const D &>(*this).Handle(), &def, &polygon); return ret; }
    template <typename D, bool ForceConst> ShapeRef BasicBodyInterface<D, ForceConst>::CreatePolygonShape(Tags::DestroyWithParent, const std::derived_from<b2ShapeDef> auto& def, const b2Polygon& polygon) requires (!ForceConst) { return b2CreatePolygonShape(static_cast<const D &>(*this).Handle(), &def, &polygon); }
    template <typename D, bool ForceConst> Shape BasicBodyInterface<D, ForceConst>::CreateCircleShape(Tags::OwningHandle, const std::derived_from<b2ShapeDef> auto& def, const b2Circle& circle) requires (!ForceConst) { Shape ret; ret.id = b2CreateCircleShape(static_cast<const D &>(*this).Handle(), &def, &circle); return ret; }
    template <typename D, bool ForceConst> ShapeRef BasicBodyInterface<D, ForceConst>::CreateCircleShape(Tags::DestroyWithParent, const std::derived_from<b2ShapeDef> auto& def, const b2Circle& circle) requires (!ForceConst) { return b2CreateCircleShape(static_cast<const D &>(*this).Handle(), &def, &circle); }
    template <typename D, bool ForceConst> Shape BasicBodyInterface<D, ForceConst>::CreateSegmentShape(Tags::OwningHandle, const std::derived_from<b2ShapeDef> auto& def, const b2Segment& segment) requires (!ForceConst) { Shape ret; ret.id = b2CreateSegmentShape(static_cast<const D &>(*this).Handle(), &def, &segment); return ret; }
    template <typename D, bool ForceConst> ShapeRef BasicBodyInterface<D, ForceConst>::CreateSegmentShape(Tags::DestroyWithParent, const std::derived_from<b2ShapeDef> auto& def, const b2Segment& segment) requires (!ForceConst) { return b2CreateSegmentShape(static_cast<const D &>(*this).Handle(), &def, &segment); }
    template <typename D, bool ForceConst> Shape BasicBodyInterface<D, ForceConst>::CreateCapsuleShape(Tags::OwningHandle, const std::derived_from<b2ShapeDef> auto& def, const b2Capsule& capsule) requires (!ForceConst) { Shape ret; ret.id = b2CreateCapsuleShape(static_cast<const D &>(*this).Handle(), &def, &capsule); return ret; }
    template <typename D, bool ForceConst> ShapeRef BasicBodyInterface<D, ForceConst>::CreateCapsuleShape(Tags::DestroyWithParent, const std::derived_from<b2ShapeDef> auto& def, const b2Capsule& capsule) requires (!ForceConst) { return b2CreateCapsuleShape(static_cast<const D &>(*this).Handle(), &def, &capsule); }
    template <typename D, bool ForceConst> Chain BasicBodyInterface<D, ForceConst>::CreateChain(Tags::OwningHandle, const std::derived_from<b2ChainDef> auto& def) requires (!ForceConst) { Chain ret; ret.id = b2CreateChain(static_cast<const D &>(*this).Handle(), &def); return ret; }
    template <typename D, bool ForceConst> ChainRef BasicBodyInterface<D, ForceConst>::CreateChain(Tags::DestroyWithParent, const std::derived_from<b2ChainDef> auto& def) requires (!ForceConst) { return b2CreateChain(static_cast<const D &>(*this).Handle(), &def); }
    template <typename D, bool ForceConst> void BasicBodyInterface<D, ForceConst>::Destroy() requires (!ForceConst) { if (*this) { b2DestroyBody(static_cast<const D &>(*this).Handle()); static_cast<D &>(*this).id = b2_nullBodyId; } }
    template <typename D, bool ForceConst> float BasicBodyInterface<D, ForceConst>::GetInertiaTensor() const { return b2Body_GetInertiaTensor(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicBodyInterface<D, ForceConst>::ApplyLinearImpulse(b2Vec2 impulse, b2Vec2 point, bool wake) requires (!ForceConst) { b2Body_ApplyLinearImpulse(static_cast<const D &>(*this).Handle(), impulse, point, wake); }
    template <typename D, bool ForceConst> b2Vec2 BasicBodyInterface<D, ForceConst>::GetWorldPoint(b2Vec2 localPoint) const { return b2Body_GetWorldPoint(static_cast<const D &>(*this).Handle(), localPoint); }
    template <typename D, bool ForceConst> bool BasicBodyInterface<D, ForceConst>::IsBullet() const { return b2Body_IsBullet(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicBodyInterface<D, ForceConst>::SetGravityScale(float gravityScale) requires (!ForceConst) { b2Body_SetGravityScale(static_cast<const D &>(*this).Handle(), gravityScale); }
    template <typename D, bool ForceConst> int BasicBodyInterface<D, ForceConst>::GetJointCount() const { return b2Body_GetJointCount(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> int BasicBodyInterface<D, ForceConst>::GetShapes(b2ShapeId& shapeArray, int capacity) const { return b2Body_GetShapes(static_cast<const D &>(*this).Handle(), &shapeArray, capacity); }
    template <typename D, bool ForceConst> void BasicBodyInterface<D, ForceConst>::SetType(b2BodyType type) requires (!ForceConst) { b2Body_SetType(static_cast<const D &>(*this).Handle(), type); }
    template <typename D, bool ForceConst> int BasicBodyInterface<D, ForceConst>::GetContactCapacity() const { return b2Body_GetContactCapacity(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> float BasicBodyInterface<D, ForceConst>::GetGravityScale() const { return b2Body_GetGravityScale(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> b2Vec2 BasicBodyInterface<D, ForceConst>::GetLocalCenterOfMass() const { return b2Body_GetLocalCenterOfMass(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> b2Vec2 BasicBodyInterface<D, ForceConst>::GetWorldCenterOfMass() const { return b2Body_GetWorldCenterOfMass(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicBodyInterface<D, ForceConst>::SetFixedRotation(bool flag) requires (!ForceConst) { b2Body_SetFixedRotation(static_cast<const D &>(*this).Handle(), flag); }
    template <typename D, bool ForceConst> void BasicBodyInterface<D, ForceConst>::SetAngularDamping(float angularDamping) requires (!ForceConst) { b2Body_SetAngularDamping(static_cast<const D &>(*this).Handle(), angularDamping); }
    template <typename D, bool ForceConst> void BasicBodyInterface<D, ForceConst>::ApplyAngularImpulse(float impulse, bool wake) requires (!ForceConst) { b2Body_ApplyAngularImpulse(static_cast<const D &>(*this).Handle(), impulse, wake); }
    template <typename D, bool ForceConst> b2Vec2 BasicBodyInterface<D, ForceConst>::GetLocalVector(b2Vec2 worldVector) const { return b2Body_GetLocalVector(static_cast<const D &>(*this).Handle(), worldVector); }
    template <typename D, bool ForceConst> void BasicBodyInterface<D, ForceConst>::SetMassData(b2MassData massData) requires (!ForceConst) { b2Body_SetMassData(static_cast<const D &>(*this).Handle(), massData); }
    template <typename D, bool ForceConst> int BasicBodyInterface<D, ForceConst>::GetShapeCount() const { return b2Body_GetShapeCount(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicBodyInterface<D, ForceConst>::SetLinearDamping(float linearDamping) requires (!ForceConst) { b2Body_SetLinearDamping(static_cast<const D &>(*this).Handle(), linearDamping); }
    template <typename D, bool ForceConst> void BasicBodyInterface<D, ForceConst>::Disable() requires (!ForceConst) { b2Body_Disable(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicBodyInterface<D, ForceConst>::SetLinearVelocity(b2Vec2 linearVelocity) requires (!ForceConst) { b2Body_SetLinearVelocity(static_cast<const D &>(*this).Handle(), linearVelocity); }
    template <typename D, bool ForceConst> b2Vec2 BasicBodyInterface<D, ForceConst>::GetLocalPoint(b2Vec2 worldPoint) const { return b2Body_GetLocalPoint(static_cast<const D &>(*this).Handle(), worldPoint); }
    template <typename D, bool ForceConst> b2MassData BasicBodyInterface<D, ForceConst>::GetMassData() const { return b2Body_GetMassData(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicBodyInterface<D, ForceConst>::SetTransform(b2Vec2 position, float angle) requires (!ForceConst) { b2Body_SetTransform(static_cast<const D &>(*this).Handle(), position, angle); }
    template <typename D, bool ForceConst> float BasicBodyInterface<D, ForceConst>::GetAngle() const { return b2Body_GetAngle(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicBodyInterface<D, ForceConst>::SetAngularVelocity(float angularVelocity) requires (!ForceConst) { b2Body_SetAngularVelocity(static_cast<const D &>(*this).Handle(), angularVelocity); }
    template <typename D, bool ForceConst> int BasicBodyInterface<D, ForceConst>::GetJoints(b2JointId& jointArray, int capacity) const { return b2Body_GetJoints(static_cast<const D &>(*this).Handle(), &jointArray, capacity); }
    template <typename D, bool ForceConst> void BasicBodyInterface<D, ForceConst>::EnableSleep(bool enableSleep) requires (!ForceConst) { b2Body_EnableSleep(static_cast<const D &>(*this).Handle(), enableSleep); }
    template <typename D, bool ForceConst> void BasicBodyInterface<D, ForceConst>::ApplyForce(b2Vec2 force, b2Vec2 point, bool wake) requires (!ForceConst) { b2Body_ApplyForce(static_cast<const D &>(*this).Handle(), force, point, wake); }
    template <typename D, bool ForceConst> float BasicBodyInterface<D, ForceConst>::GetAngularDamping() const { return b2Body_GetAngularDamping(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> bool BasicBodyInterface<D, ForceConst>::IsFixedRotation() const { return b2Body_IsFixedRotation(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicBodyInterface<D, ForceConst>::ResetMassData() requires (!ForceConst) { b2Body_ResetMassData(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicBodyInterface<D, ForceConst>::SetUserData(void* userData) requires (!ForceConst) { b2Body_SetUserData(static_cast<const D &>(*this).Handle(), userData); }
    template <typename D, bool ForceConst> b2BodyType BasicBodyInterface<D, ForceConst>::GetType() const { return b2Body_GetType(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> bool BasicBodyInterface<D, ForceConst>::IsSleepEnabled() const { return b2Body_IsSleepEnabled(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> float BasicBodyInterface<D, ForceConst>::GetAngularVelocity() const { return b2Body_GetAngularVelocity(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> bool BasicBodyInterface<D, ForceConst>::IsValid() const { return b2Body_IsValid(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicBodyInterface<D, ForceConst>::Enable() requires (!ForceConst) { b2Body_Enable(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicBodyInterface<D, ForceConst>::ApplyLinearImpulseToCenter(b2Vec2 impulse, bool wake) requires (!ForceConst) { b2Body_ApplyLinearImpulseToCenter(static_cast<const D &>(*this).Handle(), impulse, wake); }
    template <typename D, bool ForceConst> void BasicBodyInterface<D, ForceConst>::SetAwake(bool awake) requires (!ForceConst) { b2Body_SetAwake(static_cast<const D &>(*this).Handle(), awake); }
    template <typename D, bool ForceConst> void* BasicBodyInterface<D, ForceConst>::GetUserData() const { return b2Body_GetUserData(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicBodyInterface<D, ForceConst>::ApplyTorque(float torque, bool wake) requires (!ForceConst) { b2Body_ApplyTorque(static_cast<const D &>(*this).Handle(), torque, wake); }
    template <typename D, bool ForceConst> b2Rot BasicBodyInterface<D, ForceConst>::GetRotation() const { return b2Body_GetRotation(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> b2AABB BasicBodyInterface<D, ForceConst>::ComputeAABB() const { return b2Body_ComputeAABB(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> b2Vec2 BasicBodyInterface<D, ForceConst>::GetLinearVelocity() const { return b2Body_GetLinearVelocity(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> int BasicBodyInterface<D, ForceConst>::GetContactData(b2ContactData& contactData, int capacity) const { return b2Body_GetContactData(static_cast<const D &>(*this).Handle(), &contactData, capacity); }
    template <typename D, bool ForceConst> bool BasicBodyInterface<D, ForceConst>::IsEnabled() const { return b2Body_IsEnabled(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicBodyInterface<D, ForceConst>::ApplyForceToCenter(b2Vec2 force, bool wake) requires (!ForceConst) { b2Body_ApplyForceToCenter(static_cast<const D &>(*this).Handle(), force, wake); }
    template <typename D, bool ForceConst> bool BasicBodyInterface<D, ForceConst>::IsAwake() const { return b2Body_IsAwake(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> float BasicBodyInterface<D, ForceConst>::GetMass() const { return b2Body_GetMass(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> b2Vec2 BasicBodyInterface<D, ForceConst>::GetWorldVector(b2Vec2 localVector) const { return b2Body_GetWorldVector(static_cast<const D &>(*this).Handle(), localVector); }
    template <typename D, bool ForceConst> void BasicBodyInterface<D, ForceConst>::SetBullet(bool flag) requires (!ForceConst) { b2Body_SetBullet(static_cast<const D &>(*this).Handle(), flag); }
    template <typename D, bool ForceConst> b2Vec2 BasicBodyInterface<D, ForceConst>::GetPosition() const { return b2Body_GetPosition(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> b2Transform BasicBodyInterface<D, ForceConst>::GetTransform() const { return b2Body_GetTransform(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> float BasicBodyInterface<D, ForceConst>::GetLinearDamping() const { return b2Body_GetLinearDamping(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> WeldJoint BasicWorldInterface<D, ForceConst>::CreateWeldJoint(Tags::OwningHandle, const std::derived_from<b2WeldJointDef> auto& def) requires (!ForceConst) { WeldJoint ret; ret.id = b2CreateWeldJoint(static_cast<const D &>(*this).Handle(), &def); return ret; }
    template <typename D, bool ForceConst> WeldJointRef BasicWorldInterface<D, ForceConst>::CreateWeldJoint(Tags::DestroyWithParent, const std::derived_from<b2WeldJointDef> auto& def) requires (!ForceConst) { return (WeldJointRef)b2CreateWeldJoint(static_cast<const D &>(*this).Handle(), &def); }
    template <typename D, bool ForceConst> Body BasicWorldInterface<D, ForceConst>::CreateBody(Tags::OwningHandle, const std::derived_from<b2BodyDef> auto& def) requires (!ForceConst) { Body ret; ret.id = b2CreateBody(static_cast<const D &>(*this).Handle(), &def); return ret; }
    template <typename D, bool ForceConst> BodyRef BasicWorldInterface<D, ForceConst>::CreateBody(Tags::DestroyWithParent, const std::derived_from<b2BodyDef> auto& def) requires (!ForceConst) { return b2CreateBody(static_cast<const D &>(*this).Handle(), &def); }
    template <typename D, bool ForceConst> DistanceJoint BasicWorldInterface<D, ForceConst>::CreateDistanceJoint(Tags::OwningHandle, const std::derived_from<b2DistanceJointDef> auto& def) requires (!ForceConst) { DistanceJoint ret; ret.id = b2CreateDistanceJoint(static_cast<const D &>(*this).Handle(), &def); return ret; }
    template <typename D, bool ForceConst> DistanceJointRef BasicWorldInterface<D, ForceConst>::CreateDistanceJoint(Tags::DestroyWithParent, const std::derived_from<b2DistanceJointDef> auto& def) requires (!ForceConst) { return (DistanceJointRef)b2CreateDistanceJoint(static_cast<const D &>(*this).Handle(), &def); }
    template <typename D, bool ForceConst> MouseJoint BasicWorldInterface<D, ForceConst>::CreateMouseJoint(Tags::OwningHandle, const std::derived_from<b2MouseJointDef> auto& def) requires (!ForceConst) { MouseJoint ret; ret.id = b2CreateMouseJoint(static_cast<const D &>(*this).Handle(), &def); return ret; }
    template <typename D, bool ForceConst> MouseJointRef BasicWorldInterface<D, ForceConst>::CreateMouseJoint(Tags::DestroyWithParent, const std::derived_from<b2MouseJointDef> auto& def) requires (!ForceConst) { return (MouseJointRef)b2CreateMouseJoint(static_cast<const D &>(*this).Handle(), &def); }
    template <typename D, bool ForceConst> WheelJoint BasicWorldInterface<D, ForceConst>::CreateWheelJoint(Tags::OwningHandle, const std::derived_from<b2WheelJointDef> auto& def) requires (!ForceConst) { WheelJoint ret; ret.id = b2CreateWheelJoint(static_cast<const D &>(*this).Handle(), &def); return ret; }
    template <typename D, bool ForceConst> WheelJointRef BasicWorldInterface<D, ForceConst>::CreateWheelJoint(Tags::DestroyWithParent, const std::derived_from<b2WheelJointDef> auto& def) requires (!ForceConst) { return (WheelJointRef)b2CreateWheelJoint(static_cast<const D &>(*this).Handle(), &def); }
    template <typename D, bool ForceConst> MotorJoint BasicWorldInterface<D, ForceConst>::CreateMotorJoint(Tags::OwningHandle, const std::derived_from<b2MotorJointDef> auto& def) requires (!ForceConst) { MotorJoint ret; ret.id = b2CreateMotorJoint(static_cast<const D &>(*this).Handle(), &def); return ret; }
    template <typename D, bool ForceConst> MotorJointRef BasicWorldInterface<D, ForceConst>::CreateMotorJoint(Tags::DestroyWithParent, const std::derived_from<b2MotorJointDef> auto& def) requires (!ForceConst) { return (MotorJointRef)b2CreateMotorJoint(static_cast<const D &>(*this).Handle(), &def); }
    template <typename D, bool ForceConst> RevoluteJoint BasicWorldInterface<D, ForceConst>::CreateRevoluteJoint(Tags::OwningHandle, const std::derived_from<b2RevoluteJointDef> auto& def) requires (!ForceConst) { RevoluteJoint ret; ret.id = b2CreateRevoluteJoint(static_cast<const D &>(*this).Handle(), &def); return ret; }
    template <typename D, bool ForceConst> RevoluteJointRef BasicWorldInterface<D, ForceConst>::CreateRevoluteJoint(Tags::DestroyWithParent, const std::derived_from<b2RevoluteJointDef> auto& def) requires (!ForceConst) { return (RevoluteJointRef)b2CreateRevoluteJoint(static_cast<const D &>(*this).Handle(), &def); }
    template <typename D, bool ForceConst> PrismaticJoint BasicWorldInterface<D, ForceConst>::CreatePrismaticJoint(Tags::OwningHandle, const std::derived_from<b2PrismaticJointDef> auto& def) requires (!ForceConst) { PrismaticJoint ret; ret.id = b2CreatePrismaticJoint(static_cast<const D &>(*this).Handle(), &def); return ret; }
    template <typename D, bool ForceConst> PrismaticJointRef BasicWorldInterface<D, ForceConst>::CreatePrismaticJoint(Tags::DestroyWithParent, const std::derived_from<b2PrismaticJointDef> auto& def) requires (!ForceConst) { return (PrismaticJointRef)b2CreatePrismaticJoint(static_cast<const D &>(*this).Handle(), &def); }
    template <typename D, bool ForceConst> void BasicWorldInterface<D, ForceConst>::Destroy() requires (!ForceConst) { if (*this) { b2DestroyWorld(static_cast<const D &>(*this).Handle()); static_cast<D &>(*this).id = b2_nullWorldId; } }
    template <typename D, bool ForceConst> b2Vec2 BasicWorldInterface<D, ForceConst>::GetGravity() const { return b2World_GetGravity(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicWorldInterface<D, ForceConst>::OverlapCapsule(const b2Capsule& capsule, b2Transform transform, b2QueryFilter filter, b2OverlapResultFcn* fcn, void* context) const { b2World_OverlapCapsule(static_cast<const D &>(*this).Handle(), &capsule, transform, filter, fcn, context); }
    template <typename D, bool ForceConst> b2Counters BasicWorldInterface<D, ForceConst>::GetCounters() const { return b2World_GetCounters(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> b2SensorEvents BasicWorldInterface<D, ForceConst>::GetSensorEvents() const { return b2World_GetSensorEvents(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicWorldInterface<D, ForceConst>::SetPreSolveCallback(b2PreSolveFcn* fcn, void* context) requires (!ForceConst) { b2World_SetPreSolveCallback(static_cast<const D &>(*this).Handle(), fcn, context); }
    template <typename D, bool ForceConst> void BasicWorldInterface<D, ForceConst>::Step(float timeStep, int subStepCount) requires (!ForceConst) { b2World_Step(static_cast<const D &>(*this).Handle(), timeStep, subStepCount); }
    template <typename D, bool ForceConst> b2RayResult BasicWorldInterface<D, ForceConst>::RayCastClosest(b2Vec2 origin, b2Vec2 translation, b2QueryFilter filter) const { return b2World_RayCastClosest(static_cast<const D &>(*this).Handle(), origin, translation, filter); }
    template <typename D, bool ForceConst> void BasicWorldInterface<D, ForceConst>::CapsuleCast(const b2Capsule& capsule, b2Transform originTransform, b2Vec2 translation, b2QueryFilter filter, b2CastResultFcn* fcn, void* context) const { b2World_CapsuleCast(static_cast<const D &>(*this).Handle(), &capsule, originTransform, translation, filter, fcn, context); }
    template <typename D, bool ForceConst> void BasicWorldInterface<D, ForceConst>::SetRestitutionThreshold(float value) requires (!ForceConst) { b2World_SetRestitutionThreshold(static_cast<const D &>(*this).Handle(), value); }
    template <typename D, bool ForceConst> void BasicWorldInterface<D, ForceConst>::PolygonCast(const b2Polygon& polygon, b2Transform originTransform, b2Vec2 translation, b2QueryFilter filter, b2CastResultFcn* fcn, void* context) const { b2World_PolygonCast(static_cast<const D &>(*this).Handle(), &polygon, originTransform, translation, filter, fcn, context); }
    template <typename D, bool ForceConst> void BasicWorldInterface<D, ForceConst>::OverlapPolygon(const b2Polygon& polygon, b2Transform transform, b2QueryFilter filter, b2OverlapResultFcn* fcn, void* context) const { b2World_OverlapPolygon(static_cast<const D &>(*this).Handle(), &polygon, transform, filter, fcn, context); }
    template <typename D, bool ForceConst> void BasicWorldInterface<D, ForceConst>::CircleCast(const b2Circle& circle, b2Transform originTransform, b2Vec2 translation, b2QueryFilter filter, b2CastResultFcn* fcn, void* context) const { b2World_CircleCast(static_cast<const D &>(*this).Handle(), &circle, originTransform, translation, filter, fcn, context); }
    template <typename D, bool ForceConst> void BasicWorldInterface<D, ForceConst>::OverlapCircle(const b2Circle& circle, b2Transform transform, b2QueryFilter filter, b2OverlapResultFcn* fcn, void* context) const { b2World_OverlapCircle(static_cast<const D &>(*this).Handle(), &circle, transform, filter, fcn, context); }
    template <typename D, bool ForceConst> void BasicWorldInterface<D, ForceConst>::SetContactTuning(float hertz, float dampingRatio, float pushVelocity) requires (!ForceConst) { b2World_SetContactTuning(static_cast<const D &>(*this).Handle(), hertz, dampingRatio, pushVelocity); }
    template <typename D, bool ForceConst> b2Profile BasicWorldInterface<D, ForceConst>::GetProfile() const { return b2World_GetProfile(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicWorldInterface<D, ForceConst>::Draw(b2DebugDraw& draw) const { b2World_Draw(static_cast<const D &>(*this).Handle(), &draw); }
    template <typename D, bool ForceConst> void BasicWorldInterface<D, ForceConst>::EnableContinuous(bool flag) requires (!ForceConst) { b2World_EnableContinuous(static_cast<const D &>(*this).Handle(), flag); }
    template <typename D, bool ForceConst> void BasicWorldInterface<D, ForceConst>::EnableSleeping(bool flag) requires (!ForceConst) { b2World_EnableSleeping(static_cast<const D &>(*this).Handle(), flag); }
    template <typename D, bool ForceConst> bool BasicWorldInterface<D, ForceConst>::IsValid() const { return b2World_IsValid(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicWorldInterface<D, ForceConst>::OverlapAABB(b2AABB aabb, b2QueryFilter filter, b2OverlapResultFcn* fcn, void* context) const { b2World_OverlapAABB(static_cast<const D &>(*this).Handle(), aabb, filter, fcn, context); }
    template <typename D, bool ForceConst> b2ContactEvents BasicWorldInterface<D, ForceConst>::GetContactEvents() const { return b2World_GetContactEvents(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> b2BodyEvents BasicWorldInterface<D, ForceConst>::GetBodyEvents() const { return b2World_GetBodyEvents(static_cast<const D &>(*this).Handle()); }
    template <typename D, bool ForceConst> void BasicWorldInterface<D, ForceConst>::RayCast(b2Vec2 origin, b2Vec2 translation, b2QueryFilter filter, b2CastResultFcn* fcn, void* context) const { b2World_RayCast(static_cast<const D &>(*this).Handle(), origin, translation, filter, fcn, context); }
    template <typename D, bool ForceConst> void BasicWorldInterface<D, ForceConst>::EnableWarmStarting(bool flag) requires (!ForceConst) { b2World_EnableWarmStarting(static_cast<const D &>(*this).Handle(), flag); }
    template <typename D, bool ForceConst> void BasicWorldInterface<D, ForceConst>::SetGravity(b2Vec2 gravity) requires (!ForceConst) { b2World_SetGravity(static_cast<const D &>(*this).Handle(), gravity); }
    inline float DynamicTree::GetAreaRatio() const { return b2DynamicTree_GetAreaRatio(&value); }
    inline void DynamicTree::RayCast(const b2RayCastInput& input, uint32_t maskBits, b2TreeRayCastCallbackFcn* callback, void* context) const { b2DynamicTree_RayCast(&value, &input, maskBits, callback, context); }
    inline int32_t DynamicTree::GetHeight() const { return b2DynamicTree_GetHeight(&value); }
    inline void DynamicTree::MoveProxy(int32_t proxyId, b2AABB aabb) { b2DynamicTree_MoveProxy(&value, proxyId, aabb); }
    inline void DynamicTree::ShiftOrigin(b2Vec2 newOrigin) { b2DynamicTree_ShiftOrigin(&value, newOrigin); }
    inline void DynamicTree::Validate() const { b2DynamicTree_Validate(&value); }
    inline b2AABB DynamicTree::GetAABB(int32_t proxyId) const { return b2DynamicTree_GetAABB(&value, proxyId); }
    inline void DynamicTree::RebuildBottomUp() { b2DynamicTree_RebuildBottomUp(&value); }
    inline void DynamicTree::ShapeCast(const b2ShapeCastInput& input, uint32_t maskBits, b2TreeShapeCastCallbackFcn* callback, void* context) const { b2DynamicTree_ShapeCast(&value, &input, maskBits, callback, context); }
    inline int32_t DynamicTree::GetProxyCount() const { return b2DynamicTree_GetProxyCount(&value); }
    inline int32_t DynamicTree::CreateProxy(b2AABB aabb, uint32_t categoryBits, int32_t userData) { return b2DynamicTree_CreateProxy(&value, aabb, categoryBits, userData); }
    inline int32_t DynamicTree::GetUserData(int32_t proxyId) const { return b2DynamicTree_GetUserData(&value, proxyId); }
    inline void DynamicTree::QueryFiltered(b2AABB aabb, uint32_t maskBits, b2TreeQueryCallbackFcn* callback, void* context) const { b2DynamicTree_QueryFiltered(&value, aabb, maskBits, callback, context); }
    inline int32_t DynamicTree::GetMaxBalance() const { return b2DynamicTree_GetMaxBalance(&value); }
    inline void DynamicTree::Query(b2AABB aabb, b2TreeQueryCallbackFcn* callback, void* context) const { b2DynamicTree_Query(&value, aabb, callback, context); }
    inline void DynamicTree::DestroyProxy(int32_t proxyId) { b2DynamicTree_DestroyProxy(&value, proxyId); }
    inline int32_t DynamicTree::Rebuild(bool fullBuild) { return b2DynamicTree_Rebuild(&value, fullBuild); }
    inline void DynamicTree::EnlargeProxy(int32_t proxyId, b2AABB aabb) { b2DynamicTree_EnlargeProxy(&value, proxyId, aabb); }
} // namespace box2d
