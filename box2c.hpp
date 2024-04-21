#pragma once

#include "box2c/include/box2d/box2d.h"
#include "box2c/include/box2d/dynamic_tree.h"
#include "box2c/include/box2d/math.h"

#include <cstddef>
#include <concepts>
#include <stdexcept>
#include <utility>

namespace b2
{
    class World
    {
        b2WorldId id = b2_nullWorldId;

      public:
        constexpr World() {}

        // The constructor accepts either this or directly `b2WorldDef`.
        struct Params : b2WorldDef
        {
            Params() : b2WorldDef(b2DefaultWorldDef()) {}
        };

        /// Create a world for rigid body simulation. This contains all the bodies, shapes, and constraints.
        World(const std::derived_from<b2WorldDef> auto &params) : id(b2CreateWorld(&params)) { if (!*this) throw std::runtime_error("Failed to create a `b2World`."); }

        World(World&& other) noexcept : id(std::exchange(other.id, b2_nullWorldId)) {}
        World& operator=(World other) noexcept { std::swap(id, other.id); return *this; }

        ~World() { if (*this) b2DestroyWorld(id); }

        [[nodiscard]] explicit operator bool() const { return B2_IS_NON_NULL(id); }
        [[nodiscard]] const b2WorldId &Handle() const { return id; }

        /// Overlap test for all shapes that overlap the provided capsule.
        void OverlapCapsule(const b2Capsule& capsule, b2Transform transform, b2QueryFilter filter, b2OverlapResultFcn& fcn, void* context) const { return b2World_OverlapCapsule(Handle(), &capsule, transform, filter, &fcn, context); }

        /// Get counters and sizes
        [[nodiscard]] b2Counters GetCounters() const { return b2World_GetCounters(Handle()); }

        /// Get sensor events for the current time step. The event data is transient. Do not store a reference to this data.
        [[nodiscard]] b2SensorEvents GetSensorEvents() const { return b2World_GetSensorEvents(Handle()); }

        /// Register the pre-solve callback. This is optional.
        void SetPreSolveCallback(b2PreSolveFcn& fcn, void* context) { return b2World_SetPreSolveCallback(Handle(), &fcn, context); }

        /// Take a time step. This performs collision detection, integration,
        /// and constraint solution.
        /// @param timeStep the amount of time to simulate, this should not vary.
        /// @param velocityIterations for the velocity constraint solver.
        /// @param relaxIterations for reducing constraint bounce solver.
        void Step(float timeStep, int32_t subStepCount) { return b2World_Step(Handle(), timeStep, subStepCount); }

        /// Ray-cast closest hit. Convenience function. This is less general than b2World_RayCast and does not allow for custom filtering.
        [[nodiscard]] b2RayResult RayCastClosest(b2Vec2 origin, b2Vec2 translation, b2QueryFilter filter) const { return b2World_RayCastClosest(Handle(), origin, translation, filter); }

        /// Cast a capsule through the world. Similar to a ray-cast except that a capsule is cast instead of a point.
        void CapsuleCast(const b2Capsule& capsule, b2Transform originTransform, b2Vec2 translation, b2QueryFilter filter, b2CastResultFcn& fcn, void* context) const { return b2World_CapsuleCast(Handle(), &capsule, originTransform, translation, filter, &fcn, context); }

        /// Adjust the restitution threshold. Advanced feature for testing.
        void SetRestitutionThreshold(float value) { return b2World_SetRestitutionThreshold(Handle(), value); }

        /// Cast a capsule through the world. Similar to a ray-cast except that a polygon is cast instead of a point.
        void PolygonCast(const b2Polygon& polygon, b2Transform originTransform, b2Vec2 translation, b2QueryFilter filter, b2CastResultFcn& fcn, void* context) const { return b2World_PolygonCast(Handle(), &polygon, originTransform, translation, filter, &fcn, context); }

        /// Overlap test for all shapes that overlap the provided polygon.
        void OverlapPolygon(const b2Polygon& polygon, b2Transform transform, b2QueryFilter filter, b2OverlapResultFcn& fcn, void* context) const { return b2World_OverlapPolygon(Handle(), &polygon, transform, filter, &fcn, context); }

        /// Cast a circle through the world. Similar to a ray-cast except that a circle is cast instead of a point.
        void CircleCast(const b2Circle& circle, b2Transform originTransform, b2Vec2 translation, b2QueryFilter filter, b2CastResultFcn& fcn, void* context) const { return b2World_CircleCast(Handle(), &circle, originTransform, translation, filter, &fcn, context); }

        /// Overlap test for for all shapes that overlap the provided circle.
        void OverlapCircle(const b2Circle& circle, b2Transform transform, b2QueryFilter filter, b2OverlapResultFcn& fcn, void* context) const { return b2World_OverlapCircle(Handle(), &circle, transform, filter, &fcn, context); }

        /// Adjust contact tuning parameters:
        /// - hertz is the contact stiffness (cycles per second)
        /// - damping ratio is the contact bounciness with 1 being critical damping (non-dimensional)
        /// - push velocity is the maximum contact constraint push out velocity (meters per second)
        ///	Advanced feature
        void SetContactTuning(float hertz, float dampingRatio, float pushVelocity) { return b2World_SetContactTuning(Handle(), hertz, dampingRatio, pushVelocity); }

        /// Get the current profile
        [[nodiscard]] b2Profile GetProfile() const { return b2World_GetProfile(Handle()); }

        /// Call this to draw shapes and other debug draw data. This is intentionally non-const.
        void Draw(b2DebugDraw& debugDraw) const { return b2World_Draw(Handle(), &debugDraw); }

        /// Enable/disable continuous collision. Advanced feature for testing.
        void EnableContinuous(bool flag) { return b2World_EnableContinuous(Handle(), flag); }

        /// Enable/disable sleep. Advanced feature for testing.
        void EnableSleeping(bool flag) { return b2World_EnableSleeping(Handle(), flag); }

        /// World identifier validation. Provides validation for up to 64K allocations.
        [[nodiscard]] bool IsValid() const { return b2World_IsValid(Handle()); }

        /// Overlap test for all shapes that *potentially* overlap the provided AABB.
        void OverlapAABB(b2AABB aabb, b2QueryFilter filter, b2OverlapResultFcn& fcn, void* context) const { return b2World_OverlapAABB(Handle(), aabb, filter, &fcn, context); }

        /// Get contact events for this current time step. The event data is transient. Do not store a reference to this data.
        [[nodiscard]] b2ContactEvents GetContactEvents() const { return b2World_GetContactEvents(Handle()); }

        /// Get the body events for the current time step. The event data is transient. Do not store a reference to this data.
        [[nodiscard]] b2BodyEvents GetBodyEvents() const { return b2World_GetBodyEvents(Handle()); }

        /// Ray-cast the world for all shapes in the path of the ray. Your callback
        /// controls whether you get the closest point, any point, or n-points.
        /// The ray-cast ignores shapes that contain the starting point.
        /// @param callback a user implemented callback class.
        /// @param point1 the ray starting point
        /// @param point2 the ray ending point
        void RayCast(b2Vec2 origin, b2Vec2 translation, b2QueryFilter filter, b2CastResultFcn& fcn, void* context) const { return b2World_RayCast(Handle(), origin, translation, filter, &fcn, context); }

        /// Enable/disable constraint warm starting. Advanced feature for testing.
        void EnableWarmStarting(bool flag) { return b2World_EnableWarmStarting(Handle(), flag); }
    };

    class Body
    {
        b2BodyId id = b2_nullBodyId;

      public:
        constexpr Body() {}

        // The constructor accepts either this or directly `b2BodyDef`.
        struct Params : b2BodyDef
        {
            Params() : b2BodyDef(b2DefaultBodyDef()) {}
        };

        /// Create a rigid body given a definition. No reference to the definition is retained.
        /// @warning This function is locked during callbacks.
        Body(World &world, const std::derived_from<b2BodyDef> auto &params) : id(b2CreateBody(world.Handle(), &params)) { if (!*this) throw std::runtime_error("Failed to create a `b2Body`."); }

        Body(Body&& other) noexcept : id(std::exchange(other.id, b2_nullBodyId)) {}
        Body& operator=(Body other) noexcept { std::swap(id, other.id); return *this; }

        ~Body() { if (*this) b2DestroyBody(id); }

        [[nodiscard]] explicit operator bool() const { return B2_IS_NON_NULL(id); }
        [[nodiscard]] const b2BodyId &Handle() const { return id; }

        /// Get the inertia tensor of the body. In 2D this is a single number. (kilograms * meters^2)
        [[nodiscard]] float GetInertiaTensor() const { return b2Body_GetInertiaTensor(Handle()); }

        /// Apply an impulse at a point. This immediately modifies the velocity.
        /// It also modifies the angular velocity if the point of application
        /// is not at the center of mass. This wakes up the body.
        /// This should be used for one-shot impulses. If you need a steady force,
        /// use a force instead, which will work better with the sub-stepping solver.
        /// @param impulse the world impulse vector, usually in N-seconds or kg-m/s.
        /// @param point the world position of the point of application.
        /// @param wake also wake up the body
        void ApplyLinearImpulse(b2Vec2 impulse, b2Vec2 point, bool wake) { return b2Body_ApplyLinearImpulse(Handle(), impulse, point, wake); }

        /// Get a world point on a body given a local point
        [[nodiscard]] b2Vec2 GetWorldPoint(b2Vec2 localPoint) const { return b2Body_GetWorldPoint(Handle(), localPoint); }

        /// Is this body a bullet?
        [[nodiscard]] bool IsBullet() const { return b2Body_IsBullet(Handle()); }

        /// Adjust the gravity scale. Normally this is set in b2BodyDef before creation.
        void SetGravityScale(float gravityScale) { return b2Body_SetGravityScale(Handle(), gravityScale); }

        /// Set the type of a body. This has a similar cost to re-creating the body.
        void SetType(b2BodyType type) { return b2Body_SetType(Handle(), type); }

        /// Get the maximum capacity required for retrieving all the touching contacts on a body
        [[nodiscard]] int32_t GetContactCapacity() const { return b2Body_GetContactCapacity(Handle()); }

        /// Get the current gravity scale.
        [[nodiscard]] float GetGravityScale() const { return b2Body_GetGravityScale(Handle()); }

        /// Get the center of mass position of the body in local space.
        [[nodiscard]] b2Vec2 GetLocalCenterOfMass() const { return b2Body_GetLocalCenterOfMass(Handle()); }

        /// Get the center of mass position of the body in world space.
        [[nodiscard]] b2Vec2 GetWorldCenterOfMass() const { return b2Body_GetWorldCenterOfMass(Handle()); }

        /// Set this body to have fixed rotation. This causes the mass to be reset.
        void SetFixedRotation(bool flag) { return b2Body_SetFixedRotation(Handle(), flag); }

        /// Adjust the angular damping. Normally this is set in b2BodyDef before creation.
        void SetAngularDamping(float angularDamping) { return b2Body_SetAngularDamping(Handle(), angularDamping); }

        /// Apply an angular impulse.
        /// This should be used for one-shot impulses. If you need a steady force,
        /// use a force instead, which will work better with the sub-stepping solver.
        /// @param impulse the angular impulse in units of
        /// kg*m*m/s
        /// @param wake also wake up the body
        void ApplyAngularImpulse(float impulse, bool wake) { return b2Body_ApplyAngularImpulse(Handle(), impulse, wake); }

        /// Get a local vector on a body given a world vector
        [[nodiscard]] b2Vec2 GetLocalVector(b2Vec2 worldVector) const { return b2Body_GetLocalVector(Handle(), worldVector); }

        /// Override the body's mass properties. Normally this is computed automatically using the
        ///	shape geometry and density. This information is lost if a shape is added or removed or if the
        ///	body type changes.
        void SetMassData(b2MassData massData) { return b2Body_SetMassData(Handle(), massData); }

        /// Wake a body from sleep. This wakes the entire island the body is touching.
        void Wake() { return b2Body_Wake(Handle()); }

        /// Adjust the linear damping. Normally this is set in b2BodyDef before creation.
        void SetLinearDamping(float linearDamping) { return b2Body_SetLinearDamping(Handle(), linearDamping); }

        /// Disable a body by removing it completely from the simulation
        void Disable() { return b2Body_Disable(Handle()); }

        /// Set the linear velocity of a body
        void SetLinearVelocity(b2Vec2 linearVelocity) { return b2Body_SetLinearVelocity(Handle(), linearVelocity); }

        /// Get a local point on a body given a world point
        [[nodiscard]] b2Vec2 GetLocalPoint(b2Vec2 worldPoint) const { return b2Body_GetLocalPoint(Handle(), worldPoint); }

        /// Get the mass data for a body.
        [[nodiscard]] b2MassData GetMassData() const { return b2Body_GetMassData(Handle()); }

        /// Set the world transform of a body. This acts as a teleport and is fairly expensive.
        void SetTransform(b2Vec2 position, float angle) { return b2Body_SetTransform(Handle(), position, angle); }

        /// Get the world angle of a body in radians.
        [[nodiscard]] float GetAngle() const { return b2Body_GetAngle(Handle()); }

        /// Set the angular velocity of a body in radians per second
        void SetAngularVelocity(float angularVelocity) { return b2Body_SetAngularVelocity(Handle(), angularVelocity); }

        /// Enable or disable sleeping this body. If sleeping is disabled the body will wake.
        void EnableSleep(bool enableSleep) { return b2Body_EnableSleep(Handle(), enableSleep); }

        /// Apply a force at a world point. If the force is not
        /// applied at the center of mass, it will generate a torque and
        /// affect the angular velocity. This wakes up the body.
        /// @param force the world force vector, usually in Newtons (N).
        /// @param point the world position of the point of application.
        /// @param wake also wake up the body
        void ApplyForce(b2Vec2 force, b2Vec2 point, bool wake) { return b2Body_ApplyForce(Handle(), force, point, wake); }

        /// Get the current angular damping.
        [[nodiscard]] float GetAngularDamping() const { return b2Body_GetAngularDamping(Handle()); }

        /// Does this body have fixed rotation?
        [[nodiscard]] bool IsFixedRotation() const { return b2Body_IsFixedRotation(Handle()); }

        /// This resets the mass properties to the sum of the mass properties of the fixtures.
        /// This normally does not need to be called unless you called SetMassData to override
        /// the mass and you later want to reset the mass.
        void ResetMassData() { return b2Body_ResetMassData(Handle()); }

        /// Set the user data for a body
        void SetUserData(void* userData) { return b2Body_SetUserData(Handle(), userData); }

        /// Get the type of a body
        [[nodiscard]] b2BodyType GetType() const { return b2Body_GetType(Handle()); }

        /// @return is sleeping enabled for this body?
        [[nodiscard]] bool IsSleepEnabled() const { return b2Body_IsSleepEnabled(Handle()); }

        /// Get the angular velocity of a body in radians per second
        [[nodiscard]] float GetAngularVelocity() const { return b2Body_GetAngularVelocity(Handle()); }

        /// Body identifier validation. Provides validation for up to 64K allocations.
        [[nodiscard]] bool IsValid() const { return b2Body_IsValid(Handle()); }

        /// Enable a body by adding it to the simulation
        void Enable() { return b2Body_Enable(Handle()); }

        /// Apply an impulse to the center of mass. This immediately modifies the velocity.
        /// This should be used for one-shot impulses. If you need a steady force,
        /// use a force instead, which will work better with the sub-stepping solver.
        /// @param impulse the world impulse vector, usually in N-seconds or kg-m/s.
        /// @param wake also wake up the body
        void ApplyLinearImpulseToCenter(b2Vec2 impulse, bool wake) { return b2Body_ApplyLinearImpulseToCenter(Handle(), impulse, wake); }

        /// Get the user data stored in a body
        [[nodiscard]] void* GetUserData() const { return b2Body_GetUserData(Handle()); }

        /// Apply a torque. This affects the angular velocity
        /// without affecting the linear velocity of the center of mass.
        /// @param torque about the z-axis (out of the screen), usually in N-m.
        /// @param wake also wake up the body
        void ApplyTorque(float torque, bool wake) { return b2Body_ApplyTorque(Handle(), torque, wake); }

        /// Get the world rotation of a body as a sine/cosine pair.
        [[nodiscard]] b2Rot GetRotation() const { return b2Body_GetRotation(Handle()); }

        /// Get the current world AABB that contains all the attached shapes. Note that this may not emcompass the body origin.
        ///	If there are no shapes attached then the returned AABB is empty and centered on the body origin.
        [[nodiscard]] b2AABB ComputeAABB() const { return b2Body_ComputeAABB(Handle()); }

        /// Get the linear velocity of a body's center of mass
        [[nodiscard]] b2Vec2 GetLinearVelocity() const { return b2Body_GetLinearVelocity(Handle()); }

        /// Get the touching contact data for a body
        [[nodiscard]] int32_t GetContactData(b2ContactData& contactData, int32_t capacity) const { return b2Body_GetContactData(Handle(), &contactData, capacity); }

        /// Is this body enabled?
        [[nodiscard]] bool IsEnabled() const { return b2Body_IsEnabled(Handle()); }

        /// Apply a force to the center of mass. This wakes up the body.
        /// @param force the world force vector, usually in Newtons (N).
        /// @param wake also wake up the body
        void ApplyForceToCenter(b2Vec2 force, bool wake) { return b2Body_ApplyForceToCenter(Handle(), force, wake); }

        [[nodiscard]] b2ShapeId GetNextShape(b2ShapeId shapeId) const { return b2Body_GetNextShape(shapeId); }

        /// Iterate over shapes on a body
        [[nodiscard]] b2ShapeId GetFirstShape() const { return b2Body_GetFirstShape(Handle()); }

        /// Is this body awake?
        [[nodiscard]] bool IsAwake() const { return b2Body_IsAwake(Handle()); }

        /// Get the mass of the body (kilograms)
        [[nodiscard]] float GetMass() const { return b2Body_GetMass(Handle()); }

        /// Get a world vector on a body given a local vector
        [[nodiscard]] b2Vec2 GetWorldVector(b2Vec2 localVector) const { return b2Body_GetWorldVector(Handle(), localVector); }

        /// Set this body to be a bullet. A bullet does continuous collision detection
        /// against dynamic bodies (but not other bullets).
        void SetBullet(bool flag) { return b2Body_SetBullet(Handle(), flag); }

        /// Get the world position of a body. This is the location of the body origin.
        [[nodiscard]] b2Vec2 GetPosition() const { return b2Body_GetPosition(Handle()); }

        /// Get the world transform of a body.
        [[nodiscard]] b2Transform GetTransform() const { return b2Body_GetTransform(Handle()); }

        /// Get the current linear damping.
        [[nodiscard]] float GetLinearDamping() const { return b2Body_GetLinearDamping(Handle()); }
    };

    class Shape
    {
        b2ShapeId id = b2_nullShapeId;

      protected:
        constexpr Shape() {}
        explicit constexpr Shape(b2ShapeId id) : id(id) {}

      public:
        // The constructor accepts either this or directly `b2ShapeDef`.
        struct Params : b2ShapeDef
        {
            Params() : b2ShapeDef(b2DefaultShapeDef()) {}
        };

        Shape(Shape&& other) noexcept : id(std::exchange(other.id, b2_nullShapeId)) {}
        Shape& operator=(Shape other) noexcept { std::swap(id, other.id); return *this; }

        ~Shape() { if (*this) b2DestroyShape(id); }

        [[nodiscard]] explicit operator bool() const { return B2_IS_NON_NULL(id); }
        [[nodiscard]] const b2ShapeId &Handle() const { return id; }

        /// Get the user data for a shape. This is useful when you get a shape id
        ///	from an event or query.
        [[nodiscard]] void* GetUserData() const { return b2Shape_GetUserData(Handle()); }

        /// Ray cast a shape directly
        [[nodiscard]] b2CastOutput RayCast(b2Vec2 origin, b2Vec2 translation) const { return b2Shape_RayCast(Handle(), origin, translation); }

        /// Access the line segment geometry of a shape. Asserts the type is correct.
        [[nodiscard]] const b2Segment GetSegment() const { return b2Shape_GetSegment(Handle()); }

        /// @return are contact events enabled?
        [[nodiscard]] bool AreContactEventsEnabled() const { return b2Shape_AreContactEventsEnabled(Handle()); }

        /// Allows you to change a shape to be a capsule or update the current capsule.
        void SetCapsule(const b2Capsule& capsule) { return b2Shape_SetCapsule(Handle(), &capsule); }

        /// @return are sensor events enabled?
        [[nodiscard]] bool AreSensorEventsEnabled() const { return b2Shape_AreSensorEventsEnabled(Handle()); }

        /// Set the current filter. This is almost as expensive as recreating the shape.
        void SetFilter(b2Filter filter) { return b2Shape_SetFilter(Handle(), filter); }

        /// Access the circle geometry of a shape. Asserts the type is correct.
        [[nodiscard]] const b2Circle GetCircle() const { return b2Shape_GetCircle(Handle()); }

        /// Get the maximum capacity required for retrieving all the touching contacts on a shape
        [[nodiscard]] int32_t GetContactCapacity() const { return b2Shape_GetContactCapacity(Handle()); }

        /// Set the density on a shape. Normally this is specified in b2ShapeDef.
        ///	This will not update the mass properties on the parent body until you
        /// call b2Body_ResetMassData.
        void SetDensity(float density) { return b2Shape_SetDensity(Handle(), density); }

        /// Access the capsule geometry of a shape. Asserts the type is correct.
        [[nodiscard]] const b2Capsule GetCapsule() const { return b2Shape_GetCapsule(Handle()); }

        /// Get the touching contact data for a shape. The provided shapeId will be either shapeIdA or shapeIdB on the contact data.
        [[nodiscard]] int32_t GetContactData(b2ContactData& contactData, int32_t capacity) const { return b2Shape_GetContactData(Handle(), &contactData, capacity); }

        /// Get the current world AABB
        [[nodiscard]] b2AABB GetAABB() const { return b2Shape_GetAABB(Handle()); }

        /// Get the density on a shape.
        [[nodiscard]] float GetDensity() const { return b2Shape_GetDensity(Handle()); }

        /// Allows you to change a shape to be a segment or update the current segment.
        void SetPolygon(const b2Polygon& polygon) { return b2Shape_SetPolygon(Handle(), &polygon); }

        /// Enable contact events for this shape. Only applies to kinematic and dynamic bodies. Ignored for sensors.
        void EnableContactEvents(bool flag) { return b2Shape_EnableContactEvents(Handle(), flag); }

        /// Get the current filter
        [[nodiscard]] b2Filter GetFilter() const { return b2Shape_GetFilter(Handle()); }

        /// Enable pre-solve contact events for this shape. Only applies to dynamic bodies. These are expensive
        ///	and must be carefully handled due to multi-threading. Ignored for sensors.
        void EnablePreSolveEvents(bool flag) { return b2Shape_EnablePreSolveEvents(Handle(), flag); }

        /// Get the restitution on a shape.
        [[nodiscard]] float GetRestitution() const { return b2Shape_GetRestitution(Handle()); }

        /// Access the convex polygon geometry of a shape. Asserts the type is correct.
        [[nodiscard]] const b2Polygon GetPolygon() const { return b2Shape_GetPolygon(Handle()); }

        /// Test a point for overlap with a shape
        [[nodiscard]] bool TestPoint(b2Vec2 point) const { return b2Shape_TestPoint(Handle(), point); }

        /// Is this shape a sensor? See b2ShapeDef.
        [[nodiscard]] bool IsSensor() const { return b2Shape_IsSensor(Handle()); }

        /// Access the smooth line segment geometry of a shape. These come from chain shapes.
        /// Asserts the type is correct.
        [[nodiscard]] const b2SmoothSegment GetSmoothSegment() const { return b2Shape_GetSmoothSegment(Handle()); }

        /// @return are pre-solve events enabled?
        [[nodiscard]] bool ArePreSolveEventsEnabled() const { return b2Shape_ArePreSolveEventsEnabled(Handle()); }

        /// Get the body that a shape is attached to
        [[nodiscard]] b2BodyId GetBody() const { return b2Shape_GetBody(Handle()); }

        /// Set the friction on a shape. Normally this is specified in b2ShapeDef.
        void SetFriction(float friction) { return b2Shape_SetFriction(Handle(), friction); }

        /// Allows you to change a shape to be a circle or update the current circle.
        /// This does not modify the mass properties.
        void SetCircle(const b2Circle& circle) { return b2Shape_SetCircle(Handle(), &circle); }

        /// If the type is b2_smoothSegmentShape then you can get the parent chain id.
        /// If the shape is not a smooth segment then this will return b2_nullChainId.
        [[nodiscard]] b2ChainId GetParentChain() const { return b2Shape_GetParentChain(Handle()); }

        /// Set the restitution (bounciness) on a shape. Normally this is specified in b2ShapeDef.
        void SetRestitution(float restitution) { return b2Shape_SetRestitution(Handle(), restitution); }

        /// Set the user data for a shape.
        void SetUserData(void* userData) { return b2Shape_SetUserData(Handle(), userData); }

        /// Get the type of a shape.
        [[nodiscard]] b2ShapeType GetType() const { return b2Shape_GetType(Handle()); }

        /// Get the friction on a shape.
        [[nodiscard]] float GetFriction() const { return b2Shape_GetFriction(Handle()); }

        /// Allows you to change a shape to be a segment or update the current segment.
        void SetSegment(const b2Segment& segment) { return b2Shape_SetSegment(Handle(), &segment); }

        /// Shape identifier validation. Provides validation for up to 64K allocations.
        [[nodiscard]] bool IsValid() const { return b2Shape_IsValid(Handle()); }

        /// Enable sensor events for this shape. Only applies to kinematic and dynamic bodies. Ignored for sensors.
        void EnableSensorEvents(bool flag) { return b2Shape_EnableSensorEvents(Handle(), flag); }
    };

    class Chain
    {
        b2ChainId id = b2_nullChainId;

      public:
        constexpr Chain() {}

        // The constructor accepts either this or directly `b2ChainDef`.
        struct Params : b2ChainDef
        {
            Params() : b2ChainDef(b2DefaultChainDef()) {}
        };

        /// Create a chain shape
        ///	@see b2ChainDef for details
        Chain(Body &body, const std::derived_from<b2ChainDef> auto &params) : id(b2CreateChain(body.Handle(), &params)) { if (!*this) throw std::runtime_error("Failed to create a `b2Chain`."); }

        Chain(Chain&& other) noexcept : id(std::exchange(other.id, b2_nullChainId)) {}
        Chain& operator=(Chain other) noexcept { std::swap(id, other.id); return *this; }

        ~Chain() { if (*this) b2DestroyChain(id); }

        [[nodiscard]] explicit operator bool() const { return B2_IS_NON_NULL(id); }
        [[nodiscard]] const b2ChainId &Handle() const { return id; }

        /// Set the restitution (bounciness) on a chain. Normally this is specified in b2ChainDef.
        void SetRestitution(float restitution) { return b2Chain_SetRestitution(Handle(), restitution); }

        /// Chain identifier validation. Provides validation for up to 64K allocations.
        [[nodiscard]] bool IsValid() const { return b2Chain_IsValid(Handle()); }

        /// Set the friction of a chain. Normally this is set in b2ChainDef.
        void SetFriction(float friction) { return b2Chain_SetFriction(Handle(), friction); }
    };

    class Joint
    {
        b2JointId id = b2_nullJointId;

      protected:
        constexpr Joint() {}
        explicit constexpr Joint(b2JointId id) : id(id) {}

      public:
        Joint(Joint&& other) noexcept : id(std::exchange(other.id, b2_nullJointId)) {}
        Joint& operator=(Joint other) noexcept { std::swap(id, other.id); return *this; }

        // Destructor validates the handle because it could've been destroyed by `Body::DestroyBodyAndJoints()`.
        ~Joint() { if (*this && IsValid()) b2DestroyJoint(id); }

        [[nodiscard]] explicit operator bool() const { return B2_IS_NON_NULL(id); }
        [[nodiscard]] const b2JointId &Handle() const { return id; }

        /// Get local anchor on bodyA
        [[nodiscard]] b2Vec2 GetLocalAnchorA() const { return b2Joint_GetLocalAnchorA(Handle()); }

        /// Get local anchor on bodyB
        [[nodiscard]] b2Vec2 GetLocalAnchorB() const { return b2Joint_GetLocalAnchorB(Handle()); }

        /// Wake the bodies connect to this joint
        void WakeBodies() { return b2Joint_WakeBodies(Handle()); }

        /// Set the user data on a joint
        void SetUserData(void* userData) { return b2Joint_SetUserData(Handle(), userData); }

        /// Get the user data on a joint
        [[nodiscard]] void* GetUserData() const { return b2Joint_GetUserData(Handle()); }

        /// Get body A on a joint
        [[nodiscard]] b2BodyId GetBodyA() const { return b2Joint_GetBodyA(Handle()); }

        /// Get body B on a joint
        [[nodiscard]] b2BodyId GetBodyB() const { return b2Joint_GetBodyB(Handle()); }

        /// Toggle collision between connected bodies
        void SetCollideConnected(bool shouldCollide) { return b2Joint_SetCollideConnected(Handle(), shouldCollide); }

        /// Is collision allowed between connected bodies?
        [[nodiscard]] bool GetCollideConnected() const { return b2Joint_GetCollideConnected(Handle()); }

        /// Joint identifier validation. Provides validation for up to 64K allocations.
        [[nodiscard]] bool IsValid() const { return b2Joint_IsValid(Handle()); }

        /// Get the joint type
        [[nodiscard]] b2JointType GetType() const { return b2Joint_GetType(Handle()); }
    };

    class DistanceJoint : public Joint
    {
      public:
        constexpr DistanceJoint() {}

        // The constructor accepts either this or directly `b2DistanceJointDef`.
        struct Params : b2DistanceJointDef
        {
            Params() : b2DistanceJointDef(b2DefaultDistanceJointDef()) {}
        };

        /// Create a distance joint
        ///	@see b2DistanceJointDef for details
        DistanceJoint(World &world, const std::derived_from<b2DistanceJointDef> auto &params) : Joint(b2CreateDistanceJoint(world.Handle(), &params)) { if (!*this) throw std::runtime_error("Failed to create a `b2DistanceJoint`."); }

        /// Adjust the softness parameters of a distance joint
        void SetTuning(float hertz, float dampingRatio) { return b2DistanceJoint_SetTuning(Handle(), hertz, dampingRatio); }

        /// Get the current length of a distance joint
        [[nodiscard]] float GetCurrentLength() const { return b2DistanceJoint_GetCurrentLength(Handle()); }

        /// Get the minimum distance joint length
        [[nodiscard]] float GetMinLength() const { return b2DistanceJoint_GetMinLength(Handle()); }

        /// Get the constraint force on a distance joint
        [[nodiscard]] float GetConstraintForce(float timeStep) const { return b2DistanceJoint_GetConstraintForce(Handle(), timeStep); }

        /// Set the minimum and maximum length parameters of a distance joint
        void SetLengthRange(float minLength, float maxLength) { return b2DistanceJoint_SetLengthRange(Handle(), minLength, maxLength); }

        /// Set the rest length of a distance joint
        void SetLength(float length) { return b2DistanceJoint_SetLength(Handle(), length); }

        /// Get the Hertz of a distance joint
        [[nodiscard]] float GetHertz() const { return b2DistanceJoint_GetHertz(Handle()); }

        /// Get the maximum distance joint length
        [[nodiscard]] float GetMaxLength() const { return b2DistanceJoint_GetMaxLength(Handle()); }

        /// Get the damping ratio of a distance joint
        [[nodiscard]] float GetDampingRatio() const { return b2DistanceJoint_GetDampingRatio(Handle()); }

        /// Get the rest length of a distance joint
        [[nodiscard]] float GetLength() const { return b2DistanceJoint_GetLength(Handle()); }
    };

    class MotorJoint : public Joint
    {
      public:
        constexpr MotorJoint() {}

        // The constructor accepts either this or directly `b2MotorJointDef`.
        struct Params : b2MotorJointDef
        {
            Params() : b2MotorJointDef(b2DefaultMotorJointDef()) {}
        };

        /// Create a motor joint
        ///	@see b2MotorJointDef for details
        MotorJoint(World &world, const std::derived_from<b2MotorJointDef> auto &params) : Joint(b2CreateMotorJoint(world.Handle(), &params)) { if (!*this) throw std::runtime_error("Failed to create a `b2MotorJoint`."); }

        /// Get the current constraint force for a motor joint
        [[nodiscard]] b2Vec2 GetConstraintForce() const { return b2MotorJoint_GetConstraintForce(Handle()); }

        /// Get the current constraint torque for a motor joint
        [[nodiscard]] float GetConstraintTorque() const { return b2MotorJoint_GetConstraintTorque(Handle()); }

        /// @return the angular offset target for a motor joint in radians
        [[nodiscard]] float GetAngularOffset() const { return b2MotorJoint_GetAngularOffset(Handle()); }

        /// Set the maximum force for a motor joint
        void SetMaxForce(float maxForce) { return b2MotorJoint_SetMaxForce(Handle(), maxForce); }

        /// @return the maximum force for a motor joint
        [[nodiscard]] float GetMaxForce() const { return b2MotorJoint_GetMaxForce(Handle()); }

        /// Set/Get the linear offset target for a motor joint
        void SetLinearOffset(b2Vec2 linearOffset) { return b2MotorJoint_SetLinearOffset(Handle(), linearOffset); }

        /// Set the correction factor for a motor joint
        void SetCorrectionFactor(float correctionFactor) { return b2MotorJoint_SetCorrectionFactor(Handle(), correctionFactor); }

        /// @return the correction factor for a motor joint
        [[nodiscard]] float GetCorrectionFactor() const { return b2MotorJoint_GetCorrectionFactor(Handle()); }

        /// Set the maximum torque for a motor joint
        void SetMaxTorque(float maxTorque) { return b2MotorJoint_SetMaxTorque(Handle(), maxTorque); }

        /// @return the linear offset target for a motor joint
        [[nodiscard]] b2Vec2 GetLinearOffset() const { return b2MotorJoint_GetLinearOffset(Handle()); }

        /// Set the angular offset target for a motor joint in radians
        void SetAngularOffset(float angularOffset) { return b2MotorJoint_SetAngularOffset(Handle(), angularOffset); }

        /// @return the maximum torque for a motor joint
        [[nodiscard]] float GetMaxTorque() const { return b2MotorJoint_GetMaxTorque(Handle()); }
    };

    class MouseJoint : public Joint
    {
      public:
        constexpr MouseJoint() {}

        // The constructor accepts either this or directly `b2MouseJointDef`.
        struct Params : b2MouseJointDef
        {
            Params() : b2MouseJointDef(b2DefaultMouseJointDef()) {}
        };

        /// Create a mouse joint
        ///	@see b2MouseJointDef for details
        MouseJoint(World &world, const std::derived_from<b2MouseJointDef> auto &params) : Joint(b2CreateMouseJoint(world.Handle(), &params)) { if (!*this) throw std::runtime_error("Failed to create a `b2MouseJoint`."); }

        /// @return the target for a mouse joint
        [[nodiscard]] b2Vec2 GetTarget() const { return b2MouseJoint_GetTarget(Handle()); }

        /// Get the Hertz of a mouse joint
        [[nodiscard]] float GetHertz() const { return b2MouseJoint_GetHertz(Handle()); }

        /// Get the damping ratio of a mouse joint
        [[nodiscard]] float GetDampingRatio() const { return b2MouseJoint_GetDampingRatio(Handle()); }

        /// Set the target for a mouse joint
        void SetTarget(b2Vec2 target) { return b2MouseJoint_SetTarget(Handle(), target); }

        /// Adjust the softness parameters of a mouse joint
        void SetTuning(float hertz, float dampingRatio) { return b2MouseJoint_SetTuning(Handle(), hertz, dampingRatio); }
    };

    class PrismaticJoint : public Joint
    {
      public:
        constexpr PrismaticJoint() {}

        // The constructor accepts either this or directly `b2PrismaticJointDef`.
        struct Params : b2PrismaticJointDef
        {
            Params() : b2PrismaticJointDef(b2DefaultPrismaticJointDef()) {}
        };

        /// Create a prismatic (slider) joint
        ///	@see b2PrismaticJointDef for details
        PrismaticJoint(World &world, const std::derived_from<b2PrismaticJointDef> auto &params) : Joint(b2CreatePrismaticJoint(world.Handle(), &params)) { if (!*this) throw std::runtime_error("Failed to create a `b2PrismaticJoint`."); }

        /// @return is the prismatic joint motor enabled
        [[nodiscard]] bool IsMotorEnabled() const { return b2PrismaticJoint_IsMotorEnabled(Handle()); }

        /// @return is the prismatic joint limit enabled
        [[nodiscard]] bool IsLimitEnabled() const { return b2PrismaticJoint_IsLimitEnabled(Handle()); }

        /// Enable/disable a prismatic joint motor
        void EnableMotor(bool enableMotor) { return b2PrismaticJoint_EnableMotor(Handle(), enableMotor); }

        /// Enable/disable a prismatic joint limit
        void EnableLimit(bool enableLimit) { return b2PrismaticJoint_EnableLimit(Handle(), enableLimit); }

        /// @return the maximum force for a prismatic joint motor
        [[nodiscard]] float GetMaxMotorForce() const { return b2PrismaticJoint_GetMaxMotorForce(Handle()); }

        /// Get the upper joint limit in length units (meters).
        [[nodiscard]] float GetUpperLimit() const { return b2PrismaticJoint_GetUpperLimit(Handle()); }

        /// Set the motor speed for a prismatic joint
        void SetMotorSpeed(float motorSpeed) { return b2PrismaticJoint_SetMotorSpeed(Handle(), motorSpeed); }

        /// Set the joint limits in length units (meters).
        void SetLimits(float lower, float upper) { return b2PrismaticJoint_SetLimits(Handle(), lower, upper); }

        /// @return the motor speed for a prismatic joint
        [[nodiscard]] float GetMotorSpeed() const { return b2PrismaticJoint_GetMotorSpeed(Handle()); }

        /// Get the lower joint limit in length units (meters).
        [[nodiscard]] float GetLowerLimit() const { return b2PrismaticJoint_GetLowerLimit(Handle()); }

        /// Get the current constraint torque for a prismatic joint
        [[nodiscard]] float GetConstraintTorque() const { return b2PrismaticJoint_GetConstraintTorque(Handle()); }

        /// Get the current constraint force for a prismatic joint
        [[nodiscard]] b2Vec2 GetConstraintForce() const { return b2PrismaticJoint_GetConstraintForce(Handle()); }

        /// Get the current motor force for a prismatic joint
        [[nodiscard]] float GetMotorForce() const { return b2PrismaticJoint_GetMotorForce(Handle()); }

        /// Set the maximum force for a prismatic joint motor
        void SetMaxMotorForce(float force) { return b2PrismaticJoint_SetMaxMotorForce(Handle(), force); }
    };

    class RevoluteJoint : public Joint
    {
      public:
        constexpr RevoluteJoint() {}

        // The constructor accepts either this or directly `b2RevoluteJointDef`.
        struct Params : b2RevoluteJointDef
        {
            Params() : b2RevoluteJointDef(b2DefaultRevoluteJointDef()) {}
        };

        /// Create a revolute (hinge) joint
        ///	@see b2RevoluteJointDef for details
        RevoluteJoint(World &world, const std::derived_from<b2RevoluteJointDef> auto &params) : Joint(b2CreateRevoluteJoint(world.Handle(), &params)) { if (!*this) throw std::runtime_error("Failed to create a `b2RevoluteJoint`."); }

        /// Set the maximum torque for a revolute joint motor
        void SetMaxMotorTorque(float torque) { return b2RevoluteJoint_SetMaxMotorTorque(Handle(), torque); }

        /// Get the upper joint limit in radians.
        [[nodiscard]] float GetUpperLimit() const { return b2RevoluteJoint_GetUpperLimit(Handle()); }

        /// @return the motor speed for a revolute joint in radians per second
        [[nodiscard]] float GetMotorSpeed() const { return b2RevoluteJoint_GetMotorSpeed(Handle()); }

        /// Set the motor speed for a revolute joint in radians per second
        void SetMotorSpeed(float motorSpeed) { return b2RevoluteJoint_SetMotorSpeed(Handle(), motorSpeed); }

        /// Get the lower joint limit in radians.
        [[nodiscard]] float GetLowerLimit() const { return b2RevoluteJoint_GetLowerLimit(Handle()); }

        /// Set the joint limits in radians.
        void SetLimits(float lower, float upper) { return b2RevoluteJoint_SetLimits(Handle(), lower, upper); }

        /// Get the current motor torque for a revolute joint
        [[nodiscard]] float GetMotorTorque() const { return b2RevoluteJoint_GetMotorTorque(Handle()); }

        /// Get the current constraint torque for a revolute joint
        [[nodiscard]] float GetConstraintTorque() const { return b2RevoluteJoint_GetConstraintTorque(Handle()); }

        /// Get the current constraint force for a revolute joint
        [[nodiscard]] b2Vec2 GetConstraintForce() const { return b2RevoluteJoint_GetConstraintForce(Handle()); }

        /// @return is the revolute joint motor enabled
        [[nodiscard]] bool IsMotorEnabled() const { return b2RevoluteJoint_IsMotorEnabled(Handle()); }

        /// @return is the revolute joint limit enabled
        [[nodiscard]] bool IsLimitEnabled() const { return b2RevoluteJoint_IsLimitEnabled(Handle()); }

        /// Enable/disable a revolute joint motor.
        void EnableMotor(bool enableMotor) { return b2RevoluteJoint_EnableMotor(Handle(), enableMotor); }

        /// Enable/disable a revolute joint limit.
        void EnableLimit(bool enableLimit) { return b2RevoluteJoint_EnableLimit(Handle(), enableLimit); }

        /// @return the maximum torque for a revolute joint motor
        [[nodiscard]] float GetMaxMotorTorque() const { return b2RevoluteJoint_GetMaxMotorTorque(Handle()); }
    };

    class WheelJoint : public Joint
    {
      public:
        constexpr WheelJoint() {}

        // The constructor accepts either this or directly `b2WheelJointDef`.
        struct Params : b2WheelJointDef
        {
            Params() : b2WheelJointDef(b2DefaultWheelJointDef()) {}
        };

        /// Create a wheel joint
        ///	@see b2WheelJointDef for details
        WheelJoint(World &world, const std::derived_from<b2WheelJointDef> auto &params) : Joint(b2CreateWheelJoint(world.Handle(), &params)) { if (!*this) throw std::runtime_error("Failed to create a `b2WheelJoint`."); }

        /// Set the wheel joint damping ratio (non-dimensional)
        void SetSpringDampingRatio(float dampingRatio) { return b2WheelJoint_SetSpringDampingRatio(Handle(), dampingRatio); }

        /// Get the upper joint limit in length units (meters).
        [[nodiscard]] float GetUpperLimit() const { return b2WheelJoint_GetUpperLimit(Handle()); }

        /// Get the current wheel joint constraint torque
        [[nodiscard]] float GetConstraintTorque() const { return b2WheelJoint_GetConstraintTorque(Handle()); }

        /// Set the wheel joint stiffness in Hertz
        void SetSpringHertz(float hertz) { return b2WheelJoint_SetSpringHertz(Handle(), hertz); }

        /// @return the wheel joint motor speed in radians per second
        [[nodiscard]] float GetMotorSpeed() const { return b2WheelJoint_GetMotorSpeed(Handle()); }

        /// Get the wheel joint current motor torque
        [[nodiscard]] float GetMotorTorque() const { return b2WheelJoint_GetMotorTorque(Handle()); }

        /// Set the wheel joint motor speed in radians per second
        void SetMotorSpeed(float motorSpeed) { return b2WheelJoint_SetMotorSpeed(Handle(), motorSpeed); }

        /// Get the lower joint limit in length units (meters).
        [[nodiscard]] float GetLowerLimit() const { return b2WheelJoint_GetLowerLimit(Handle()); }

        /// Set the wheel joint maximum motor torque
        void SetMaxMotorTorque(float torque) { return b2WheelJoint_SetMaxMotorTorque(Handle(), torque); }

        /// @return is the wheel joint motor enabled
        [[nodiscard]] bool IsMotorEnabled() const { return b2WheelJoint_IsMotorEnabled(Handle()); }

        /// @return is the wheel joint limit enabled
        [[nodiscard]] bool IsLimitEnabled() const { return b2WheelJoint_IsLimitEnabled(Handle()); }

        /// @return the wheel joint damping ratio (non-dimensional)
        [[nodiscard]] float GetSpringDampingRatio() const { return b2WheelJoint_GetSpringDampingRatio(Handle()); }

        /// Enable/disable the wheel joint limit.
        void EnableLimit(bool enableLimit) { return b2WheelJoint_EnableLimit(Handle(), enableLimit); }

        /// Set the joint limits in length units (meters).
        void SetLimits(float lower, float upper) { return b2WheelJoint_SetLimits(Handle(), lower, upper); }

        /// Get the current wheel joint constraint force
        [[nodiscard]] b2Vec2 GetConstraintForce() const { return b2WheelJoint_GetConstraintForce(Handle()); }

        /// Enable/disable the wheel joint motor
        void EnableMotor(bool enableMotor) { return b2WheelJoint_EnableMotor(Handle(), enableMotor); }

        /// @return the wheel joint stiffness in Hertz
        [[nodiscard]] float GetSpringHertz() const { return b2WheelJoint_GetSpringHertz(Handle()); }

        /// @return the wheel joint maximum motor torque
        [[nodiscard]] float GetMaxMotorTorque() const { return b2WheelJoint_GetMaxMotorTorque(Handle()); }
    };

    class WeldJoint : public Joint
    {
      public:
        constexpr WeldJoint() {}

        // The constructor accepts either this or directly `b2WeldJointDef`.
        struct Params : b2WeldJointDef
        {
            Params() : b2WeldJointDef(b2DefaultWeldJointDef()) {}
        };

        /// Create a weld joint
        ///	@see b2WeldJointDef for details
        WeldJoint(World &world, const std::derived_from<b2WeldJointDef> auto &params) : Joint(b2CreateWeldJoint(world.Handle(), &params)) { if (!*this) throw std::runtime_error("Failed to create a `b2WeldJoint`."); }

        /// Set weld joint angular stiffness in Hertz. 0 is rigid.
        void SetAngularHertz(float hertz) { return b2WeldJoint_SetAngularHertz(Handle(), hertz); }

        /// Set weld joint angular damping ratio (non-dimensional)
        void SetAngularDampingRatio(float dampingRatio) { return b2WeldJoint_SetAngularDampingRatio(Handle(), dampingRatio); }

        /// @return the weld joint angular stiffness in Hertz.
        [[nodiscard]] float GetAngularHertz() const { return b2WeldJoint_GetAngularHertz(Handle()); }

        /// @return the weld joint linear stiffness in Hertz.
        [[nodiscard]] float GetLinearHertz() const { return b2WeldJoint_GetLinearHertz(Handle()); }

        /// @return the weld joint angular damping ratio (non-dimensional)
        [[nodiscard]] float GetAngularDampingRatio() const { return b2WeldJoint_GetAngularDampingRatio(Handle()); }

        /// @return the weld joint linear damping ratio (non-dimensional)
        [[nodiscard]] float GetLinearDampingRatio() const { return b2WeldJoint_GetLinearDampingRatio(Handle()); }

        /// Set weld joint linear stiffness in Hertz. 0 is rigid.
        void SetLinearHertz(float hertz) { return b2WeldJoint_SetLinearHertz(Handle(), hertz); }

        /// Set weld joint linear damping ratio (non-dimensional)
        void SetLinearDampingRatio(float dampingRatio) { return b2WeldJoint_SetLinearDampingRatio(Handle(), dampingRatio); }
    };

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
        [[nodiscard]] float GetAreaRatio() const { return b2DynamicTree_GetAreaRatio(&value); }

        /// Ray-cast against the proxies in the tree. This relies on the callback
        /// to perform a exact ray-cast in the case were the proxy contains a shape.
        /// The callback also performs the any collision filtering. This has performance
        /// roughly equal to k * log(n), where k is the number of collisions and n is the
        /// number of proxies in the tree.
        /// @param input the ray-cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1).
        /// @param callback a callback class that is called for each proxy that is hit by the ray.
        void RayCast(const b2RayCastInput& input, uint32_t maskBits, b2TreeRayCastCallbackFcn& callback, void* context) const { return b2DynamicTree_RayCast(&value, &input, maskBits, &callback, context); }

        /// Compute the height of the binary tree in O(N) time. Should not be
        /// called often.
        [[nodiscard]] int32_t GetHeight() const { return b2DynamicTree_GetHeight(&value); }

        /// Move a proxy to a new AABB by removing and reinserting into the tree.
        void MoveProxy(int32_t proxyId, b2AABB aabb) { return b2DynamicTree_MoveProxy(&value, proxyId, aabb); }

        /// Shift the world origin. Useful for large worlds.
        /// The shift formula is: position -= newOrigin
        /// @param newOrigin the new origin with respect to the old origin
        void ShiftOrigin(b2Vec2 newOrigin) { return b2DynamicTree_ShiftOrigin(&value, newOrigin); }

        /// Validate this tree. For testing.
        void Validate() const { return b2DynamicTree_Validate(&value); }

        /// Get the AABB of a proxy
        [[nodiscard]] b2AABB GetAABB(int32_t proxyId) const { return b2DynamicTree_GetAABB(&value, proxyId); }

        /// Build an optimal tree. Very expensive. For testing.
        void RebuildBottomUp() { return b2DynamicTree_RebuildBottomUp(&value); }

        /// Ray-cast against the proxies in the tree. This relies on the callback
        /// to perform a exact ray-cast in the case were the proxy contains a shape.
        /// The callback also performs the any collision filtering. This has performance
        /// roughly equal to k * log(n), where k is the number of collisions and n is the
        /// number of proxies in the tree.
        /// @param input the ray-cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1).
        /// @param callback a callback class that is called for each proxy that is hit by the ray.
        void ShapeCast(const b2ShapeCastInput& input, uint32_t maskBits, b2TreeShapeCastCallbackFcn& callback, void* context) const { return b2DynamicTree_ShapeCast(&value, &input, maskBits, &callback, context); }

        /// Get the number of proxies created
        [[nodiscard]] int32_t GetProxyCount() const { return b2DynamicTree_GetProxyCount(&value); }

        /// Create a proxy. Provide a tight fitting AABB and a userData value.
        [[nodiscard]] int32_t CreateProxy(b2AABB aabb, uint32_t categoryBits, int32_t userData) { return b2DynamicTree_CreateProxy(&value, aabb, categoryBits, userData); }

        /// Get proxy user data
        /// @return the proxy user data or 0 if the id is invalid
        [[nodiscard]] int32_t GetUserData(int32_t proxyId) const { return b2DynamicTree_GetUserData(&value, proxyId); }

        /// Query an AABB for overlapping proxies. The callback class
        /// is called for each proxy that overlaps the supplied AABB.
        void QueryFiltered(b2AABB aabb, uint32_t maskBits, b2TreeQueryCallbackFcn& callback, void* context) const { return b2DynamicTree_QueryFiltered(&value, aabb, maskBits, &callback, context); }

        /// Get the maximum balance of the tree. The balance is the difference in height of the two children of a node.
        [[nodiscard]] int32_t GetMaxBalance() const { return b2DynamicTree_GetMaxBalance(&value); }

        /// Query an AABB for overlapping proxies. The callback class
        /// is called for each proxy that overlaps the supplied AABB.
        void Query(b2AABB aabb, b2TreeQueryCallbackFcn& callback, void* context) const { return b2DynamicTree_Query(&value, aabb, &callback, context); }

        /// Destroy a proxy. This asserts if the id is invalid.
        void DestroyProxy(int32_t proxyId) { return b2DynamicTree_DestroyProxy(&value, proxyId); }

        /// Rebuild the tree while retaining subtrees that haven't changed. Returns the number of boxes sorted.
        [[nodiscard]] int32_t Rebuild(bool fullBuild) { return b2DynamicTree_Rebuild(&value, fullBuild); }

        /// Enlarge a proxy and enlarge ancestors as necessary.
        void EnlargeProxy(int32_t proxyId, b2AABB aabb) { return b2DynamicTree_EnlargeProxy(&value, proxyId, aabb); }
    };

    class Rot : public b2Rot
    {
      public:
        constexpr Rot() {}

        constexpr Rot(float s, float c) : b2Rot{.s = s, .c = c} {}

        [[nodiscard]] bool IsValid() const { return b2Rot_IsValid(*this); }

        /// Get the x-axis
        [[nodiscard]] b2Vec2 GetXAxis() const { return b2Rot_GetXAxis(*this); }

        /// Get the y-axis
        [[nodiscard]] b2Vec2 GetYAxis() const { return b2Rot_GetYAxis(*this); }

        /// Get the angle in radians
        [[nodiscard]] float GetAngle() const { return b2Rot_GetAngle(*this); }
    };

    class AABB : public b2AABB
    {
      public:
        constexpr AABB() {}

        constexpr AABB(b2Vec2 lowerBound, b2Vec2 upperBound) : b2AABB{.lowerBound = lowerBound, .upperBound = upperBound} {}

        /// Get the extents of the AABB (half-widths).
        [[nodiscard]] b2Vec2 Extents() const { return b2AABB_Extents(*this); }

        /// Does a fully contain b
        [[nodiscard]] bool Contains(b2AABB b) const { return b2AABB_Contains(*this, b); }

        /// Union of two AABBs
        [[nodiscard]] b2AABB Union(b2AABB b) const { return b2AABB_Union(*this, b); }

        /// Get the center of the AABB.
        [[nodiscard]] b2Vec2 Center() const { return b2AABB_Center(*this); }

        [[nodiscard]] bool IsValid() const { return b2AABB_IsValid(*this); }
    };
} // namespace box2d
