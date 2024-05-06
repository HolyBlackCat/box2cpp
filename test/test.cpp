#include "test_header.hpp"
#include "../box2c/include/box2d/math_functions.h"

#include <iostream>

template <typename T>
concept MoveOnly = std::movable<T> && !std::copyable<T>;

// ------ Basic type properties:

// --- Non-joints:

// to/from ID
static_assert(std::convertible_to<b2::Body, b2BodyId>);
static_assert(std::convertible_to<b2::BodyRef, b2BodyId>);
static_assert(std::convertible_to<b2::BodyConstRef, b2BodyId>);
static_assert(!std::constructible_from<b2::Body, b2BodyId>);
static_assert(std::convertible_to<b2BodyId, b2::BodyRef>);
static_assert(std::convertible_to<b2BodyId, b2::BodyConstRef>);

// basic ref/non-ref properties
static_assert(std::default_initializable<b2::Body> && MoveOnly<b2::Body>);
static_assert(std::default_initializable<b2::Body> && std::copyable<b2::BodyRef>);
static_assert(std::default_initializable<b2::Body> && std::copyable<b2::BodyConstRef>);

// owning <-> ref
static_assert(std::convertible_to<b2::Body, b2::BodyRef>);
static_assert(std::convertible_to<b2::Body, b2::BodyConstRef>);
static_assert(!std::convertible_to<b2::BodyRef, b2::Body>);
static_assert(!std::convertible_to<b2::BodyConstRef, b2::Body>);

// ref <-> const ref
static_assert(std::convertible_to<b2::BodyRef, b2::BodyConstRef>);
static_assert(!std::convertible_to<b2::BodyConstRef, b2::BodyRef>);

// --- Joints:

// to/from ID (base)
static_assert(std::convertible_to<b2::Joint, b2JointId>);
static_assert(std::convertible_to<b2::JointRef, b2JointId>);
static_assert(std::convertible_to<b2::JointConstRef, b2JointId>);
static_assert(!std::constructible_from<b2::Joint, b2JointId>);
static_assert(std::convertible_to<b2JointId, b2::JointRef>);
static_assert(std::convertible_to<b2JointId, b2::JointConstRef>);

// to/from ID (derived)
static_assert(std::convertible_to<b2::WeldJoint, b2JointId>);
static_assert(std::convertible_to<b2::WeldJointRef, b2JointId>);
static_assert(std::convertible_to<b2::WeldJointConstRef, b2JointId>);
static_assert(!std::constructible_from<b2::WeldJoint, b2JointId>);
static_assert(!std::convertible_to<b2JointId, b2::WeldJointRef>);
static_assert(!std::convertible_to<b2JointId, b2::WeldJointConstRef>);
static_assert(std::constructible_from<b2::WeldJointRef, b2JointId>);
static_assert(std::constructible_from<b2::WeldJointConstRef, b2JointId>);

// basic ref/non-ref properties
static_assert(std::default_initializable<b2::Body> && MoveOnly<b2::Joint>);
static_assert(std::default_initializable<b2::Body> && MoveOnly<b2::WeldJoint>);
static_assert(std::default_initializable<b2::Body> && std::copyable<b2::JointRef>);
static_assert(std::default_initializable<b2::Body> && std::copyable<b2::JointConstRef>);
static_assert(std::default_initializable<b2::Body> && std::copyable<b2::WeldJointRef>);
static_assert(std::default_initializable<b2::Body> && std::copyable<b2::WeldJointConstRef>);

// owning <-> ref (base)
static_assert(std::convertible_to<b2::Joint, b2::JointRef>);
static_assert(std::convertible_to<b2::Joint, b2::JointConstRef>);
static_assert(!std::convertible_to<b2::JointRef, b2::Joint>);
static_assert(!std::convertible_to<b2::JointConstRef, b2::Joint>);

// owning <-> ref (derived)
static_assert(std::convertible_to<b2::WeldJoint, b2::WeldJointRef>);
static_assert(std::convertible_to<b2::WeldJoint, b2::WeldJointConstRef>);
static_assert(!std::convertible_to<b2::WeldJointRef, b2::WeldJoint>);
static_assert(!std::convertible_to<b2::WeldJointConstRef, b2::WeldJoint>);

// owning <-> ref (mixed)
static_assert(std::convertible_to<b2::WeldJoint, b2::JointRef>);
static_assert(std::convertible_to<b2::WeldJoint, b2::JointConstRef>);
static_assert(!std::convertible_to<b2::JointRef, b2::WeldJoint>);
static_assert(!std::convertible_to<b2::JointConstRef, b2::WeldJoint>);

// owning <-> ref (mixed, wrong way)
static_assert(!std::convertible_to<b2::Joint, b2::WeldJointRef>);
static_assert(!std::convertible_to<b2::Joint, b2::WeldJointConstRef>);
static_assert(!std::convertible_to<b2::WeldJointRef, b2::Joint>);
static_assert(!std::convertible_to<b2::WeldJointConstRef, b2::Joint>);

// upcast:
// - upcast owning object
static_assert(std::convertible_to<b2::WeldJoint, b2::Joint>);
static_assert(std::convertible_to<b2::WeldJoint, b2::JointRef>);
static_assert(std::convertible_to<b2::WeldJoint, b2::JointConstRef>);
// - upcast ref
static_assert(!std::constructible_from<b2::Joint, b2::WeldJointRef>); // Drops constness = illegal.
static_assert(std::convertible_to<b2::WeldJointRef, b2::JointRef>);
static_assert(std::convertible_to<b2::WeldJointRef, b2::JointConstRef>);
// - upcast const ref
static_assert(!std::constructible_from<b2::Joint, b2::WeldJointConstRef>); // Drops constness = illegal.
static_assert(!std::constructible_from<b2::JointRef, b2::WeldJointConstRef>); // Drops constness = illegal.
static_assert(std::convertible_to<b2::WeldJointConstRef, b2::JointConstRef>);

// downcast
// - downcast owning
static_assert(!std::convertible_to<b2::Joint, b2::WeldJoint>); // implicit = no
static_assert(std::constructible_from<b2::WeldJoint, b2::Joint>); // explicit = yes
static_assert(!std::convertible_to<b2::Joint, b2::WeldJointRef>); // implicit = no
static_assert(std::constructible_from<b2::WeldJointRef, b2::Joint>); // explicit = yes
static_assert(!std::convertible_to<b2::Joint, b2::WeldJointConstRef>); // implicit = no
static_assert(std::constructible_from<b2::WeldJointConstRef, b2::Joint>); // explicit = yes
// - downcast ref
static_assert(!std::constructible_from<b2::WeldJoint, b2::JointRef>); // Drops reference = illegal.
static_assert(!std::convertible_to<b2::JointRef, b2::WeldJointRef>); // implicit = no
static_assert(std::constructible_from<b2::WeldJointRef, b2::JointRef>); // explicit = yes
static_assert(!std::convertible_to<b2::JointRef, b2::WeldJointConstRef>); // implicit = no
static_assert(std::constructible_from<b2::WeldJointConstRef, b2::JointRef>); // explicit = yes
// - downcast const ref
static_assert(!std::constructible_from<b2::WeldJoint, b2::JointConstRef>); // Drops reference = illegal.
static_assert(!std::constructible_from<b2::WeldJointRef, b2::JointConstRef>); // Drops constness = illegal.
static_assert(!std::convertible_to<b2::JointConstRef, b2::WeldJointConstRef>); // implicit = no
static_assert(std::constructible_from<b2::WeldJointConstRef, b2::JointConstRef>); // explicit = yes


// ------ Callbacks:

template <typename W, typename P, bool Passes>
concept CheckCallbacks = requires{
    // Overlap.
    requires Passes == requires(W w) { w.Overlap( b2Circle{}, b2Transform{}, b2DefaultQueryFilter(), [](P shape){ (void)shape; return bool{}; } ); };
    requires Passes == requires(W w) { w.Overlap( b2Capsule{}, b2Transform{}, b2DefaultQueryFilter(), [](P shape){ (void)shape; return bool{}; } ); };
    requires Passes == requires(W w) { w.Overlap( b2Polygon{}, b2Transform{}, b2DefaultQueryFilter(), [](P shape){ (void)shape; return bool{}; } ); };
    // Overlap AABB.
    requires Passes == requires(W w) { w.Overlap( b2AABB{}, b2DefaultQueryFilter(), [](P shape){ (void)shape; return bool{}; } ); };

    // Cast.
    requires Passes == requires(W w) { w.Cast( b2Circle{}, b2Transform{}, b2Vec2{}, b2DefaultQueryFilter(), [](P shape, b2Vec2 point, b2Vec2 normal, float fraction){ (void)shape; (void)point; (void)normal; (void)fraction; return float{}; } ); };
    requires Passes == requires(W w) { w.Cast( b2Capsule{}, b2Transform{}, b2Vec2{}, b2DefaultQueryFilter(), [](P shape, b2Vec2 point, b2Vec2 normal, float fraction){ (void)shape; (void)point; (void)normal; (void)fraction; return float{}; } ); };
    requires Passes == requires(W w) { w.Cast( b2Polygon{}, b2Transform{}, b2Vec2{}, b2DefaultQueryFilter(), [](P shape, b2Vec2 point, b2Vec2 normal, float fraction){ (void)shape; (void)point; (void)normal; (void)fraction; return float{}; } ); };
    // Raycast.
    requires Passes == requires(W w) { w.RayCast( b2Vec2{}, b2Vec2{}, b2DefaultQueryFilter(), [](P shape, b2Vec2 point, b2Vec2 normal, float fraction){ (void)shape; (void)point; (void)normal; (void)fraction; return float{}; } ); };
};

// Raw ID works for both const and non-const.
static_assert(CheckCallbacks<      b2::World, b2ShapeId, true>);
static_assert(CheckCallbacks<const b2::World, b2ShapeId, true>);
// Non-const ref only works for the non-const callback.
static_assert(CheckCallbacks<      b2::World, b2::ShapeRef, true>);
static_assert(CheckCallbacks<const b2::World, b2::ShapeRef, false>);
// Const ref works for everything.
static_assert(CheckCallbacks<      b2::World, b2::ShapeConstRef, true>);
static_assert(CheckCallbacks<const b2::World, b2::ShapeConstRef, true>);


int main()
{
    b2::World w(b2::World::Params{});
    (void)w.Handle();

    // Just check that raycast compiles.
    (void)w.RayCastClosest(b2Vec2{}, b2Vec2{}, b2DefaultQueryFilter());

    b2::Body::Params bp;
    bp.type = b2_dynamicBody;

    b2::Body b = w.CreateBody(b2::OwningHandle, bp);
    (void)b.Handle();

    b.CreateShape(
        b2::DestroyWithParent,
        b2::Shape::Params{},
        b2Circle{.center{}, .radius = 3}
    );

    for (int i = 0; i < 10; i++)
    {
        w.Step(1/60.f, 4);
        //std::cout << b.GetPosition().y << "\n";
    }

    // Overlap query.
    // Notice that we pass a lambda directly, without the `void* context` madness.
    std::string str = "Foo!";
    w.Overlap(
        b2Circle{.center{}, .radius = 1},
        b2Transform_identity,
        b2DefaultQueryFilter(),
        [&](b2::ShapeRef shape)
        {
            (void)shape;
            std::cout << str << '\n'; // Lambdas can capture values.
            return true;
        }
    );

    // Joints.
    bp.position.x += 2;
    b2::Body b2 = w.CreateBody(b2::OwningHandle, bp);

    b2::WeldJoint::Params wjp;
    wjp.bodyIdA = b;
    wjp.bodyIdB = b2;
    b2::WeldJoint wj = w.CreateJoint(b2::OwningHandle, wjp);
    assert(wj);
    b2::Joint bj = std::move(wj);
    assert(bj && !wj);
    wj = b2::WeldJoint(std::move(bj));
    assert(wj && !bj);
}
