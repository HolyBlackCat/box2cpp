#include "test_header.hpp"

#include <iostream>

template <typename T>
concept MoveOnly = std::movable<T> && !std::copyable<T>;

int main()
{
    // --- Non-joints:

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


    b2::World w(b2::World::Params{});

    b2::Body::Params bp;
    bp.type = b2_dynamicBody;

    b2::Body b = w.CreateBody(b2::OwningHandle, bp);
    b2::BodyRef b2 = b;

    b.CreateCircleShape(b2::DestroyWithParent, b2::Shape::Params{}, b2Circle{.center = b2Vec2(), .radius = 3});

    std::cout << b.IsValid() << '\n';

    b2::Body::Params bp2;
    bp2.position = b2Vec2(3,0);
    // bp2.type = b2_dynamicBody;

    {
        b2::BodyRef b2 = w.CreateBody(b2::DestroyWithParent, bp2);
        b2.CreateCircleShape(b2::DestroyWithParent, b2::Shape::Params{}, b2Circle{.center = b2Vec2(3,0), .radius = 3});

        b2::WeldJoint::Params wp;
        wp.bodyIdA = b;
        wp.bodyIdB = b2;

        b2::WeldJointConstRef weld = w.CreateWeldJoint(b2::OwningHandle, wp);
    }

    // w.create

    for (int i = 0; i < 10; i++)
    {
        w.Step(1/60.f, 4);
        std::cout << b.GetPosition().x << " " << b.GetPosition().y << " | ";
        // std::cout << b2.GetPosition().x << " " << b2.GetPosition().y << " | ";
        std::cout << '\n';
    }
}
