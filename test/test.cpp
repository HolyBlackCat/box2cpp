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

    b.CreateShape(
        b2::DestroyWithParent,
        b2::Shape::Params{},
        b2Circle{.center = b2Vec2(), .radius = 3}
    );

    for (int i = 0; i < 10; i++)
    {
        w.Step(1/60.f, 4);
        //std::cout << b.GetPosition().y << "\n";
    }
}
