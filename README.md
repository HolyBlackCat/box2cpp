# 📦 box2cpp

**C++ bindings for [box2d 3.x](https://github.com/erincatto/box2d) physics engine (aka box2c)**

* Provides classes for all box2d entities: `World`, `Body`, `Shape`, `Chain`, all joint kinds, `DynamicTree`.

  * [With RAII](#ownership) - automatically destroy box2d objects in destructors

  * All related functions as methods (with const-correctness).

  * [Callbacks accept arbitrary lambdas](#using-callbacks) (no need to mess with `void *` and function pointers).

* Automatically generated, so no missing methods.

* Minimal - we don't wrap enums, most structs (including as `b2Vec2` and `b2AABB`), functions that are not related to a specific class.

And some extras:

* A [debug renderer](#debug-imgui-renderer) using ImGui.

## Hello world

```cpp
#include <box2cpp/box2cpp.h>

int main()
{
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
        std::cout << b.GetPosition().y << "\n";
    }

    // No cleanup needed, everything is destroyed automatically.
}
```



## Usage

Must use C++20 or newer.

Install [box2d 3.x](https://github.com/erincatto/box2d) as usual, our header includes their.

Clone and add `include/` to the header search path. Include `<box2cpp.h>`. The library is header-only.

**Note on branches:**<br/>
Box2cpp's `master` branch corresponds to box2d's `main` branch (open an issue if it gets outdated).<br/>
If you instead use box2d's release (e.g. `v3.0.0`), use the respective box2cpp's branch (e.g. `box2d-3.0.0`).

<details><summary>Regenerating the header</summary>

If the header is outdated, you can regenerate it yourself. (See comment at the beginning of header for underlying box2d version.)

Prerequisites: `git`, GNU `make`, `sed`, `perl`, `gawk` (GNU awk). On Windows, install that from MSYS2 and run inside MSYS2 terminal (or use WSL).

Run `make`. It will clone box2d to `./box2d` (or do nothing if already exists; make sure to advance it to the latest commit manually if needed), and will regenerate `include/box2cpp.h`. The comments at the beginning of `Makefile` have instructions for running tests, if you want to.

</details>

## How it works

### Ownership

You might have noticed from the example that all `.Create...(...)` functions accept an extra parameter at the beginning, either `b2::OwningHandle` or `b2::DestroyWithParent`.

* `w.CreateBody(b2::OwningHandle, ...)` returns `b2::Body` that will destroy the body in its destructor.

  You must save the return value somewhere, or the body will die immediately.

  `b2::Body` is movable but not copyable. You can destroy it manually without waiting for the destructor using `= {};` or `.Destroy()`.

  It's an error to destroy `b2::Body` after destroying `b2::World` it was created in (box2d should raise an assertion in debug mode).

* `w.CreateBody(b2::DestroyWithParent, ...)` returns `b2::BodyRef` that **doesn't** destroy the body automatically. Box2d will destroy it with the world. (And shapes and joints created in this mode will be destroyed with the bodies they're attached to.)

  This behavior is useful for entities that you don't want to touch after creation, such as shapes and joints, and perhaps static level geometry.

  You can safely ignore the return value.

  `b2::BodyRef` is copyable and cheap to copy, pass it around by value. Use it instead of references to `b2::Body`. Use `b2::BodyConstRef` instead of const references. (Using references to `b2::Body` as function parameters is not recommended, because you will be unable to pass `b2::BodyRef` you get from `b2::DestroyWithParent`.)

  You can still destroy the object manually by calling `.Destroy()` on a `b2::BodyRef`. Note that `= {};` for `b2::BodyRef` just zeroes the reference without destroying the body. It is an error to call `.Destroy()` on a `b2::BodyRef` if `b2::Body` exists for the same object (can only happen if you got it via `b2::OwningHandle` and then made a `b2::BodyRef` from it).

All our classes behave similarly. (E.g. there is `b2::Shape` and `b2::ShapeRef` for shapes, etc.)

All classes are default-constructible, and are null by default. They are convertible to `bool`, use that to check for null. There is also `.IsValid()` that validates the handle through box2d, but it shouldn't be very useful outside of debug assertions.

`b2::Body` is convertible to `b2::BodyRef` and `b2::BodyConstRef`. `b2::BodyRef` is convertible to `b2::BodyConstRef`. (And so on.)

Joint classes are a bit special, as they inherit from a common base (e.g. `b2::WeldJoint` inherits from `b2::Joint`). You can store them using either the specific derived classes or using the base class. Specific joint types are implicitly convertible to the generic joint, but the inverse (downcast) requires an `explicit` constructor call that raises an assertion if the actual joint type is wrong.

### Parameters structs

We have `b2::Body::Params` instead of `b2BodyDef` (and similarly for all other classes).

Those `Params` classes are automatically set to their default values in the constructor (unlike the original `...Def` structs, for which you must call `b2Default...Def()` to initialize them).

But since we have a constructor, you can no longer use the designated initializer syntax (you **can't** do `b2::Body::Params{.type = ..., .position = ...}`, and must do `b2::Body::Params p; p.type = ...; p.position = ...`;).

As a workaround, if you like oneliners more than you don't like macros, you can do this:

```cpp
#define ADJUST(x, ...) [&]{auto _ = x; __VA_ARGS__; return _;}

auto params = ADJUST(b2::Body::Params{}, _.type = ..., _.position = ...);
```

All our functions accept both `Params` and the original `...Def` structs if you must use them.

### Using callbacks

Functions accepting callbacks are enhanced to accept lambdas (including capturing lambdas), rather than only function pointers.

The `void* context` parameter is removed, it's no longer needed when you can use capturing lambdas directly.

When the callback receives a `b2ShapeId`, you can (and should) instead use `b2::ShapeRef` or `b2::ShapeConstRef` instead. (This is const-correct: if called on a const `b2::World`, `b2::ShapeConstRef` works and `b2::ShapeRef` doesn't).

```cpp
std::string str = "Foo!";
world.Overlap(
    b2ShapeProxy{.points = {point}, .count = 1, .radius = 0},
    b2DefaultQueryFilter(),
    [&](b2::ShapeRef shape)
    {
        (void)shape;
        std::cout << str << '\n'; // Lambdas can capture variables.
        return true;
    }
);
```
Compare with the original C API:
```cpp
std::string str = "Foo!";
b2Circle circle{.center{}, .radius = 1};
b2World_OverlapCircle(
    world,
    &circle,
    b2Transform_identity,
    b2DefaultQueryFilter(),
    [](b2ShapeId shape, void *context)
    {
        (void)shape;
        // Must pass information through a `void *`.
        std::cout << static_cast<std::string *>(context) << '\n';
        return true;
    },
    &str
);
```

Those callback improvements are not yet implemented for `b2::World::SetPreSolveCallback()`, and for functions in `b2::DynamicTree`.

## Debug ImGui renderer

We have an optional debug renderer using ImGui.

See comments in [`<box2cpp/debug_imgui_renderer.h>`](./include/box2cpp/debug_imgui_renderer.h) for usage details.

Simple example: (this assumes you already know how to use ImGui)
```cpp

#include <box2cpp/box2cpp.h>
#include <box2cpp/debug_imgui_renderer.h>

int main()
{
    // Initialize ImGui...

    b2::World world(b2::World::Params{});
    b2::DebugImguiRenderer debug_renderer;

    while (true)
    {
        // ImGui main loop...

        debug_renderer.DrawShapes(world); // Draw Box2D shapes.
        debug_renderer.MouseDrag(world); // Allow dragging bodies with mouse.
        debug_renderer.DrawModeToggles(); // Checkboxes for adjusting visualization.
    }
}

```
