#pragma once

#include "box2c/include/box2d/box2d.h"
#include "box2c/include/box2d/color.h"
#include "box2c/include/box2d/distance.h"
#include "box2c/include/box2d/dynamic_tree.h"
#include "box2c/include/box2d/hull.h"
#include "box2c/include/box2d/math.h"
#include "box2c/include/box2d/timer.h"

#include <cstddef>
#include <concepts>
#include <stdexcept>
#include <utility>

namespace b2
{
    using Capsule = b2Capsule;
    using Circle = b2Circle;
    using DebugDraw = b2DebugDraw;
    using Polygon = b2Polygon;
    using Segment = b2Segment;
    using Manifold = b2Manifold;
    /// Version numbering scheme.
    /// See http://en.wikipedia.org/wiki/Software_versioning
    using Version = b2Version;
    /// Result of computing the distance between two line segments
    using SegmentDistanceResult = b2SegmentDistanceResult;
    /// A distance proxy is used by the GJK algorithm. It encapsulates any shape.
    using DistanceProxy = b2DistanceProxy;
    /// Used to warm start b2Distance.
    /// Set count to zero on first call.
    using DistanceCache = b2DistanceCache;
    /// Input for b2Distance.
    /// You have to option to use the shape radii
    /// in the computation. Even
    using DistanceInput = b2DistanceInput;
    /// Output for b2Distance.
    using DistanceOutput = b2DistanceOutput;
    /// Input parameters for b2ShapeCast
    using ShapeCastPairInput = b2ShapeCastPairInput;
    /// This describes the motion of a body/shape for TOI computation. Shapes are defined with respect to the body origin,
    /// which may not coincide with the center of mass. However, to support dynamics we must interpolate the center of mass
    /// position.
    using Sweep = b2Sweep;
    /// Input parameters for b2TimeOfImpact
    using TOIInput = b2TOIInput;
    /// Output parameters for b2TimeOfImpact.
    using TOIOutput = b2TOIOutput;
    using RayCastInput = b2RayCastInput;
    using ShapeCastInput = b2ShapeCastInput;
    /// A node in the dynamic tree. The user does not interact with this directly.
    /// 16 + 16 + 8 + pad(8)
    using TreeNode = b2TreeNode;
    /// A begin touch event is generated when a shape starts to overlap a sensor shape.
    using SensorBeginTouchEvent = b2SensorBeginTouchEvent;
    /// An end touch event is generated when a shape stops overlapping a sensor shape.
    using SensorEndTouchEvent = b2SensorEndTouchEvent;
    /// Sensor events are buffered in the Box2D world and are available
    ///	as begin/end overlap event arrays after the time step is complete.
    ///	Note: these may become invalid if bodies and/or shapes are destroyed
    using SensorEvents = b2SensorEvents;
    /// A begin touch event is generated when two shapes begin touching. By convention the manifold
    /// normal points from shape A to shape B.
    using ContactBeginTouchEvent = b2ContactBeginTouchEvent;
    /// An end touch event is generated when two shapes stop touching.
    using ContactEndTouchEvent = b2ContactEndTouchEvent;
    /// Contact events are buffered in the Box2D world and are available
    ///	as event arrays after the time step is complete.
    ///	Note: these may become invalid if bodies and/or shapes are destroyed
    using ContactEvents = b2ContactEvents;
    /// The contact data for two shapes. By convention the manifold normal points
    ///	from shape A to shape B.
    using ContactData = b2ContactData;
    /// Triggered when a body moves from simulation. Not reported for bodies moved by the user.
    /// This also has a flag to indicate that the body went to sleep so the application can also
    /// sleep that actor/entity/object associated with the body.
    /// On the other hand if the flag does not indicate the body went to sleep then the application
    /// can treat the actor/entity/object associated with the body as awake.
    using BodyMoveEvent = b2BodyMoveEvent;
    /// Body events are buffered in the Box2D world and are available
    ///	as event arrays after the time step is complete.
    ///	Note: this date becomes invalid if bodies are destroyed
    using BodyEvents = b2BodyEvents;
    using Hull = b2Hull;
    /// Low level ray-cast or shape-cast output data
    using CastOutput = b2CastOutput;
    /// This holds the mass data computed for a shape.
    using MassData = b2MassData;
    /// A smooth line segment with one-sided collision. Only collides on the right side.
    /// Several of these are generated for a chain shape.
    /// ghost1 -> point1 -> point2 -> ghost2
    using SmoothSegment = b2SmoothSegment;
    /// World identifier
    using WorldId = b2WorldId;
    /// Body identifier
    using BodyId = b2BodyId;
    /// References a shape instance
    using ShapeId = b2ShapeId;
    /// References a joint instance
    using JointId = b2JointId;
    /// References a chain instances
    using ChainId = b2ChainId;
    /// A manifold point is a contact point belonging to a contact
    /// manifold. It holds details related to the geometry and dynamics
    /// of the contact points.
    using ManifoldPoint = b2ManifoldPoint;
    /// 2D vector
    /// This can be used to represent a point or free vector
    using Vec2 = b2Vec2;
    /// A 2D rigid transform
    using Transform = b2Transform;
    /// A 2-by-2 Matrix
    using Mat22 = b2Mat22;
    /// Color for debug drawing. Each value has the range [0,1].
    using Color = b2Color;
    /// Timer for profiling. This has platform specific code and may not work on every platform.
    using Timer = b2Timer;
    /// Result from b2World_RayCastClosest
    using RayResult = b2RayResult;
    /// This holds contact filtering data.
    using Filter = b2Filter;
    /// This holds contact filtering data.
    using QueryFilter = b2QueryFilter;
    /// Profiling data. Times are in milliseconds.
    using Profile = b2Profile;
    /// Counters that give details of the simulation size
    using Counters = b2Counters;

    enum class JointType
    { 
        DistanceJoint = b2_distanceJoint, 
        MotorJoint = b2_motorJoint, 
        MouseJoint = b2_mouseJoint, 
        PrismaticJoint = b2_prismaticJoint, 
        RevoluteJoint = b2_revoluteJoint, 
        WeldJoint = b2_weldJoint, 
        WheelJoint = b2_wheelJoint, 
    };

    /// All the colors! Credit to wherever I got this from, I forget.
    enum class HexColor
    { 
        AliceBlue = b2_colorAliceBlue, 
        AntiqueWhite = b2_colorAntiqueWhite, 
        AntiqueWhite1 = b2_colorAntiqueWhite1, 
        AntiqueWhite2 = b2_colorAntiqueWhite2, 
        AntiqueWhite3 = b2_colorAntiqueWhite3, 
        AntiqueWhite4 = b2_colorAntiqueWhite4, 
        Aqua = b2_colorAqua, 
        Aquamarine = b2_colorAquamarine, 
        Aquamarine1 = b2_colorAquamarine1, 
        Aquamarine2 = b2_colorAquamarine2, 
        Aquamarine3 = b2_colorAquamarine3, 
        Aquamarine4 = b2_colorAquamarine4, 
        Azure = b2_colorAzure, 
        Azure1 = b2_colorAzure1, 
        Azure2 = b2_colorAzure2, 
        Azure3 = b2_colorAzure3, 
        Azure4 = b2_colorAzure4, 
        Beige = b2_colorBeige, 
        Bisque = b2_colorBisque, 
        Bisque1 = b2_colorBisque1, 
        Bisque2 = b2_colorBisque2, 
        Bisque3 = b2_colorBisque3, 
        Bisque4 = b2_colorBisque4, 
        Black = b2_colorBlack, 
        BlanchedAlmond = b2_colorBlanchedAlmond, 
        Blue = b2_colorBlue, 
        Blue1 = b2_colorBlue1, 
        Blue2 = b2_colorBlue2, 
        Blue3 = b2_colorBlue3, 
        Blue4 = b2_colorBlue4, 
        BlueViolet = b2_colorBlueViolet, 
        Brown = b2_colorBrown, 
        Brown1 = b2_colorBrown1, 
        Brown2 = b2_colorBrown2, 
        Brown3 = b2_colorBrown3, 
        Brown4 = b2_colorBrown4, 
        Burlywood = b2_colorBurlywood, 
        Burlywood1 = b2_colorBurlywood1, 
        Burlywood2 = b2_colorBurlywood2, 
        Burlywood3 = b2_colorBurlywood3, 
        Burlywood4 = b2_colorBurlywood4, 
        CadetBlue = b2_colorCadetBlue, 
        CadetBlue1 = b2_colorCadetBlue1, 
        CadetBlue2 = b2_colorCadetBlue2, 
        CadetBlue3 = b2_colorCadetBlue3, 
        CadetBlue4 = b2_colorCadetBlue4, 
        Chartreuse = b2_colorChartreuse, 
        Chartreuse1 = b2_colorChartreuse1, 
        Chartreuse2 = b2_colorChartreuse2, 
        Chartreuse3 = b2_colorChartreuse3, 
        Chartreuse4 = b2_colorChartreuse4, 
        Chocolate = b2_colorChocolate, 
        Chocolate1 = b2_colorChocolate1, 
        Chocolate2 = b2_colorChocolate2, 
        Chocolate3 = b2_colorChocolate3, 
        Chocolate4 = b2_colorChocolate4, 
        Coral = b2_colorCoral, 
        Coral1 = b2_colorCoral1, 
        Coral2 = b2_colorCoral2, 
        Coral3 = b2_colorCoral3, 
        Coral4 = b2_colorCoral4, 
        CornflowerBlue = b2_colorCornflowerBlue, 
        Cornsilk = b2_colorCornsilk, 
        Cornsilk1 = b2_colorCornsilk1, 
        Cornsilk2 = b2_colorCornsilk2, 
        Cornsilk3 = b2_colorCornsilk3, 
        Cornsilk4 = b2_colorCornsilk4, 
        Crimson = b2_colorCrimson, 
        Cyan = b2_colorCyan, 
        Cyan1 = b2_colorCyan1, 
        Cyan2 = b2_colorCyan2, 
        Cyan3 = b2_colorCyan3, 
        Cyan4 = b2_colorCyan4, 
        DarkBlue = b2_colorDarkBlue, 
        DarkCyan = b2_colorDarkCyan, 
        DarkGoldenrod = b2_colorDarkGoldenrod, 
        DarkGoldenrod1 = b2_colorDarkGoldenrod1, 
        DarkGoldenrod2 = b2_colorDarkGoldenrod2, 
        DarkGoldenrod3 = b2_colorDarkGoldenrod3, 
        DarkGoldenrod4 = b2_colorDarkGoldenrod4, 
        DarkGray = b2_colorDarkGray, 
        DarkGreen = b2_colorDarkGreen, 
        DarkKhaki = b2_colorDarkKhaki, 
        DarkMagenta = b2_colorDarkMagenta, 
        DarkOliveGreen = b2_colorDarkOliveGreen, 
        DarkOliveGreen1 = b2_colorDarkOliveGreen1, 
        DarkOliveGreen2 = b2_colorDarkOliveGreen2, 
        DarkOliveGreen3 = b2_colorDarkOliveGreen3, 
        DarkOliveGreen4 = b2_colorDarkOliveGreen4, 
        DarkOrange = b2_colorDarkOrange, 
        DarkOrange1 = b2_colorDarkOrange1, 
        DarkOrange2 = b2_colorDarkOrange2, 
        DarkOrange3 = b2_colorDarkOrange3, 
        DarkOrange4 = b2_colorDarkOrange4, 
        DarkOrchid = b2_colorDarkOrchid, 
        DarkOrchid1 = b2_colorDarkOrchid1, 
        DarkOrchid2 = b2_colorDarkOrchid2, 
        DarkOrchid3 = b2_colorDarkOrchid3, 
        DarkOrchid4 = b2_colorDarkOrchid4, 
        DarkRed = b2_colorDarkRed, 
        DarkSalmon = b2_colorDarkSalmon, 
        DarkSeaGreen = b2_colorDarkSeaGreen, 
        DarkSeaGreen1 = b2_colorDarkSeaGreen1, 
        DarkSeaGreen2 = b2_colorDarkSeaGreen2, 
        DarkSeaGreen3 = b2_colorDarkSeaGreen3, 
        DarkSeaGreen4 = b2_colorDarkSeaGreen4, 
        DarkSlateBlue = b2_colorDarkSlateBlue, 
        DarkSlateGray = b2_colorDarkSlateGray, 
        DarkSlateGray1 = b2_colorDarkSlateGray1, 
        DarkSlateGray2 = b2_colorDarkSlateGray2, 
        DarkSlateGray3 = b2_colorDarkSlateGray3, 
        DarkSlateGray4 = b2_colorDarkSlateGray4, 
        DarkTurquoise = b2_colorDarkTurquoise, 
        DarkViolet = b2_colorDarkViolet, 
        DeepPink = b2_colorDeepPink, 
        DeepPink1 = b2_colorDeepPink1, 
        DeepPink2 = b2_colorDeepPink2, 
        DeepPink3 = b2_colorDeepPink3, 
        DeepPink4 = b2_colorDeepPink4, 
        DeepSkyBlue = b2_colorDeepSkyBlue, 
        DeepSkyBlue1 = b2_colorDeepSkyBlue1, 
        DeepSkyBlue2 = b2_colorDeepSkyBlue2, 
        DeepSkyBlue3 = b2_colorDeepSkyBlue3, 
        DeepSkyBlue4 = b2_colorDeepSkyBlue4, 
        DimGray = b2_colorDimGray, 
        DodgerBlue = b2_colorDodgerBlue, 
        DodgerBlue1 = b2_colorDodgerBlue1, 
        DodgerBlue2 = b2_colorDodgerBlue2, 
        DodgerBlue3 = b2_colorDodgerBlue3, 
        DodgerBlue4 = b2_colorDodgerBlue4, 
        Firebrick = b2_colorFirebrick, 
        Firebrick1 = b2_colorFirebrick1, 
        Firebrick2 = b2_colorFirebrick2, 
        Firebrick3 = b2_colorFirebrick3, 
        Firebrick4 = b2_colorFirebrick4, 
        FloralWhite = b2_colorFloralWhite, 
        ForestGreen = b2_colorForestGreen, 
        Fuchsia = b2_colorFuchsia, 
        Gainsboro = b2_colorGainsboro, 
        GhostWhite = b2_colorGhostWhite, 
        Gold = b2_colorGold, 
        Gold1 = b2_colorGold1, 
        Gold2 = b2_colorGold2, 
        Gold3 = b2_colorGold3, 
        Gold4 = b2_colorGold4, 
        Goldenrod = b2_colorGoldenrod, 
        Goldenrod1 = b2_colorGoldenrod1, 
        Goldenrod2 = b2_colorGoldenrod2, 
        Goldenrod3 = b2_colorGoldenrod3, 
        Goldenrod4 = b2_colorGoldenrod4, 
        Gray = b2_colorGray, 
        Gray0 = b2_colorGray0, 
        Gray1 = b2_colorGray1, 
        Gray10 = b2_colorGray10, 
        Gray100 = b2_colorGray100, 
        Gray11 = b2_colorGray11, 
        Gray12 = b2_colorGray12, 
        Gray13 = b2_colorGray13, 
        Gray14 = b2_colorGray14, 
        Gray15 = b2_colorGray15, 
        Gray16 = b2_colorGray16, 
        Gray17 = b2_colorGray17, 
        Gray18 = b2_colorGray18, 
        Gray19 = b2_colorGray19, 
        Gray2 = b2_colorGray2, 
        Gray20 = b2_colorGray20, 
        Gray21 = b2_colorGray21, 
        Gray22 = b2_colorGray22, 
        Gray23 = b2_colorGray23, 
        Gray24 = b2_colorGray24, 
        Gray25 = b2_colorGray25, 
        Gray26 = b2_colorGray26, 
        Gray27 = b2_colorGray27, 
        Gray28 = b2_colorGray28, 
        Gray29 = b2_colorGray29, 
        Gray3 = b2_colorGray3, 
        Gray30 = b2_colorGray30, 
        Gray31 = b2_colorGray31, 
        Gray32 = b2_colorGray32, 
        Gray33 = b2_colorGray33, 
        Gray34 = b2_colorGray34, 
        Gray35 = b2_colorGray35, 
        Gray36 = b2_colorGray36, 
        Gray37 = b2_colorGray37, 
        Gray38 = b2_colorGray38, 
        Gray39 = b2_colorGray39, 
        Gray4 = b2_colorGray4, 
        Gray40 = b2_colorGray40, 
        Gray41 = b2_colorGray41, 
        Gray42 = b2_colorGray42, 
        Gray43 = b2_colorGray43, 
        Gray44 = b2_colorGray44, 
        Gray45 = b2_colorGray45, 
        Gray46 = b2_colorGray46, 
        Gray47 = b2_colorGray47, 
        Gray48 = b2_colorGray48, 
        Gray49 = b2_colorGray49, 
        Gray5 = b2_colorGray5, 
        Gray50 = b2_colorGray50, 
        Gray51 = b2_colorGray51, 
        Gray52 = b2_colorGray52, 
        Gray53 = b2_colorGray53, 
        Gray54 = b2_colorGray54, 
        Gray55 = b2_colorGray55, 
        Gray56 = b2_colorGray56, 
        Gray57 = b2_colorGray57, 
        Gray58 = b2_colorGray58, 
        Gray59 = b2_colorGray59, 
        Gray6 = b2_colorGray6, 
        Gray60 = b2_colorGray60, 
        Gray61 = b2_colorGray61, 
        Gray62 = b2_colorGray62, 
        Gray63 = b2_colorGray63, 
        Gray64 = b2_colorGray64, 
        Gray65 = b2_colorGray65, 
        Gray66 = b2_colorGray66, 
        Gray67 = b2_colorGray67, 
        Gray68 = b2_colorGray68, 
        Gray69 = b2_colorGray69, 
        Gray7 = b2_colorGray7, 
        Gray70 = b2_colorGray70, 
        Gray71 = b2_colorGray71, 
        Gray72 = b2_colorGray72, 
        Gray73 = b2_colorGray73, 
        Gray74 = b2_colorGray74, 
        Gray75 = b2_colorGray75, 
        Gray76 = b2_colorGray76, 
        Gray77 = b2_colorGray77, 
        Gray78 = b2_colorGray78, 
        Gray79 = b2_colorGray79, 
        Gray8 = b2_colorGray8, 
        Gray80 = b2_colorGray80, 
        Gray81 = b2_colorGray81, 
        Gray82 = b2_colorGray82, 
        Gray83 = b2_colorGray83, 
        Gray84 = b2_colorGray84, 
        Gray85 = b2_colorGray85, 
        Gray86 = b2_colorGray86, 
        Gray87 = b2_colorGray87, 
        Gray88 = b2_colorGray88, 
        Gray89 = b2_colorGray89, 
        Gray9 = b2_colorGray9, 
        Gray90 = b2_colorGray90, 
        Gray91 = b2_colorGray91, 
        Gray92 = b2_colorGray92, 
        Gray93 = b2_colorGray93, 
        Gray94 = b2_colorGray94, 
        Gray95 = b2_colorGray95, 
        Gray96 = b2_colorGray96, 
        Gray97 = b2_colorGray97, 
        Gray98 = b2_colorGray98, 
        Gray99 = b2_colorGray99, 
        Green = b2_colorGreen, 
        Green1 = b2_colorGreen1, 
        Green2 = b2_colorGreen2, 
        Green3 = b2_colorGreen3, 
        Green4 = b2_colorGreen4, 
        GreenYellow = b2_colorGreenYellow, 
        Honeydew = b2_colorHoneydew, 
        Honeydew1 = b2_colorHoneydew1, 
        Honeydew2 = b2_colorHoneydew2, 
        Honeydew3 = b2_colorHoneydew3, 
        Honeydew4 = b2_colorHoneydew4, 
        HotPink = b2_colorHotPink, 
        HotPink1 = b2_colorHotPink1, 
        HotPink2 = b2_colorHotPink2, 
        HotPink3 = b2_colorHotPink3, 
        HotPink4 = b2_colorHotPink4, 
        IndianRed = b2_colorIndianRed, 
        IndianRed1 = b2_colorIndianRed1, 
        IndianRed2 = b2_colorIndianRed2, 
        IndianRed3 = b2_colorIndianRed3, 
        IndianRed4 = b2_colorIndianRed4, 
        Indigo = b2_colorIndigo, 
        Ivory = b2_colorIvory, 
        Ivory1 = b2_colorIvory1, 
        Ivory2 = b2_colorIvory2, 
        Ivory3 = b2_colorIvory3, 
        Ivory4 = b2_colorIvory4, 
        Khaki = b2_colorKhaki, 
        Khaki1 = b2_colorKhaki1, 
        Khaki2 = b2_colorKhaki2, 
        Khaki3 = b2_colorKhaki3, 
        Khaki4 = b2_colorKhaki4, 
        Lavender = b2_colorLavender, 
        LavenderBlush = b2_colorLavenderBlush, 
        LavenderBlush1 = b2_colorLavenderBlush1, 
        LavenderBlush2 = b2_colorLavenderBlush2, 
        LavenderBlush3 = b2_colorLavenderBlush3, 
        LavenderBlush4 = b2_colorLavenderBlush4, 
        LawnGreen = b2_colorLawnGreen, 
        LemonChiffon = b2_colorLemonChiffon, 
        LemonChiffon1 = b2_colorLemonChiffon1, 
        LemonChiffon2 = b2_colorLemonChiffon2, 
        LemonChiffon3 = b2_colorLemonChiffon3, 
        LemonChiffon4 = b2_colorLemonChiffon4, 
        LightBlue = b2_colorLightBlue, 
        LightBlue1 = b2_colorLightBlue1, 
        LightBlue2 = b2_colorLightBlue2, 
        LightBlue3 = b2_colorLightBlue3, 
        LightBlue4 = b2_colorLightBlue4, 
        LightCoral = b2_colorLightCoral, 
        LightCyan = b2_colorLightCyan, 
        LightCyan1 = b2_colorLightCyan1, 
        LightCyan2 = b2_colorLightCyan2, 
        LightCyan3 = b2_colorLightCyan3, 
        LightCyan4 = b2_colorLightCyan4, 
        LightGoldenrod = b2_colorLightGoldenrod, 
        LightGoldenrod1 = b2_colorLightGoldenrod1, 
        LightGoldenrod2 = b2_colorLightGoldenrod2, 
        LightGoldenrod3 = b2_colorLightGoldenrod3, 
        LightGoldenrod4 = b2_colorLightGoldenrod4, 
        LightGoldenrodYellow = b2_colorLightGoldenrodYellow, 
        LightGray = b2_colorLightGray, 
        LightGreen = b2_colorLightGreen, 
        LightPink = b2_colorLightPink, 
        LightPink1 = b2_colorLightPink1, 
        LightPink2 = b2_colorLightPink2, 
        LightPink3 = b2_colorLightPink3, 
        LightPink4 = b2_colorLightPink4, 
        LightSalmon = b2_colorLightSalmon, 
        LightSalmon1 = b2_colorLightSalmon1, 
        LightSalmon2 = b2_colorLightSalmon2, 
        LightSalmon3 = b2_colorLightSalmon3, 
        LightSalmon4 = b2_colorLightSalmon4, 
        LightSeaGreen = b2_colorLightSeaGreen, 
        LightSkyBlue = b2_colorLightSkyBlue, 
        LightSkyBlue1 = b2_colorLightSkyBlue1, 
        LightSkyBlue2 = b2_colorLightSkyBlue2, 
        LightSkyBlue3 = b2_colorLightSkyBlue3, 
        LightSkyBlue4 = b2_colorLightSkyBlue4, 
        LightSlateBlue = b2_colorLightSlateBlue, 
        LightSlateGray = b2_colorLightSlateGray, 
        LightSteelBlue = b2_colorLightSteelBlue, 
        LightSteelBlue1 = b2_colorLightSteelBlue1, 
        LightSteelBlue2 = b2_colorLightSteelBlue2, 
        LightSteelBlue3 = b2_colorLightSteelBlue3, 
        LightSteelBlue4 = b2_colorLightSteelBlue4, 
        LightYellow = b2_colorLightYellow, 
        LightYellow1 = b2_colorLightYellow1, 
        LightYellow2 = b2_colorLightYellow2, 
        LightYellow3 = b2_colorLightYellow3, 
        LightYellow4 = b2_colorLightYellow4, 
        Lime = b2_colorLime, 
        LimeGreen = b2_colorLimeGreen, 
        Linen = b2_colorLinen, 
        Magenta = b2_colorMagenta, 
        Magenta1 = b2_colorMagenta1, 
        Magenta2 = b2_colorMagenta2, 
        Magenta3 = b2_colorMagenta3, 
        Magenta4 = b2_colorMagenta4, 
        Maroon = b2_colorMaroon, 
        Maroon1 = b2_colorMaroon1, 
        Maroon2 = b2_colorMaroon2, 
        Maroon3 = b2_colorMaroon3, 
        Maroon4 = b2_colorMaroon4, 
        MediumAquamarine = b2_colorMediumAquamarine, 
        MediumBlue = b2_colorMediumBlue, 
        MediumOrchid = b2_colorMediumOrchid, 
        MediumOrchid1 = b2_colorMediumOrchid1, 
        MediumOrchid2 = b2_colorMediumOrchid2, 
        MediumOrchid3 = b2_colorMediumOrchid3, 
        MediumOrchid4 = b2_colorMediumOrchid4, 
        MediumPurple = b2_colorMediumPurple, 
        MediumPurple1 = b2_colorMediumPurple1, 
        MediumPurple2 = b2_colorMediumPurple2, 
        MediumPurple3 = b2_colorMediumPurple3, 
        MediumPurple4 = b2_colorMediumPurple4, 
        MediumSeaGreen = b2_colorMediumSeaGreen, 
        MediumSlateBlue = b2_colorMediumSlateBlue, 
        MediumSpringGreen = b2_colorMediumSpringGreen, 
        MediumTurquoise = b2_colorMediumTurquoise, 
        MediumVioletRed = b2_colorMediumVioletRed, 
        MidnightBlue = b2_colorMidnightBlue, 
        MintCream = b2_colorMintCream, 
        MistyRose = b2_colorMistyRose, 
        MistyRose1 = b2_colorMistyRose1, 
        MistyRose2 = b2_colorMistyRose2, 
        MistyRose3 = b2_colorMistyRose3, 
        MistyRose4 = b2_colorMistyRose4, 
        Moccasin = b2_colorMoccasin, 
        NavajoWhite = b2_colorNavajoWhite, 
        NavajoWhite1 = b2_colorNavajoWhite1, 
        NavajoWhite2 = b2_colorNavajoWhite2, 
        NavajoWhite3 = b2_colorNavajoWhite3, 
        NavajoWhite4 = b2_colorNavajoWhite4, 
        Navy = b2_colorNavy, 
        NavyBlue = b2_colorNavyBlue, 
        OldLace = b2_colorOldLace, 
        Olive = b2_colorOlive, 
        OliveDrab = b2_colorOliveDrab, 
        OliveDrab1 = b2_colorOliveDrab1, 
        OliveDrab2 = b2_colorOliveDrab2, 
        OliveDrab3 = b2_colorOliveDrab3, 
        OliveDrab4 = b2_colorOliveDrab4, 
        Orange = b2_colorOrange, 
        Orange1 = b2_colorOrange1, 
        Orange2 = b2_colorOrange2, 
        Orange3 = b2_colorOrange3, 
        Orange4 = b2_colorOrange4, 
        OrangeRed = b2_colorOrangeRed, 
        OrangeRed1 = b2_colorOrangeRed1, 
        OrangeRed2 = b2_colorOrangeRed2, 
        OrangeRed3 = b2_colorOrangeRed3, 
        OrangeRed4 = b2_colorOrangeRed4, 
        Orchid = b2_colorOrchid, 
        Orchid1 = b2_colorOrchid1, 
        Orchid2 = b2_colorOrchid2, 
        Orchid3 = b2_colorOrchid3, 
        Orchid4 = b2_colorOrchid4, 
        PaleGoldenrod = b2_colorPaleGoldenrod, 
        PaleGreen = b2_colorPaleGreen, 
        PaleGreen1 = b2_colorPaleGreen1, 
        PaleGreen2 = b2_colorPaleGreen2, 
        PaleGreen3 = b2_colorPaleGreen3, 
        PaleGreen4 = b2_colorPaleGreen4, 
        PaleTurquoise = b2_colorPaleTurquoise, 
        PaleTurquoise1 = b2_colorPaleTurquoise1, 
        PaleTurquoise2 = b2_colorPaleTurquoise2, 
        PaleTurquoise3 = b2_colorPaleTurquoise3, 
        PaleTurquoise4 = b2_colorPaleTurquoise4, 
        PaleVioletRed = b2_colorPaleVioletRed, 
        PaleVioletRed1 = b2_colorPaleVioletRed1, 
        PaleVioletRed2 = b2_colorPaleVioletRed2, 
        PaleVioletRed3 = b2_colorPaleVioletRed3, 
        PaleVioletRed4 = b2_colorPaleVioletRed4, 
        PapayaWhip = b2_colorPapayaWhip, 
        PeachPuff = b2_colorPeachPuff, 
        PeachPuff1 = b2_colorPeachPuff1, 
        PeachPuff2 = b2_colorPeachPuff2, 
        PeachPuff3 = b2_colorPeachPuff3, 
        PeachPuff4 = b2_colorPeachPuff4, 
        Peru = b2_colorPeru, 
        Pink = b2_colorPink, 
        Pink1 = b2_colorPink1, 
        Pink2 = b2_colorPink2, 
        Pink3 = b2_colorPink3, 
        Pink4 = b2_colorPink4, 
        Plum = b2_colorPlum, 
        Plum1 = b2_colorPlum1, 
        Plum2 = b2_colorPlum2, 
        Plum3 = b2_colorPlum3, 
        Plum4 = b2_colorPlum4, 
        PowderBlue = b2_colorPowderBlue, 
        Purple = b2_colorPurple, 
        Purple1 = b2_colorPurple1, 
        Purple2 = b2_colorPurple2, 
        Purple3 = b2_colorPurple3, 
        Purple4 = b2_colorPurple4, 
        RebeccaPurple = b2_colorRebeccaPurple, 
        Red = b2_colorRed, 
        Red1 = b2_colorRed1, 
        Red2 = b2_colorRed2, 
        Red3 = b2_colorRed3, 
        Red4 = b2_colorRed4, 
        RosyBrown = b2_colorRosyBrown, 
        RosyBrown1 = b2_colorRosyBrown1, 
        RosyBrown2 = b2_colorRosyBrown2, 
        RosyBrown3 = b2_colorRosyBrown3, 
        RosyBrown4 = b2_colorRosyBrown4, 
        RoyalBlue = b2_colorRoyalBlue, 
        RoyalBlue1 = b2_colorRoyalBlue1, 
        RoyalBlue2 = b2_colorRoyalBlue2, 
        RoyalBlue3 = b2_colorRoyalBlue3, 
        RoyalBlue4 = b2_colorRoyalBlue4, 
        SaddleBrown = b2_colorSaddleBrown, 
        Salmon = b2_colorSalmon, 
        Salmon1 = b2_colorSalmon1, 
        Salmon2 = b2_colorSalmon2, 
        Salmon3 = b2_colorSalmon3, 
        Salmon4 = b2_colorSalmon4, 
        SandyBrown = b2_colorSandyBrown, 
        SeaGreen = b2_colorSeaGreen, 
        SeaGreen1 = b2_colorSeaGreen1, 
        SeaGreen2 = b2_colorSeaGreen2, 
        SeaGreen3 = b2_colorSeaGreen3, 
        SeaGreen4 = b2_colorSeaGreen4, 
        Seashell = b2_colorSeashell, 
        Seashell1 = b2_colorSeashell1, 
        Seashell2 = b2_colorSeashell2, 
        Seashell3 = b2_colorSeashell3, 
        Seashell4 = b2_colorSeashell4, 
        Sienna = b2_colorSienna, 
        Sienna1 = b2_colorSienna1, 
        Sienna2 = b2_colorSienna2, 
        Sienna3 = b2_colorSienna3, 
        Sienna4 = b2_colorSienna4, 
        Silver = b2_colorSilver, 
        SkyBlue = b2_colorSkyBlue, 
        SkyBlue1 = b2_colorSkyBlue1, 
        SkyBlue2 = b2_colorSkyBlue2, 
        SkyBlue3 = b2_colorSkyBlue3, 
        SkyBlue4 = b2_colorSkyBlue4, 
        SlateBlue = b2_colorSlateBlue, 
        SlateBlue1 = b2_colorSlateBlue1, 
        SlateBlue2 = b2_colorSlateBlue2, 
        SlateBlue3 = b2_colorSlateBlue3, 
        SlateBlue4 = b2_colorSlateBlue4, 
        SlateGray = b2_colorSlateGray, 
        SlateGray1 = b2_colorSlateGray1, 
        SlateGray2 = b2_colorSlateGray2, 
        SlateGray3 = b2_colorSlateGray3, 
        SlateGray4 = b2_colorSlateGray4, 
        Snow = b2_colorSnow, 
        Snow1 = b2_colorSnow1, 
        Snow2 = b2_colorSnow2, 
        Snow3 = b2_colorSnow3, 
        Snow4 = b2_colorSnow4, 
        SpringGreen = b2_colorSpringGreen, 
        SpringGreen1 = b2_colorSpringGreen1, 
        SpringGreen2 = b2_colorSpringGreen2, 
        SpringGreen3 = b2_colorSpringGreen3, 
        SpringGreen4 = b2_colorSpringGreen4, 
        SteelBlue = b2_colorSteelBlue, 
        SteelBlue1 = b2_colorSteelBlue1, 
        SteelBlue2 = b2_colorSteelBlue2, 
        SteelBlue3 = b2_colorSteelBlue3, 
        SteelBlue4 = b2_colorSteelBlue4, 
        Tan = b2_colorTan, 
        Tan1 = b2_colorTan1, 
        Tan2 = b2_colorTan2, 
        Tan3 = b2_colorTan3, 
        Tan4 = b2_colorTan4, 
        Teal = b2_colorTeal, 
        Thistle = b2_colorThistle, 
        Thistle1 = b2_colorThistle1, 
        Thistle2 = b2_colorThistle2, 
        Thistle3 = b2_colorThistle3, 
        Thistle4 = b2_colorThistle4, 
        Tomato = b2_colorTomato, 
        Tomato1 = b2_colorTomato1, 
        Tomato2 = b2_colorTomato2, 
        Tomato3 = b2_colorTomato3, 
        Tomato4 = b2_colorTomato4, 
        Turquoise = b2_colorTurquoise, 
        Turquoise1 = b2_colorTurquoise1, 
        Turquoise2 = b2_colorTurquoise2, 
        Turquoise3 = b2_colorTurquoise3, 
        Turquoise4 = b2_colorTurquoise4, 
        Violet = b2_colorViolet, 
        VioletRed = b2_colorVioletRed, 
        VioletRed1 = b2_colorVioletRed1, 
        VioletRed2 = b2_colorVioletRed2, 
        VioletRed3 = b2_colorVioletRed3, 
        VioletRed4 = b2_colorVioletRed4, 
        WebGray = b2_colorWebGray, 
        WebGreen = b2_colorWebGreen, 
        WebMaroon = b2_colorWebMaroon, 
        WebPurple = b2_colorWebPurple, 
        Wheat = b2_colorWheat, 
        Wheat1 = b2_colorWheat1, 
        Wheat2 = b2_colorWheat2, 
        Wheat3 = b2_colorWheat3, 
        Wheat4 = b2_colorWheat4, 
        White = b2_colorWhite, 
        WhiteSmoke = b2_colorWhiteSmoke, 
        X11Gray = b2_colorX11Gray, 
        X11Green = b2_colorX11Green, 
        X11Maroon = b2_colorX11Maroon, 
        X11Purple = b2_colorX11Purple, 
        Yellow = b2_colorYellow, 
        Yellow1 = b2_colorYellow1, 
        Yellow2 = b2_colorYellow2, 
        Yellow3 = b2_colorYellow3, 
        Yellow4 = b2_colorYellow4, 
        YellowGreen = b2_colorYellowGreen, 
    };

    /// Shape type
    enum class ShapeType
    { 
        Circle = b2_circleShape, 
        Capsule = b2_capsuleShape, 
        Segment = b2_segmentShape, 
        Polygon = b2_polygonShape, 
        SmoothSegment = b2_smoothSegmentShape, 
        _count = b2_shapeTypeCount, 
    };

    /// The body type.
    /// static: zero mass, zero velocity, may be manually moved
    /// kinematic: zero mass, non-zero velocity set by user, moved by solver
    /// dynamic: positive mass, non-zero velocity determined by forces, moved by solver
    enum class BodyType
    { 
        StaticBody = b2_staticBody, 
        KinematicBody = b2_kinematicBody, 
        DynamicBody = b2_dynamicBody, 
        _count = b2_bodyTypeCount, 
    };

    /// Describes the TOI output
    enum class TOIState
    { 
        Unknown = b2_toiStateUnknown, 
        Failed = b2_toiStateFailed, 
        Overlapped = b2_toiStateOverlapped, 
        Hit = b2_toiStateHit, 
        Separated = b2_toiStateSeparated, 
    };

    // This can be passed to some functions to express ownership.
    struct OwningTag { explicit OwningTag() = default; };
    inline constexpr OwningTag Owning{};

    // This can be passed to the constructors of our classes along with a handle to just store a reference to it, without assuming ownership.
    struct RefTag { explicit RefTag() = default; };
    inline constexpr RefTag Ref{};

    template <typename T> concept MaybeOwningTag = std::same_as<T, OwningTag> || std::same_as<T, RefTag>;

    class Rot : public b2Rot
    {
      public:
        // Consturcts a null (invalid) object.
        constexpr Rot() {}

        constexpr Rot(float s, float c) : b2Rot{.s = s, .c = c} {}

        constexpr Rot(const b2Rot& raw_value) noexcept : b2Rot(raw_value) {}
        constexpr Rot& operator=(const b2Rot& raw_value) noexcept { b2Rot::operator=(raw_value); return *this; }

        [[nodiscard]] bool IsValid() const { return b2Rot_IsValid(*this); }

        /// Get the x-axis
        [[nodiscard]] Vec2 GetXAxis() const { return b2Rot_GetXAxis(*this); }

        /// Get the y-axis
        [[nodiscard]] Vec2 GetYAxis() const { return b2Rot_GetYAxis(*this); }

        /// Get the angle in radians
        [[nodiscard]] float GetAngle() const { return b2Rot_GetAngle(*this); }
    };

    class AABB : public b2AABB
    {
      public:
        // Consturcts a null (invalid) object.
        constexpr AABB() {}

        constexpr AABB(b2Vec2 lowerBound, b2Vec2 upperBound) : b2AABB{.lowerBound = lowerBound, .upperBound = upperBound} {}

        constexpr AABB(const b2AABB& raw_value) noexcept : b2AABB(raw_value) {}
        constexpr AABB& operator=(const b2AABB& raw_value) noexcept { b2AABB::operator=(raw_value); return *this; }

        /// Get the extents of the AABB (half-widths).
        [[nodiscard]] Vec2 Extents() const { return b2AABB_Extents(*this); }

        /// Does a fully contain b
        [[nodiscard]] bool Contains(AABB b) const { return b2AABB_Contains(*this, b); }

        /// Union of two AABBs
        [[nodiscard]] AABB Union(AABB b) const { return b2AABB_Union(*this, b); }

        /// Get the center of the AABB.
        [[nodiscard]] Vec2 Center() const { return b2AABB_Center(*this); }

        [[nodiscard]] bool IsValid() const { return b2AABB_IsValid(*this); }
    };

    /// Used to create a shape
    class Shape
    {
        b2ShapeId id = b2_nullShapeId;
        bool is_owner = false;

      protected:
        Shape(OwningTag, b2ShapeId id) noexcept : id(id), is_owner(true) {}

      public:
        // Consturcts a null (invalid) object.
        constexpr Shape() {}

        // The constructor accepts either this or directly `b2ShapeDef`.
        struct Params : b2ShapeDef
        {
            Params() : b2ShapeDef(b2DefaultShapeDef()) {}
        };

        Shape(const std::derived_from<b2ShapeDef> auto &params) : Shape(Owning, b2CreateShape(&params)) {}

        // Will act as a reference to this handle, but will not destroy it in the destructor.
        Shape(RefTag, b2ShapeId id) noexcept : id(id), is_owner(false) {}

        Shape(Shape&& other) noexcept : id(std::exchange(other.id, b2_nullShapeId)), is_owner(std::exchange(other.is_owner, false)) {}
        Shape& operator=(Shape other) noexcept { std::swap(id, other.id); std::swap(is_owner, other.is_owner); return *this; }

        ~Shape() { if (IsOwner()) b2DestroyShape(id); }

        [[nodiscard]] explicit operator bool() const { return B2_IS_NON_NULL(id); }
        [[nodiscard]] const b2ShapeId &Handle() const { return id; }
        [[nodiscard]] bool IsOwner() const { return *this && is_owner; } // Whether we own this handle or just act as a reference.

        /// Get the user data for a shape. This is useful when you get a shape id
        ///	from an event or query.
        [[nodiscard]] void* GetUserData() const { return b2Shape_GetUserData(Handle()); }

        /// Ray cast a shape directly
        [[nodiscard]] CastOutput RayCast(Vec2 origin, Vec2 translation) const { return b2Shape_RayCast(Handle(), origin, translation); }

        /// Access the line segment geometry of a shape. Asserts the type is correct.
        [[nodiscard]] const b2Segment GetSegment() const { return b2Shape_GetSegment(Handle()); }

        /// @return are contact events enabled?
        [[nodiscard]] bool AreContactEventsEnabled() const { return b2Shape_AreContactEventsEnabled(Handle()); }

        /// Allows you to change a shape to be a capsule or update the current capsule.
        void SetCapsule(const Capsule& capsule) { return b2Shape_SetCapsule(Handle(), &capsule); }

        /// @return are sensor events enabled?
        [[nodiscard]] bool AreSensorEventsEnabled() const { return b2Shape_AreSensorEventsEnabled(Handle()); }

        /// Set the current filter. This is almost as expensive as recreating the shape.
        void SetFilter(Filter filter) { return b2Shape_SetFilter(Handle(), filter); }

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
        [[nodiscard]] int32_t GetContactData(ContactData& contactData, int32_t capacity) const { return b2Shape_GetContactData(Handle(), &contactData, capacity); }

        /// Get the current world AABB
        [[nodiscard]] AABB GetAABB() const { return b2Shape_GetAABB(Handle()); }

        /// Get the density on a shape.
        [[nodiscard]] float GetDensity() const { return b2Shape_GetDensity(Handle()); }

        /// Allows you to change a shape to be a segment or update the current segment.
        void SetPolygon(const Polygon& polygon) { return b2Shape_SetPolygon(Handle(), &polygon); }

        /// Enable contact events for this shape. Only applies to kinematic and dynamic bodies. Ignored for sensors.
        void EnableContactEvents(bool flag) { return b2Shape_EnableContactEvents(Handle(), flag); }

        /// Get the current filter
        [[nodiscard]] Filter GetFilter() const { return b2Shape_GetFilter(Handle()); }

        /// Enable pre-solve contact events for this shape. Only applies to dynamic bodies. These are expensive
        ///	and must be carefully handled due to multi-threading. Ignored for sensors.
        void EnablePreSolveEvents(bool flag) { return b2Shape_EnablePreSolveEvents(Handle(), flag); }

        /// Get the restitution on a shape.
        [[nodiscard]] float GetRestitution() const { return b2Shape_GetRestitution(Handle()); }

        /// Access the convex polygon geometry of a shape. Asserts the type is correct.
        [[nodiscard]] const b2Polygon GetPolygon() const { return b2Shape_GetPolygon(Handle()); }

        /// Test a point for overlap with a shape
        [[nodiscard]] bool TestPoint(Vec2 point) const { return b2Shape_TestPoint(Handle(), point); }

        /// Is this shape a sensor? See b2ShapeDef.
        [[nodiscard]] bool IsSensor() const { return b2Shape_IsSensor(Handle()); }

        /// Access the smooth line segment geometry of a shape. These come from chain shapes.
        /// Asserts the type is correct.
        [[nodiscard]] const b2SmoothSegment GetSmoothSegment() const { return b2Shape_GetSmoothSegment(Handle()); }

        /// @return are pre-solve events enabled?
        [[nodiscard]] bool ArePreSolveEventsEnabled() const { return b2Shape_ArePreSolveEventsEnabled(Handle()); }

        /// Get the body that a shape is attached to
        [[nodiscard]] BodyId GetBody() const { return b2Shape_GetBody(Handle()); }

        /// Set the friction on a shape. Normally this is specified in b2ShapeDef.
        void SetFriction(float friction) { return b2Shape_SetFriction(Handle(), friction); }

        /// Allows you to change a shape to be a circle or update the current circle.
        /// This does not modify the mass properties.
        void SetCircle(const Circle& circle) { return b2Shape_SetCircle(Handle(), &circle); }

        /// If the type is b2_smoothSegmentShape then you can get the parent chain id.
        /// If the shape is not a smooth segment then this will return b2_nullChainId.
        [[nodiscard]] ChainId GetParentChain() const { return b2Shape_GetParentChain(Handle()); }

        /// Set the restitution (bounciness) on a shape. Normally this is specified in b2ShapeDef.
        void SetRestitution(float restitution) { return b2Shape_SetRestitution(Handle(), restitution); }

        /// Set the user data for a shape.
        void SetUserData(void* userData) { return b2Shape_SetUserData(Handle(), userData); }

        /// Get the type of a shape.
        [[nodiscard]] ShapeType GetType() const { return (ShapeType)b2Shape_GetType(Handle()); }

        /// Get the friction on a shape.
        [[nodiscard]] float GetFriction() const { return b2Shape_GetFriction(Handle()); }

        /// Allows you to change a shape to be a segment or update the current segment.
        void SetSegment(const Segment& segment) { return b2Shape_SetSegment(Handle(), &segment); }

        /// Shape identifier validation. Provides validation for up to 64K allocations.
        [[nodiscard]] bool IsValid() const { return b2Shape_IsValid(Handle()); }

        /// Enable sensor events for this shape. Only applies to kinematic and dynamic bodies. Ignored for sensors.
        void EnableSensorEvents(bool flag) { return b2Shape_EnableSensorEvents(Handle(), flag); }
    };

    /// World definition used to create a simulation world. Must be initialized using b2DefaultWorldDef.
    class World
    {
        b2WorldId id = b2_nullWorldId;
        bool is_owner = false;

      protected:
        World(OwningTag, b2WorldId id) noexcept : id(id), is_owner(true) {}

      public:
        // Consturcts a null (invalid) object.
        constexpr World() {}

        // The constructor accepts either this or directly `b2WorldDef`.
        struct Params : b2WorldDef
        {
            Params() : b2WorldDef(b2DefaultWorldDef()) {}
        };

        /// Create a world for rigid body simulation. This contains all the bodies, shapes, and constraints.
        World(const std::derived_from<b2WorldDef> auto &params) : World(Owning, b2CreateWorld(&params)) {}

        // Will act as a reference to this handle, but will not destroy it in the destructor.
        World(RefTag, b2WorldId id) noexcept : id(id), is_owner(false) {}

        World(World&& other) noexcept : id(std::exchange(other.id, b2_nullWorldId)), is_owner(std::exchange(other.is_owner, false)) {}
        World& operator=(World other) noexcept { std::swap(id, other.id); std::swap(is_owner, other.is_owner); return *this; }

        ~World() { if (IsOwner()) b2DestroyWorld(id); }

        [[nodiscard]] explicit operator bool() const { return B2_IS_NON_NULL(id); }
        [[nodiscard]] const b2WorldId &Handle() const { return id; }
        [[nodiscard]] bool IsOwner() const { return *this && is_owner; } // Whether we own this handle or just act as a reference.

        /// Overlap test for all shapes that overlap the provided capsule.
        void OverlapCapsule(const Capsule& capsule, Transform transform, QueryFilter filter, b2OverlapResultFcn* fcn, void* context) const { return b2World_OverlapCapsule(Handle(), &capsule, transform, filter, fcn, context); }

        /// Get counters and sizes
        [[nodiscard]] Counters GetCounters() const { return b2World_GetCounters(Handle()); }

        /// Get sensor events for the current time step. The event data is transient. Do not store a reference to this data.
        [[nodiscard]] SensorEvents GetSensorEvents() const { return b2World_GetSensorEvents(Handle()); }

        /// Register the pre-solve callback. This is optional.
        void SetPreSolveCallback(b2PreSolveFcn* fcn, void* context) { return b2World_SetPreSolveCallback(Handle(), fcn, context); }

        /// Take a time step. This performs collision detection, integration,
        /// and constraint solution.
        /// @param timeStep the amount of time to simulate, this should not vary.
        /// @param velocityIterations for the velocity constraint solver.
        /// @param relaxIterations for reducing constraint bounce solver.
        void Step(float timeStep, int32_t subStepCount) { return b2World_Step(Handle(), timeStep, subStepCount); }

        /// Ray-cast closest hit. Convenience function. This is less general than b2World_RayCast and does not allow for custom filtering.
        [[nodiscard]] RayResult RayCastClosest(Vec2 origin, Vec2 translation, QueryFilter filter) const { return b2World_RayCastClosest(Handle(), origin, translation, filter); }

        /// Cast a capsule through the world. Similar to a ray-cast except that a capsule is cast instead of a point.
        void CapsuleCast(const Capsule& capsule, Transform originTransform, Vec2 translation, QueryFilter filter, b2CastResultFcn* fcn, void* context) const { return b2World_CapsuleCast(Handle(), &capsule, originTransform, translation, filter, fcn, context); }

        /// Adjust the restitution threshold. Advanced feature for testing.
        void SetRestitutionThreshold(float value) { return b2World_SetRestitutionThreshold(Handle(), value); }

        /// Cast a capsule through the world. Similar to a ray-cast except that a polygon is cast instead of a point.
        void PolygonCast(const Polygon& polygon, Transform originTransform, Vec2 translation, QueryFilter filter, b2CastResultFcn* fcn, void* context) const { return b2World_PolygonCast(Handle(), &polygon, originTransform, translation, filter, fcn, context); }

        /// Overlap test for all shapes that overlap the provided polygon.
        void OverlapPolygon(const Polygon& polygon, Transform transform, QueryFilter filter, b2OverlapResultFcn* fcn, void* context) const { return b2World_OverlapPolygon(Handle(), &polygon, transform, filter, fcn, context); }

        /// Cast a circle through the world. Similar to a ray-cast except that a circle is cast instead of a point.
        void CircleCast(const Circle& circle, Transform originTransform, Vec2 translation, QueryFilter filter, b2CastResultFcn* fcn, void* context) const { return b2World_CircleCast(Handle(), &circle, originTransform, translation, filter, fcn, context); }

        /// Overlap test for for all shapes that overlap the provided circle.
        void OverlapCircle(const Circle& circle, Transform transform, QueryFilter filter, b2OverlapResultFcn* fcn, void* context) const { return b2World_OverlapCircle(Handle(), &circle, transform, filter, fcn, context); }

        /// Adjust contact tuning parameters:
        /// - hertz is the contact stiffness (cycles per second)
        /// - damping ratio is the contact bounciness with 1 being critical damping (non-dimensional)
        /// - push velocity is the maximum contact constraint push out velocity (meters per second)
        ///	Advanced feature
        void SetContactTuning(float hertz, float dampingRatio, float pushVelocity) { return b2World_SetContactTuning(Handle(), hertz, dampingRatio, pushVelocity); }

        /// Get the current profile
        [[nodiscard]] Profile GetProfile() const { return b2World_GetProfile(Handle()); }

        /// Call this to draw shapes and other debug draw data. This is intentionally non-const.
        void Draw(DebugDraw& debugDraw) const { return b2World_Draw(Handle(), &debugDraw); }

        /// Enable/disable continuous collision. Advanced feature for testing.
        void EnableContinuous(bool flag) { return b2World_EnableContinuous(Handle(), flag); }

        /// Enable/disable sleep. Advanced feature for testing.
        void EnableSleeping(bool flag) { return b2World_EnableSleeping(Handle(), flag); }

        /// World identifier validation. Provides validation for up to 64K allocations.
        [[nodiscard]] bool IsValid() const { return b2World_IsValid(Handle()); }

        /// Overlap test for all shapes that *potentially* overlap the provided AABB.
        void OverlapAABB(AABB aabb, QueryFilter filter, b2OverlapResultFcn* fcn, void* context) const { return b2World_OverlapAABB(Handle(), aabb, filter, fcn, context); }

        /// Get contact events for this current time step. The event data is transient. Do not store a reference to this data.
        [[nodiscard]] ContactEvents GetContactEvents() const { return b2World_GetContactEvents(Handle()); }

        /// Get the body events for the current time step. The event data is transient. Do not store a reference to this data.
        [[nodiscard]] BodyEvents GetBodyEvents() const { return b2World_GetBodyEvents(Handle()); }

        /// Ray-cast the world for all shapes in the path of the ray. Your callback
        /// controls whether you get the closest point, any point, or n-points.
        /// The ray-cast ignores shapes that contain the starting point.
        /// @param callback a user implemented callback class.
        /// @param point1 the ray starting point
        /// @param point2 the ray ending point
        void RayCast(Vec2 origin, Vec2 translation, QueryFilter filter, b2CastResultFcn* fcn, void* context) const { return b2World_RayCast(Handle(), origin, translation, filter, fcn, context); }

        /// Enable/disable constraint warm starting. Advanced feature for testing.
        void EnableWarmStarting(bool flag) { return b2World_EnableWarmStarting(Handle(), flag); }
    };

    /// A body definition holds all the data needed to construct a rigid body.
    /// You can safely re-use body definitions. Shapes are added to a body after construction.
    class Body
    {
        b2BodyId id = b2_nullBodyId;
        bool is_owner = false;

      protected:
        Body(OwningTag, b2BodyId id) noexcept : id(id), is_owner(true) {}

      public:
        // Consturcts a null (invalid) object.
        constexpr Body() {}

        // The constructor accepts either this or directly `b2BodyDef`.
        struct Params : b2BodyDef
        {
            Params() : b2BodyDef(b2DefaultBodyDef()) {}
        };

        /// Create a rigid body given a definition. No reference to the definition is retained.
        /// @warning This function is locked during callbacks.
        Body(World &world, const std::derived_from<b2BodyDef> auto &params) : Body(Owning, b2CreateBody(world.Handle(), &params)) {}

        // Will act as a reference to this handle, but will not destroy it in the destructor.
        Body(RefTag, b2BodyId id) noexcept : id(id), is_owner(false) {}

        Body(Body&& other) noexcept : id(std::exchange(other.id, b2_nullBodyId)), is_owner(std::exchange(other.is_owner, false)) {}
        Body& operator=(Body other) noexcept { std::swap(id, other.id); std::swap(is_owner, other.is_owner); return *this; }

        ~Body() { if (IsOwner()) b2DestroyBody(id); }

        [[nodiscard]] explicit operator bool() const { return B2_IS_NON_NULL(id); }
        [[nodiscard]] const b2BodyId &Handle() const { return id; }
        [[nodiscard]] bool IsOwner() const { return *this && is_owner; } // Whether we own this handle or just act as a reference.

        /// Create a polygon shape and attach it to a body. The shape definition and geometry are fully cloned.
        /// Contacts are not created until the next time step.
        ///	@return the shape id for accessing the shape
        Shape CreatePolygonShape(MaybeOwningTag auto ownership, const std::derived_from<b2ShapeDef> auto& def, const Polygon& polygon) { return {ownership, b2CreatePolygonShape(Handle(), &def, &polygon)}; }

        /// Create a circle shape and attach it to a body. The shape definition and geometry are fully cloned.
        /// Contacts are not created until the next time step.
        ///	@return the shape id for accessing the shape
        Shape CreateCircleShape(MaybeOwningTag auto ownership, const std::derived_from<b2ShapeDef> auto& def, const Circle& circle) { return {ownership, b2CreateCircleShape(Handle(), &def, &circle)}; }

        /// Create a line segment shape and attach it to a body. The shape definition and geometry are fully cloned.
        /// Contacts are not created until the next time step.
        ///	@return the shape id for accessing the shape
        Shape CreateSegmentShape(MaybeOwningTag auto ownership, const std::derived_from<b2ShapeDef> auto& def, const Segment& segment) { return {ownership, b2CreateSegmentShape(Handle(), &def, &segment)}; }

        /// Create a capsule shape and attach it to a body. The shape definition and geometry are fully cloned.
        /// Contacts are not created until the next time step.
        ///	@return the shape id for accessing the shape
        Shape CreateCapsuleShape(MaybeOwningTag auto ownership, const std::derived_from<b2ShapeDef> auto& def, const Capsule& capsule) { return {ownership, b2CreateCapsuleShape(Handle(), &def, &capsule)}; }

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
        void ApplyLinearImpulse(Vec2 impulse, Vec2 point, bool wake) { return b2Body_ApplyLinearImpulse(Handle(), impulse, point, wake); }

        /// Get a world point on a body given a local point
        [[nodiscard]] Vec2 GetWorldPoint(Vec2 localPoint) const { return b2Body_GetWorldPoint(Handle(), localPoint); }

        /// Is this body a bullet?
        [[nodiscard]] bool IsBullet() const { return b2Body_IsBullet(Handle()); }

        /// Adjust the gravity scale. Normally this is set in b2BodyDef before creation.
        void SetGravityScale(float gravityScale) { return b2Body_SetGravityScale(Handle(), gravityScale); }

        /// Set the type of a body. This has a similar cost to re-creating the body.
        void SetType(BodyType type) { return b2Body_SetType(Handle(), (b2BodyType)type); }

        /// Get the maximum capacity required for retrieving all the touching contacts on a body
        [[nodiscard]] int32_t GetContactCapacity() const { return b2Body_GetContactCapacity(Handle()); }

        /// Get the current gravity scale.
        [[nodiscard]] float GetGravityScale() const { return b2Body_GetGravityScale(Handle()); }

        /// Get the center of mass position of the body in local space.
        [[nodiscard]] Vec2 GetLocalCenterOfMass() const { return b2Body_GetLocalCenterOfMass(Handle()); }

        /// Get the center of mass position of the body in world space.
        [[nodiscard]] Vec2 GetWorldCenterOfMass() const { return b2Body_GetWorldCenterOfMass(Handle()); }

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
        [[nodiscard]] Vec2 GetLocalVector(Vec2 worldVector) const { return b2Body_GetLocalVector(Handle(), worldVector); }

        /// Override the body's mass properties. Normally this is computed automatically using the
        ///	shape geometry and density. This information is lost if a shape is added or removed or if the
        ///	body type changes.
        void SetMassData(MassData massData) { return b2Body_SetMassData(Handle(), massData); }

        /// Wake a body from sleep. This wakes the entire island the body is touching.
        void Wake() { return b2Body_Wake(Handle()); }

        /// Adjust the linear damping. Normally this is set in b2BodyDef before creation.
        void SetLinearDamping(float linearDamping) { return b2Body_SetLinearDamping(Handle(), linearDamping); }

        /// Disable a body by removing it completely from the simulation
        void Disable() { return b2Body_Disable(Handle()); }

        /// Set the linear velocity of a body
        void SetLinearVelocity(Vec2 linearVelocity) { return b2Body_SetLinearVelocity(Handle(), linearVelocity); }

        /// Get a local point on a body given a world point
        [[nodiscard]] Vec2 GetLocalPoint(Vec2 worldPoint) const { return b2Body_GetLocalPoint(Handle(), worldPoint); }

        /// Get the mass data for a body.
        [[nodiscard]] MassData GetMassData() const { return b2Body_GetMassData(Handle()); }

        /// Set the world transform of a body. This acts as a teleport and is fairly expensive.
        void SetTransform(Vec2 position, float angle) { return b2Body_SetTransform(Handle(), position, angle); }

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
        void ApplyForce(Vec2 force, Vec2 point, bool wake) { return b2Body_ApplyForce(Handle(), force, point, wake); }

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
        [[nodiscard]] BodyType GetType() const { return (BodyType)b2Body_GetType(Handle()); }

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
        void ApplyLinearImpulseToCenter(Vec2 impulse, bool wake) { return b2Body_ApplyLinearImpulseToCenter(Handle(), impulse, wake); }

        /// Get the user data stored in a body
        [[nodiscard]] void* GetUserData() const { return b2Body_GetUserData(Handle()); }

        /// Apply a torque. This affects the angular velocity
        /// without affecting the linear velocity of the center of mass.
        /// @param torque about the z-axis (out of the screen), usually in N-m.
        /// @param wake also wake up the body
        void ApplyTorque(float torque, bool wake) { return b2Body_ApplyTorque(Handle(), torque, wake); }

        /// Get the world rotation of a body as a sine/cosine pair.
        [[nodiscard]] Rot GetRotation() const { return b2Body_GetRotation(Handle()); }

        /// Get the current world AABB that contains all the attached shapes. Note that this may not emcompass the body origin.
        ///	If there are no shapes attached then the returned AABB is empty and centered on the body origin.
        [[nodiscard]] AABB ComputeAABB() const { return b2Body_ComputeAABB(Handle()); }

        /// Get the linear velocity of a body's center of mass
        [[nodiscard]] Vec2 GetLinearVelocity() const { return b2Body_GetLinearVelocity(Handle()); }

        /// Get the touching contact data for a body
        [[nodiscard]] int32_t GetContactData(ContactData& contactData, int32_t capacity) const { return b2Body_GetContactData(Handle(), &contactData, capacity); }

        /// Is this body enabled?
        [[nodiscard]] bool IsEnabled() const { return b2Body_IsEnabled(Handle()); }

        /// Apply a force to the center of mass. This wakes up the body.
        /// @param force the world force vector, usually in Newtons (N).
        /// @param wake also wake up the body
        void ApplyForceToCenter(Vec2 force, bool wake) { return b2Body_ApplyForceToCenter(Handle(), force, wake); }

        [[nodiscard]] ShapeId GetNextShape(Shape& shapeId) const { return b2Body_GetNextShape(shapeId.Handle()); }

        /// Iterate over shapes on a body
        [[nodiscard]] ShapeId GetFirstShape() const { return b2Body_GetFirstShape(Handle()); }

        /// Is this body awake?
        [[nodiscard]] bool IsAwake() const { return b2Body_IsAwake(Handle()); }

        /// Get the mass of the body (kilograms)
        [[nodiscard]] float GetMass() const { return b2Body_GetMass(Handle()); }

        /// Get a world vector on a body given a local vector
        [[nodiscard]] Vec2 GetWorldVector(Vec2 localVector) const { return b2Body_GetWorldVector(Handle(), localVector); }

        /// Set this body to be a bullet. A bullet does continuous collision detection
        /// against dynamic bodies (but not other bullets).
        void SetBullet(bool flag) { return b2Body_SetBullet(Handle(), flag); }

        /// Get the world position of a body. This is the location of the body origin.
        [[nodiscard]] Vec2 GetPosition() const { return b2Body_GetPosition(Handle()); }

        /// Get the world transform of a body.
        [[nodiscard]] Transform GetTransform() const { return b2Body_GetTransform(Handle()); }

        /// Get the current linear damping.
        [[nodiscard]] float GetLinearDamping() const { return b2Body_GetLinearDamping(Handle()); }
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
    class Chain
    {
        b2ChainId id = b2_nullChainId;
        bool is_owner = false;

      protected:
        Chain(OwningTag, b2ChainId id) noexcept : id(id), is_owner(true) {}

      public:
        // Consturcts a null (invalid) object.
        constexpr Chain() {}

        // The constructor accepts either this or directly `b2ChainDef`.
        struct Params : b2ChainDef
        {
            Params() : b2ChainDef(b2DefaultChainDef()) {}
        };

        /// Create a chain shape
        ///	@see b2ChainDef for details
        Chain(Body &body, const std::derived_from<b2ChainDef> auto &params) : Chain(Owning, b2CreateChain(body.Handle(), &params)) {}

        // Will act as a reference to this handle, but will not destroy it in the destructor.
        Chain(RefTag, b2ChainId id) noexcept : id(id), is_owner(false) {}

        Chain(Chain&& other) noexcept : id(std::exchange(other.id, b2_nullChainId)), is_owner(std::exchange(other.is_owner, false)) {}
        Chain& operator=(Chain other) noexcept { std::swap(id, other.id); std::swap(is_owner, other.is_owner); return *this; }

        ~Chain() { if (IsOwner()) b2DestroyChain(id); }

        [[nodiscard]] explicit operator bool() const { return B2_IS_NON_NULL(id); }
        [[nodiscard]] const b2ChainId &Handle() const { return id; }
        [[nodiscard]] bool IsOwner() const { return *this && is_owner; } // Whether we own this handle or just act as a reference.

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
        bool is_owner = false;

      protected:
        Joint(OwningTag, b2JointId id) noexcept : id(id), is_owner(true) {}

      public:
        // Consturcts a null (invalid) object.
        constexpr Joint() {}

        // Will act as a reference to this handle, but will not destroy it in the destructor.
        Joint(RefTag, b2JointId id) noexcept : id(id), is_owner(false) {}

        Joint(Joint&& other) noexcept : id(std::exchange(other.id, b2_nullJointId)), is_owner(std::exchange(other.is_owner, false)) {}
        Joint& operator=(Joint other) noexcept { std::swap(id, other.id); std::swap(is_owner, other.is_owner); return *this; }

        // Destructor validates the handle because it could've been destroyed by `Body::DestroyBodyAndJoints()`.
        ~Joint() { if (IsOwner() && IsValid()) b2DestroyJoint(id); }

        [[nodiscard]] explicit operator bool() const { return B2_IS_NON_NULL(id); }
        [[nodiscard]] const b2JointId &Handle() const { return id; }
        [[nodiscard]] bool IsOwner() const { return *this && is_owner; } // Whether we own this handle or just act as a reference.

        /// Get local anchor on bodyA
        [[nodiscard]] Vec2 GetLocalAnchorA() const { return b2Joint_GetLocalAnchorA(Handle()); }

        /// Get local anchor on bodyB
        [[nodiscard]] Vec2 GetLocalAnchorB() const { return b2Joint_GetLocalAnchorB(Handle()); }

        /// Wake the bodies connect to this joint
        void WakeBodies() { return b2Joint_WakeBodies(Handle()); }

        /// Set the user data on a joint
        void SetUserData(void* userData) { return b2Joint_SetUserData(Handle(), userData); }

        /// Get the user data on a joint
        [[nodiscard]] void* GetUserData() const { return b2Joint_GetUserData(Handle()); }

        /// Get body A on a joint
        [[nodiscard]] BodyId GetBodyA() const { return b2Joint_GetBodyA(Handle()); }

        /// Get body B on a joint
        [[nodiscard]] BodyId GetBodyB() const { return b2Joint_GetBodyB(Handle()); }

        /// Toggle collision between connected bodies
        void SetCollideConnected(bool shouldCollide) { return b2Joint_SetCollideConnected(Handle(), shouldCollide); }

        /// Is collision allowed between connected bodies?
        [[nodiscard]] bool GetCollideConnected() const { return b2Joint_GetCollideConnected(Handle()); }

        /// Joint identifier validation. Provides validation for up to 64K allocations.
        [[nodiscard]] bool IsValid() const { return b2Joint_IsValid(Handle()); }

        /// Get the joint type
        [[nodiscard]] JointType GetType() const { return (JointType)b2Joint_GetType(Handle()); }
    };

    /// Distance joint definition. This requires defining an anchor point on both
    /// bodies and the non-zero distance of the distance joint. The definition uses
    /// local anchor points so that the initial configuration can violate the
    /// constraint slightly. This helps when saving and loading a game.
    class DistanceJoint : public Joint
    {
      protected:
        DistanceJoint(OwningTag, b2JointId id) noexcept : Joint(Owning, id) {}

      public:
        // Consturcts a null (invalid) object.
        constexpr DistanceJoint() {}

        // The constructor accepts either this or directly `b2DistanceJointDef`.
        struct Params : b2DistanceJointDef
        {
            Params() : b2DistanceJointDef(b2DefaultDistanceJointDef()) {}
        };

        /// Create a distance joint
        ///	@see b2DistanceJointDef for details
        DistanceJoint(World &world, const std::derived_from<b2DistanceJointDef> auto &params) : DistanceJoint(Owning, b2CreateDistanceJoint(world.Handle(), &params)) {}

        // Will act as a reference to this handle, but will not destroy it in the destructor.
        DistanceJoint(RefTag, b2JointId id) noexcept : Joint(Ref, id) {}

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

    /// A motor joint is used to control the relative motion
    /// between two bodies. A typical usage is to control the movement
    /// of a dynamic body with respect to the ground.
    class MotorJoint : public Joint
    {
      protected:
        MotorJoint(OwningTag, b2JointId id) noexcept : Joint(Owning, id) {}

      public:
        // Consturcts a null (invalid) object.
        constexpr MotorJoint() {}

        // The constructor accepts either this or directly `b2MotorJointDef`.
        struct Params : b2MotorJointDef
        {
            Params() : b2MotorJointDef(b2DefaultMotorJointDef()) {}
        };

        /// Create a motor joint
        ///	@see b2MotorJointDef for details
        MotorJoint(World &world, const std::derived_from<b2MotorJointDef> auto &params) : MotorJoint(Owning, b2CreateMotorJoint(world.Handle(), &params)) {}

        // Will act as a reference to this handle, but will not destroy it in the destructor.
        MotorJoint(RefTag, b2JointId id) noexcept : Joint(Ref, id) {}

        /// Get the current constraint force for a motor joint
        [[nodiscard]] Vec2 GetConstraintForce() const { return b2MotorJoint_GetConstraintForce(Handle()); }

        /// Get the current constraint torque for a motor joint
        [[nodiscard]] float GetConstraintTorque() const { return b2MotorJoint_GetConstraintTorque(Handle()); }

        /// @return the angular offset target for a motor joint in radians
        [[nodiscard]] float GetAngularOffset() const { return b2MotorJoint_GetAngularOffset(Handle()); }

        /// Set the maximum force for a motor joint
        void SetMaxForce(float maxForce) { return b2MotorJoint_SetMaxForce(Handle(), maxForce); }

        /// @return the maximum force for a motor joint
        [[nodiscard]] float GetMaxForce() const { return b2MotorJoint_GetMaxForce(Handle()); }

        /// Set/Get the linear offset target for a motor joint
        void SetLinearOffset(Vec2 linearOffset) { return b2MotorJoint_SetLinearOffset(Handle(), linearOffset); }

        /// Set the correction factor for a motor joint
        void SetCorrectionFactor(float correctionFactor) { return b2MotorJoint_SetCorrectionFactor(Handle(), correctionFactor); }

        /// @return the correction factor for a motor joint
        [[nodiscard]] float GetCorrectionFactor() const { return b2MotorJoint_GetCorrectionFactor(Handle()); }

        /// Set the maximum torque for a motor joint
        void SetMaxTorque(float maxTorque) { return b2MotorJoint_SetMaxTorque(Handle(), maxTorque); }

        /// @return the linear offset target for a motor joint
        [[nodiscard]] Vec2 GetLinearOffset() const { return b2MotorJoint_GetLinearOffset(Handle()); }

        /// Set the angular offset target for a motor joint in radians
        void SetAngularOffset(float angularOffset) { return b2MotorJoint_SetAngularOffset(Handle(), angularOffset); }

        /// @return the maximum torque for a motor joint
        [[nodiscard]] float GetMaxTorque() const { return b2MotorJoint_GetMaxTorque(Handle()); }
    };

    /// A mouse joint is used to make a point on a body track a
    /// specified world point. This a soft constraint and allows the constraint to stretch without
    /// applying huge forces. This also applies rotation constraint heuristic to improve control.
    class MouseJoint : public Joint
    {
      protected:
        MouseJoint(OwningTag, b2JointId id) noexcept : Joint(Owning, id) {}

      public:
        // Consturcts a null (invalid) object.
        constexpr MouseJoint() {}

        // The constructor accepts either this or directly `b2MouseJointDef`.
        struct Params : b2MouseJointDef
        {
            Params() : b2MouseJointDef(b2DefaultMouseJointDef()) {}
        };

        /// Create a mouse joint
        ///	@see b2MouseJointDef for details
        MouseJoint(World &world, const std::derived_from<b2MouseJointDef> auto &params) : MouseJoint(Owning, b2CreateMouseJoint(world.Handle(), &params)) {}

        // Will act as a reference to this handle, but will not destroy it in the destructor.
        MouseJoint(RefTag, b2JointId id) noexcept : Joint(Ref, id) {}

        /// @return the target for a mouse joint
        [[nodiscard]] Vec2 GetTarget() const { return b2MouseJoint_GetTarget(Handle()); }

        /// Get the Hertz of a mouse joint
        [[nodiscard]] float GetHertz() const { return b2MouseJoint_GetHertz(Handle()); }

        /// Get the damping ratio of a mouse joint
        [[nodiscard]] float GetDampingRatio() const { return b2MouseJoint_GetDampingRatio(Handle()); }

        /// Set the target for a mouse joint
        void SetTarget(Vec2 target) { return b2MouseJoint_SetTarget(Handle(), target); }

        /// Adjust the softness parameters of a mouse joint
        void SetTuning(float hertz, float dampingRatio) { return b2MouseJoint_SetTuning(Handle(), hertz, dampingRatio); }
    };

    /// Prismatic joint definition. This requires defining a line of
    /// motion using an axis and an anchor point. The definition uses local
    /// anchor points and a local axis so that the initial configuration
    /// can violate the constraint slightly. The joint translation is zero
    /// when the local anchor points coincide in world space.
    class PrismaticJoint : public Joint
    {
      protected:
        PrismaticJoint(OwningTag, b2JointId id) noexcept : Joint(Owning, id) {}

      public:
        // Consturcts a null (invalid) object.
        constexpr PrismaticJoint() {}

        // The constructor accepts either this or directly `b2PrismaticJointDef`.
        struct Params : b2PrismaticJointDef
        {
            Params() : b2PrismaticJointDef(b2DefaultPrismaticJointDef()) {}
        };

        /// Create a prismatic (slider) joint
        ///	@see b2PrismaticJointDef for details
        PrismaticJoint(World &world, const std::derived_from<b2PrismaticJointDef> auto &params) : PrismaticJoint(Owning, b2CreatePrismaticJoint(world.Handle(), &params)) {}

        // Will act as a reference to this handle, but will not destroy it in the destructor.
        PrismaticJoint(RefTag, b2JointId id) noexcept : Joint(Ref, id) {}

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
        [[nodiscard]] Vec2 GetConstraintForce() const { return b2PrismaticJoint_GetConstraintForce(Handle()); }

        /// Get the current motor force for a prismatic joint
        [[nodiscard]] float GetMotorForce() const { return b2PrismaticJoint_GetMotorForce(Handle()); }

        /// Set the maximum force for a prismatic joint motor
        void SetMaxMotorForce(float force) { return b2PrismaticJoint_SetMaxMotorForce(Handle(), force); }
    };

    /// Revolute joint definition. This requires defining an anchor point where the
    /// bodies are joined. The definition uses local anchor points so that the
    /// initial configuration can violate the constraint slightly. You also need to
    /// specify the initial relative angle for joint limits. This helps when saving
    /// and loading a game.
    /// The local anchor points are measured from the body's origin
    /// rather than the center of mass because:
    /// 1. you might not know where the center of mass will be.
    /// 2. if you add/remove shapes from a body and recompute the mass,
    ///    the joints will be broken.
    class RevoluteJoint : public Joint
    {
      protected:
        RevoluteJoint(OwningTag, b2JointId id) noexcept : Joint(Owning, id) {}

      public:
        // Consturcts a null (invalid) object.
        constexpr RevoluteJoint() {}

        // The constructor accepts either this or directly `b2RevoluteJointDef`.
        struct Params : b2RevoluteJointDef
        {
            Params() : b2RevoluteJointDef(b2DefaultRevoluteJointDef()) {}
        };

        /// Create a revolute (hinge) joint
        ///	@see b2RevoluteJointDef for details
        RevoluteJoint(World &world, const std::derived_from<b2RevoluteJointDef> auto &params) : RevoluteJoint(Owning, b2CreateRevoluteJoint(world.Handle(), &params)) {}

        // Will act as a reference to this handle, but will not destroy it in the destructor.
        RevoluteJoint(RefTag, b2JointId id) noexcept : Joint(Ref, id) {}

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
        [[nodiscard]] Vec2 GetConstraintForce() const { return b2RevoluteJoint_GetConstraintForce(Handle()); }

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

    /// Wheel joint definition. This requires defining a line of
    /// motion using an axis and an anchor point. The definition uses local
    /// anchor points and a local axis so that the initial configuration
    /// can violate the constraint slightly. The joint translation is zero
    /// when the local anchor points coincide in world space. Using local
    /// anchors and a local axis helps when saving and loading a game.
    class WheelJoint : public Joint
    {
      protected:
        WheelJoint(OwningTag, b2JointId id) noexcept : Joint(Owning, id) {}

      public:
        // Consturcts a null (invalid) object.
        constexpr WheelJoint() {}

        // The constructor accepts either this or directly `b2WheelJointDef`.
        struct Params : b2WheelJointDef
        {
            Params() : b2WheelJointDef(b2DefaultWheelJointDef()) {}
        };

        /// Create a wheel joint
        ///	@see b2WheelJointDef for details
        WheelJoint(World &world, const std::derived_from<b2WheelJointDef> auto &params) : WheelJoint(Owning, b2CreateWheelJoint(world.Handle(), &params)) {}

        // Will act as a reference to this handle, but will not destroy it in the destructor.
        WheelJoint(RefTag, b2JointId id) noexcept : Joint(Ref, id) {}

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
        [[nodiscard]] Vec2 GetConstraintForce() const { return b2WheelJoint_GetConstraintForce(Handle()); }

        /// Enable/disable the wheel joint motor
        void EnableMotor(bool enableMotor) { return b2WheelJoint_EnableMotor(Handle(), enableMotor); }

        /// @return the wheel joint stiffness in Hertz
        [[nodiscard]] float GetSpringHertz() const { return b2WheelJoint_GetSpringHertz(Handle()); }

        /// @return the wheel joint maximum motor torque
        [[nodiscard]] float GetMaxMotorTorque() const { return b2WheelJoint_GetMaxMotorTorque(Handle()); }
    };

    /// A weld joint connect to bodies together rigidly. This constraint can be made soft to mimic
    ///	soft-body simulation.
    /// @warning the approximate solver in Box2D cannot hold many bodies together rigidly
    class WeldJoint : public Joint
    {
      protected:
        WeldJoint(OwningTag, b2JointId id) noexcept : Joint(Owning, id) {}

      public:
        // Consturcts a null (invalid) object.
        constexpr WeldJoint() {}

        // The constructor accepts either this or directly `b2WeldJointDef`.
        struct Params : b2WeldJointDef
        {
            Params() : b2WeldJointDef(b2DefaultWeldJointDef()) {}
        };

        /// Create a weld joint
        ///	@see b2WeldJointDef for details
        WeldJoint(World &world, const std::derived_from<b2WeldJointDef> auto &params) : WeldJoint(Owning, b2CreateWeldJoint(world.Handle(), &params)) {}

        // Will act as a reference to this handle, but will not destroy it in the destructor.
        WeldJoint(RefTag, b2JointId id) noexcept : Joint(Ref, id) {}

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
        void RayCast(const RayCastInput& input, uint32_t maskBits, b2TreeRayCastCallbackFcn* callback, void* context) const { return b2DynamicTree_RayCast(&value, &input, maskBits, callback, context); }

        /// Compute the height of the binary tree in O(N) time. Should not be
        /// called often.
        [[nodiscard]] int32_t GetHeight() const { return b2DynamicTree_GetHeight(&value); }

        /// Move a proxy to a new AABB by removing and reinserting into the tree.
        void MoveProxy(int32_t proxyId, AABB aabb) { return b2DynamicTree_MoveProxy(&value, proxyId, aabb); }

        /// Shift the world origin. Useful for large worlds.
        /// The shift formula is: position -= newOrigin
        /// @param newOrigin the new origin with respect to the old origin
        void ShiftOrigin(Vec2 newOrigin) { return b2DynamicTree_ShiftOrigin(&value, newOrigin); }

        /// Validate this tree. For testing.
        void Validate() const { return b2DynamicTree_Validate(&value); }

        /// Get the AABB of a proxy
        [[nodiscard]] AABB GetAABB(int32_t proxyId) const { return b2DynamicTree_GetAABB(&value, proxyId); }

        /// Build an optimal tree. Very expensive. For testing.
        void RebuildBottomUp() { return b2DynamicTree_RebuildBottomUp(&value); }

        /// Ray-cast against the proxies in the tree. This relies on the callback
        /// to perform a exact ray-cast in the case were the proxy contains a shape.
        /// The callback also performs the any collision filtering. This has performance
        /// roughly equal to k * log(n), where k is the number of collisions and n is the
        /// number of proxies in the tree.
        /// @param input the ray-cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1).
        /// @param callback a callback class that is called for each proxy that is hit by the ray.
        void ShapeCast(const ShapeCastInput& input, uint32_t maskBits, b2TreeShapeCastCallbackFcn* callback, void* context) const { return b2DynamicTree_ShapeCast(&value, &input, maskBits, callback, context); }

        /// Get the number of proxies created
        [[nodiscard]] int32_t GetProxyCount() const { return b2DynamicTree_GetProxyCount(&value); }

        /// Create a proxy. Provide a tight fitting AABB and a userData value.
        [[nodiscard]] int32_t CreateProxy(AABB aabb, uint32_t categoryBits, int32_t userData) { return b2DynamicTree_CreateProxy(&value, aabb, categoryBits, userData); }

        /// Get proxy user data
        /// @return the proxy user data or 0 if the id is invalid
        [[nodiscard]] int32_t GetUserData(int32_t proxyId) const { return b2DynamicTree_GetUserData(&value, proxyId); }

        /// Query an AABB for overlapping proxies. The callback class
        /// is called for each proxy that overlaps the supplied AABB.
        void QueryFiltered(AABB aabb, uint32_t maskBits, b2TreeQueryCallbackFcn* callback, void* context) const { return b2DynamicTree_QueryFiltered(&value, aabb, maskBits, callback, context); }

        /// Get the maximum balance of the tree. The balance is the difference in height of the two children of a node.
        [[nodiscard]] int32_t GetMaxBalance() const { return b2DynamicTree_GetMaxBalance(&value); }

        /// Query an AABB for overlapping proxies. The callback class
        /// is called for each proxy that overlaps the supplied AABB.
        void Query(AABB aabb, b2TreeQueryCallbackFcn* callback, void* context) const { return b2DynamicTree_Query(&value, aabb, callback, context); }

        /// Destroy a proxy. This asserts if the id is invalid.
        void DestroyProxy(int32_t proxyId) { return b2DynamicTree_DestroyProxy(&value, proxyId); }

        /// Rebuild the tree while retaining subtrees that haven't changed. Returns the number of boxes sorted.
        int32_t Rebuild(bool fullBuild) { return b2DynamicTree_Rebuild(&value, fullBuild); }

        /// Enlarge a proxy and enlarge ancestors as necessary.
        void EnlargeProxy(int32_t proxyId, AABB aabb) { return b2DynamicTree_EnlargeProxy(&value, proxyId, aabb); }
    };

    /// Inverse transform a point (e.g. world space to local space)
    [[nodiscard]] inline Vec2 InvTransformPoint(Transform xf, const Vec2 p) { return b2InvTransformPoint(xf, p); }

    /// Rotate a vector
    [[nodiscard]] inline Vec2 RotateVector(Rot q, Vec2 v) { return b2RotateVector(q, v); }

    /// Component-wise maximum vector
    [[nodiscard]] inline Vec2 Max(Vec2 a, Vec2 b) { return b2Max(a, b); }

    /// Compute the distance between two line segments, clamping at the end points if needed.
    [[nodiscard]] inline SegmentDistanceResult SegmentDistance(Vec2 p1, Vec2 q1, Vec2 p2, Vec2 q2) { return b2SegmentDistance(p1, q1, p2, q2); }

    /// Inverse rotate a vector
    [[nodiscard]] inline Vec2 InvRotateVector(Rot q, Vec2 v) { return b2InvRotateVector(q, v); }

    /// Transpose multiply two rotations: qT * r
    [[nodiscard]] inline Rot InvMulRot(Rot q, Rot r) { return b2InvMulRot(q, r); }

    /// Set using an angle in radians
    [[nodiscard]] inline Rot MakeRot(float angle) { return b2MakeRot(angle); }

    /// Perform a linear shape cast of shape B moving and shape A fixed. Determines the hit point, normal, and translation fraction.
    /// @returns true if hit, false if there is no hit or an initial overlap
    [[nodiscard]] inline CastOutput ShapeCast(const ShapeCastPairInput& input) { return b2ShapeCast(&input); }

    [[nodiscard]] inline Timer CreateTimer() { return b2CreateTimer(); }

    /// Component-wise clamp vector so v into the range [a, b]
    [[nodiscard]] inline Vec2 Clamp(Vec2 v, Vec2 a, Vec2 b) { return b2Clamp(v, a, b); }

    /// Compute the collision manifold between a capsule and circle
    [[nodiscard]] inline Manifold CollideCapsules(const Capsule& capsuleA, Transform xfA, const Capsule& capsuleB, Transform xfB, DistanceCache& cache) { return b2CollideCapsules(&capsuleA, xfA, &capsuleB, xfB, &cache); }

    /// Destroy a rigid body given an id. Destroys all joints attached to the body. Be careful
    ///	because this may invalidate some b2JointId that you have stored.
    /// @warning This function is locked during callbacks.
    inline void DestroyBodyAndJoints(Body& bodyId) { return b2DestroyBodyAndJoints(bodyId.Handle()); }

    /// Get a right pointing perpendicular vector. Equivalent to b2CrossVS(v, 1.0f)
    [[nodiscard]] inline Vec2 RightPerp(Vec2 v) { return b2RightPerp(v); }

    /// Compute the convex hull of a set of points. Returns an empty hull if it fails.
    /// Some failure cases:
    /// - all points very close together
    /// - all points on a line
    /// - less than 3 points
    /// - more than b2_maxPolygonVertices points
    /// This welds close points and removes collinear points.
    [[nodiscard]] inline Hull ComputeHull(const Vec2& points, int32_t count) { return b2ComputeHull(&points, count); }

    /// Validate ray cast input data (NaN, etc)
    [[nodiscard]] inline bool IsValidRay(const RayCastInput& input) { return b2IsValidRay(&input); }

    /// Is this rotation normalized?
    [[nodiscard]] inline bool IsNormalized(Rot q) { return b2IsNormalized(q); }

    /// Component-wise minimum vector
    [[nodiscard]] inline Vec2 Min(Vec2 a, Vec2 b) { return b2Min(a, b); }

    /// Compute the collision manifold between two circles.
    [[nodiscard]] inline Manifold CollideCircles(const Circle& circleA, Transform xfA, const Circle& circleB, Transform xfB) { return b2CollideCircles(&circleA, xfA, &circleB, xfB); }

    /// Vector negation
    [[nodiscard]] inline Vec2 Neg(Vec2 a) { return b2Neg(a); }

    /// Shape cast versus a convex polygon. Initial overlap is treated as a miss.
    [[nodiscard]] inline CastOutput ShapeCastPolygon(const ShapeCastInput& input, const Polygon& shape) { return b2ShapeCastPolygon(&input, &shape); }

    /// Ray cast versus polygon in shape local space. Initial overlap is treated as a miss.
    [[nodiscard]] inline CastOutput RayCastPolygon(const RayCastInput& input, const Polygon& shape) { return b2RayCastPolygon(&input, &shape); }

    /// Make an offset convex polygon from a convex hull. This will assert if the hull is not valid.
    [[nodiscard]] inline Polygon MakeOffsetPolygon(const Hull& hull, float radius, Transform transform) { return b2MakeOffsetPolygon(&hull, radius, transform); }

    /// Solve A * x = b, where b is a column vector. This is more efficient
    /// than computing the inverse in one-shot cases.
    [[nodiscard]] inline Vec2 Solve22(Mat22 A, Vec2 b) { return b2Solve22(A, b); }

    /// Get the length squared of this vector.
    [[nodiscard]] inline float LengthSquared(Vec2 v) { return b2LengthSquared(v); }

    /// Compute the collision manifold between an segment and a circle.
    [[nodiscard]] inline Manifold CollideSegmentAndCircle(const Segment& segmentA, Transform xfA, const Circle& circleB, Transform xfB) { return b2CollideSegmentAndCircle(&segmentA, xfA, &circleB, xfB); }

    /// Compute the closest points between two shapes. Supports any combination of:
    /// b2Circle, b2Polygon, b2EdgeShape. The simplex cache is input/output.
    /// On the first call set b2SimplexCache.count to zero.
    [[nodiscard]] inline DistanceOutput ShapeDistance(DistanceCache& cache, const DistanceInput& input) { return b2ShapeDistance(&cache, &input); }

    /// Compute the collision manifold between a capsule and circle
    [[nodiscard]] inline Manifold CollideCapsuleAndCircle(const Capsule& capsuleA, Transform xfA, const Circle& circleB, Transform xfB) { return b2CollideCapsuleAndCircle(&capsuleA, xfA, &circleB, xfB); }

    /// Transform a point (e.g. local space to world space)
    [[nodiscard]] inline Vec2 TransformPoint(Transform xf, const Vec2 p) { return b2TransformPoint(xf, p); }

    /// Compute the collision manifold between an segment and a capsule.
    [[nodiscard]] inline Manifold CollideSegmentAndCapsule(const Segment& segmentA, Transform xfA, const Capsule& capsuleB, Transform xfB, DistanceCache& cache) { return b2CollideSegmentAndCapsule(&segmentA, xfA, &capsuleB, xfB, &cache); }

    /// Perform the cross product on a vector and a scalar. In 2D this produces
    /// a vector.
    [[nodiscard]] inline Vec2 CrossVS(Vec2 v, float s) { return b2CrossVS(v, s); }

    /// Make a square polygon, bypassing the need for a convex hull.
    [[nodiscard]] inline Polygon MakeSquare(float h) { return b2MakeSquare(h); }

    /// Use this to initialize your filter
    [[nodiscard]] inline Filter DefaultFilter() { return b2DefaultFilter(); }

    /// Override the default assert callback.
    ///	@param assertFcn a non-null assert callback
    inline void SetAssertFcn(b2AssertFcn* assertFcn) { return b2SetAssertFcn(assertFcn); }

    /// Total bytes allocated by Box2D
    [[nodiscard]] inline uint32_t GetByteCount() { return b2GetByteCount(); }

    /// Vector dot product
    [[nodiscard]] inline float Dot(Vec2 a, Vec2 b) { return b2Dot(a, b); }

    [[nodiscard]] inline bool Vec2_IsValid(Vec2 v) { return b2Vec2_IsValid(v); }

    /// Compute the angular velocity necessary to rotate between two
    ///	rotations over a give time
    ///	@param q1 initial rotation
    ///	@param q2 final rotation
    ///	@param inv_h inverse time step
    [[nodiscard]] inline float ComputeAngularVelocity(Rot q1, Rot q2, float inv_h) { return b2ComputeAngularVelocity(q1, q2, inv_h); }

    /// Make a proxy for use in GJK and related functions.
    [[nodiscard]] inline DistanceProxy MakeProxy(const Vec2& vertices, int32_t count, float radius) { return b2MakeProxy(&vertices, count, radius); }

    [[nodiscard]] inline int64_t GetTicks(Timer& timer) { return b2GetTicks(&timer); }

    /// Multiply a 2-by-2 matrix times a 2D vector
    [[nodiscard]] inline Vec2 MulMV(Mat22 A, Vec2 v) { return b2MulMV(A, v); }

    /// Component-wise multiplication
    [[nodiscard]] inline Vec2 Mul(Vec2 a, Vec2 b) { return b2Mul(a, b); }

    /// Convert this vector into a unit vector
    [[nodiscard]] inline Vec2 Normalize(Vec2 v) { return b2Normalize(v); }

    /// Compute the collision manifold between two polygons.
    [[nodiscard]] inline Manifold CollidePolygons(const Polygon& polyA, Transform xfA, const Polygon& polyB, Transform xfB, DistanceCache& cache) { return b2CollidePolygons(&polyA, xfA, &polyB, xfB, &cache); }

    // relative angle between b and a (rot_b * inv(rot_a))
    [[nodiscard]] inline float RelativeAngle(Rot b, Rot a) { return b2RelativeAngle(b, a); }

    /// Multiply two rotations: q * r
    [[nodiscard]] inline Rot MulRot(Rot q, Rot r) { return b2MulRot(q, r); }

    /// Vector cross product. In 2D this yields a scalar.
    [[nodiscard]] inline float Cross(Vec2 a, Vec2 b) { return b2Cross(a, b); }

    /// Compute mass properties of a circle
    [[nodiscard]] inline MassData ComputeCircleMass(const Circle& shape, float density) { return b2ComputeCircleMass(&shape, density); }

    /// Get the inverse of a 2-by-2 matrix
    [[nodiscard]] inline Mat22 GetInverse22(Mat22 A) { return b2GetInverse22(A); }

    [[nodiscard]] inline float GetMillisecondsAndReset(Timer& timer) { return b2GetMillisecondsAndReset(&timer); }

    [[nodiscard]] inline float Distance(Vec2 a, Vec2 b) { return b2Distance(a, b); }

    /// Compute the collision manifold between an segment and a capsule.
    [[nodiscard]] inline Manifold CollideSmoothSegmentAndCapsule(const SmoothSegment& smoothSegmentA, Transform xfA, const Capsule& capsuleB, Transform xfB, DistanceCache& cache) { return b2CollideSmoothSegmentAndCapsule(&smoothSegmentA, xfA, &capsuleB, xfB, &cache); }

    /// Make a box (rectangle) polygon, bypassing the need for a convex hull.
    [[nodiscard]] inline Polygon MakeBox(float hx, float hy) { return b2MakeBox(hx, hy); }

    /// This asserts of the vector is too short
    [[nodiscard]] inline Vec2 NormalizeChecked(Vec2 v) { return b2NormalizeChecked(v); }

    /// Compute mass properties of a capsule
    [[nodiscard]] inline MassData ComputeCapsuleMass(const Capsule& shape, float density) { return b2ComputeCapsuleMass(&shape, density); }

    /// Multiply a scalar and vector
    [[nodiscard]] inline Vec2 MulSV(float s, Vec2 v) { return b2MulSV(s, v); }

    /// Compute mass properties of a polygon
    [[nodiscard]] inline MassData ComputePolygonMass(const Polygon& shape, float density) { return b2ComputePolygonMass(&shape, density); }

    /// Transform a polygon. This is useful for transferring a shape from one body to another.
    [[nodiscard]] inline Polygon TransformPolygon(Transform transform, const Polygon& polygon) { return b2TransformPolygon(transform, &polygon); }

    /// Compute the collision manifold between an segment and a polygon.
    [[nodiscard]] inline Manifold CollideSegmentAndPolygon(const Segment& segmentA, Transform xfA, const Polygon& polygonB, Transform xfB, DistanceCache& cache) { return b2CollideSegmentAndPolygon(&segmentA, xfA, &polygonB, xfB, &cache); }

    /// Get the length of this vector (the norm).
    [[nodiscard]] inline float Length(Vec2 v) { return b2Length(v); }

    /// Compute the collision manifold between a smooth segment and a circle.
    [[nodiscard]] inline Manifold CollideSmoothSegmentAndCircle(const SmoothSegment& smoothSegmentA, Transform xfA, const Circle& circleB, Transform xfB) { return b2CollideSmoothSegmentAndCircle(&smoothSegmentA, xfA, &circleB, xfB); }

    /// Test a point for overlap with a capsule in local space
    [[nodiscard]] inline bool PointInCapsule(Vec2 point, const Capsule& shape) { return b2PointInCapsule(point, &shape); }

    /// Compute the collision manifold between a polygon and a circle.
    [[nodiscard]] inline Manifold CollidePolygonAndCircle(const Polygon& polygonA, Transform xfA, const Circle& circleB, Transform xfB) { return b2CollidePolygonAndCircle(&polygonA, xfA, &circleB, xfB); }

    /// Shape cast versus a circle. Initial overlap is treated as a miss.
    [[nodiscard]] inline CastOutput ShapeCastCircle(const ShapeCastInput& input, const Circle& shape) { return b2ShapeCastCircle(&input, &shape); }

    [[nodiscard]] inline bool IsValid(float a) { return b2IsValid(a); }

    /// v2 = A.q' * (B.q * v1 + B.p - A.p)
    ///    = A.q' * B.q * v1 + A.q' * (B.p - A.p)
    [[nodiscard]] inline Transform InvMulTransforms(Transform A, Transform B) { return b2InvMulTransforms(A, B); }

    /// Make an offset box, bypassing the need for a convex hull.
    [[nodiscard]] inline Polygon MakeOffsetBox(float hx, float hy, Vec2 center, float angle) { return b2MakeOffsetBox(hx, hy, center, angle); }

    /// Make a color from a hex code
    [[nodiscard]] inline Color MakeColor(HexColor hexCode) { return b2MakeColor((b2HexColor)hexCode); }

    /// Vector subtraction
    [[nodiscard]] inline Vec2 Sub(Vec2 a, Vec2 b) { return b2Sub(a, b); }

    /// Get a left pointing perpendicular vector. Equivalent to b2CrossSV(1.0f, v)
    [[nodiscard]] inline Vec2 LeftPerp(Vec2 v) { return b2LeftPerp(v); }

    /// Make a rounded box, bypassing the need for a convex hull.
    [[nodiscard]] inline Polygon MakeRoundedBox(float hx, float hy, float radius) { return b2MakeRoundedBox(hx, hy, radius); }

    /// Compute the upper bound on time before two shapes penetrate. Time is represented as
    /// a fraction between [0,tMax]. This uses a swept separating axis and may miss some intermediate,
    /// non-tunneling collisions. If you change the time interval, you should call this function
    /// again.
    [[nodiscard]] inline TOIOutput TimeOfImpact(const TOIInput& input) { return b2TimeOfImpact(&input); }

    /// Shape cast versus a line segment. Initial overlap is treated as a miss.
    [[nodiscard]] inline CastOutput ShapeCastSegment(const ShapeCastInput& input, const Segment& shape) { return b2ShapeCastSegment(&input, &shape); }

    /// Use this to initialize your query filter
    [[nodiscard]] inline QueryFilter DefaultQueryFilter() { return b2DefaultQueryFilter(); }

    /// a - s * b
    [[nodiscard]] inline Vec2 MulSub(Vec2 a, float s, Vec2 b) { return b2MulSub(a, s, b); }

    /// Integration rotation from angular velocity
    ///	@param q1 initial rotation
    ///	@param deltaAngle the angular displacement in radians
    [[nodiscard]] inline Rot IntegrateRotation(Rot q1, float deltaAngle) { return b2IntegrateRotation(q1, deltaAngle); }

    /// Compute the collision manifold between a polygon and capsule
    [[nodiscard]] inline Manifold CollidePolygonAndCapsule(const Polygon& polygonA, Transform xfA, const Capsule& capsuleB, Transform xfB, DistanceCache& cache) { return b2CollidePolygonAndCapsule(&polygonA, xfA, &capsuleB, xfB, &cache); }

    /// Ray cast versus segment in shape local space. Optionally treat the segment as one-sided with hits from
    /// the left side being treated as a miss.
    [[nodiscard]] inline CastOutput RayCastSegment(const RayCastInput& input, const Segment& shape, bool oneSided) { return b2RayCastSegment(&input, &shape, oneSided); }

    /// Perform the cross product on a scalar and a vector. In 2D this produces
    /// a vector.
    [[nodiscard]] inline Vec2 CrossSV(float s, Vec2 v) { return b2CrossSV(s, v); }

    /// Vector addition
    [[nodiscard]] inline Vec2 Add(Vec2 a, Vec2 b) { return b2Add(a, b); }

    /// Compute the bounding box of a transformed capsule
    [[nodiscard]] inline AABB ComputeCapsuleAABB(const Capsule& shape, Transform transform) { return b2ComputeCapsuleAABB(&shape, transform); }

    /// Ray cast versus circle in shape local space. Initial overlap is treated as a miss.
    [[nodiscard]] inline CastOutput RayCastCircle(const RayCastInput& input, const Circle& shape) { return b2RayCastCircle(&input, &shape); }

    /// a + s * b
    [[nodiscard]] inline Vec2 MulAdd(Vec2 a, float s, Vec2 b) { return b2MulAdd(a, s, b); }

    /// This determines if a hull is valid. Checks for:
    /// - convexity
    /// - collinear points
    /// This is expensive and should not be called at runtime.
    [[nodiscard]] inline bool ValidateHull(const Hull& hull) { return b2ValidateHull(&hull); }

    /// Make a convex polygon from a convex hull. This will assert if the hull is not valid.
    [[nodiscard]] inline Polygon MakePolygon(const Hull& hull, float radius) { return b2MakePolygon(&hull, radius); }

    /// Compute the bounding box of a transformed polygon
    [[nodiscard]] inline AABB ComputePolygonAABB(const Polygon& shape, Transform transform) { return b2ComputePolygonAABB(&shape, transform); }

    /// Get the distance squared between points
    [[nodiscard]] inline float DistanceSquared(Vec2 a, Vec2 b) { return b2DistanceSquared(a, b); }

    /// Compute the bounding box of a transformed circle
    [[nodiscard]] inline AABB ComputeCircleAABB(const Circle& shape, Transform transform) { return b2ComputeCircleAABB(&shape, transform); }

    /// Component-wise absolute vector
    [[nodiscard]] inline Vec2 Abs(Vec2 a) { return b2Abs(a); }

    /// Compute the bounding box of a transformed line segment
    [[nodiscard]] inline AABB ComputeSegmentAABB(const Segment& shape, Transform transform) { return b2ComputeSegmentAABB(&shape, transform); }

    inline void SleepMilliseconds(float milliseconds) { return b2SleepMilliseconds(milliseconds); }

    [[nodiscard]] inline float GetMilliseconds(const Timer& timer) { return b2GetMilliseconds(&timer); }

    /// Normalized linear interpolation
    /// https://fgiesen.wordpress.com/2012/08/15/linear-interpolation-past-present-and-future/
    [[nodiscard]] inline Rot NLerp(Rot q1, Rot q2, float t) { return b2NLerp(q1, q2, t); }

    /// Compute the collision manifold between a smooth segment and a rounded polygon.
    [[nodiscard]] inline Manifold CollideSmoothSegmentAndPolygon(const SmoothSegment& smoothSegmentA, Transform xfA, const Polygon& polygonB, Transform xfB, DistanceCache& cache) { return b2CollideSmoothSegmentAndPolygon(&smoothSegmentA, xfA, &polygonB, xfB, &cache); }

    [[nodiscard]] inline Vec2 GetLengthAndNormalize(float& length, Vec2 v) { return b2GetLengthAndNormalize(&length, v); }

    /// Normalize rotation
    [[nodiscard]] inline Rot NormalizeRot(Rot q) { return b2NormalizeRot(q); }

    /// Shape cast versus a capsule. Initial overlap is treated as a miss.
    [[nodiscard]] inline CastOutput ShapeCastCapsule(const ShapeCastInput& input, const Capsule& shape) { return b2ShapeCastCapsule(&input, &shape); }

    /// Test a point for overlap with a circle in local space
    [[nodiscard]] inline bool PointInCircle(Vec2 point, const Circle& shape) { return b2PointInCircle(point, &shape); }

    /// This allows the user to override the allocation functions. These should be
    ///	set during application startup.
    inline void SetAllocator(b2AllocFcn* allocFcn, b2FreeFcn* freeFcn) { return b2SetAllocator(allocFcn, freeFcn); }

    /// Vector linear interpolation
    /// https://fgiesen.wordpress.com/2012/08/15/linear-interpolation-past-present-and-future/
    [[nodiscard]] inline Vec2 Lerp(Vec2 a, Vec2 b, float t) { return b2Lerp(a, b, t); }

    /// Ray cast versus capsule in shape local space. Initial overlap is treated as a miss.
    [[nodiscard]] inline CastOutput RayCastCapsule(const RayCastInput& input, const Capsule& shape) { return b2RayCastCapsule(&input, &shape); }

    /// Test a point for overlap with a convex polygon in local space
    [[nodiscard]] inline bool PointInPolygon(Vec2 point, const Polygon& shape) { return b2PointInPolygon(point, &shape); }

    /// v2 = A.q.Rot(B.q.Rot(v1) + B.p) + A.p
    ///    = (A.q * B.q).Rot(v1) + A.q.Rot(B.p) + A.p
    [[nodiscard]] inline Transform MulTransforms(Transform A, Transform B) { return b2MulTransforms(A, B); }

    /// Make a color from a hex code and alpha
    [[nodiscard]] inline Color MakeColorAlpha(HexColor hexCode, float alpha) { return b2MakeColorAlpha((b2HexColor)hexCode, alpha); }

    [[nodiscard]] inline Transform GetSweepTransform(const Sweep& sweep, float time) { return b2GetSweepTransform(&sweep, time); }
} // namespace box2d
