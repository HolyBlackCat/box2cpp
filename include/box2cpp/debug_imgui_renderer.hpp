#pragma once

#include <box2cpp/box2c.hpp>
#include <box2d/debug_draw.h>
#include <imgui.h>

namespace b2
{
    // A debug ImGui renderer.
    // You need to have ImGui already configured.
    // Usage: (each of those is optional)
    // * Set `.camera_scale`, `.camera_pos`, `.camera_rot`, `.y_axis_is_upwards`  for your preferred camera orientation.
    //   Camera orientation has viable defaults for running simple examples.
    // * Call `.DrawShapes(world);` every frame to render the shapes.
    // * Call `.DrawModeToggles();` every frame to draw checkboxes for tweaking shape visualization.
    // * Call `.MouseDrag(world);` every frame to enable dragging shapes with mouse. Also draws a checkbox to enable/disable it.
    class DebugImguiRenderer
    {
      public:
        b2DebugDraw callbacks{};

        // Basic settings:

        // Camera position. This box2d position maps to the screen center.
        // Can grab this from a body you want to follow, via `.GetPosition()`.
        b2Vec2 camera_pos{};
        // Camera scale. Screen pixels per 1 box2d unit.
        float camera_scale = 32;

        // Camera rotation. The vector `(c,s)` is the box2d direction that maps to the +X screen direction.
        // Can grab this from a body you want to follow, via `.GetRotation()`.
        b2Rot camera_rot = b2Rot{.c = 1, .s = 0};
        // If true, the Y axis is inverted and points upwards instead of downwards.
        float y_axis_is_upwards = false;

        // Drag bodies with mouse:

        // If true, enable dragging shapes with the mouse. Must call `MouseDrag()` to do this, which also shows a checkbox for this value.
        bool enable_mouse_drag = false;
        // Which button to use for mouse dragging.
        ImGuiMouseButton mouse_drag_button = ImGuiMouseButton_Left;

        // Render settings:

        // This thickness is used for all lines.
        float line_thickness = 1;

        // The global alpha for all shapes.
        float shape_alpha = 1;
        // Alpha for filling shapes, multiplied by `shape_alpha`.
        float shape_fill_alpha_factor = 0.5f;

        // This is used as axis length for transforms. In ImGui pixels.
        float transform_axis_length = 16;
        // Axis colors for transforms.
        ImVec4 transform_color_x = ImVec4(1, 0, 0, 1);
        ImVec4 transform_color_y = ImVec4(0, 1, 0, 1);

        // Text color.
        ImVec4 text_color = ImVec4(1, 1, 1, 1);
        // Text background color.
        ImVec4 text_bg_color = ImVec4(0, 0, 0, 0.5f);
        // Make background behind text larger than the text by this amount.
        ImVec2 text_background_padding = ImVec2(2, 2);

        DebugImguiRenderer()
        {
            callbacks.context = this;

            callbacks.drawShapes = true;
            callbacks.drawJoints = true;
            callbacks.drawJointExtras = false;
            callbacks.drawAABBs = false;
            callbacks.drawMass = true;
            callbacks.drawContacts = false;
            callbacks.drawGraphColors = false;
            callbacks.drawContactNormals = false;
            callbacks.drawContactImpulses = false;
            callbacks.drawFrictionImpulses = false;

            callbacks.DrawPolygon = [](const b2Vec2* vertices, int vertexCount, b2Color color, void* context)
            {
                auto &self = *static_cast<DebugImguiRenderer *>(context);
                self.DrawPolygon(vertices, vertexCount, color, 0);
            };

            callbacks.DrawSolidPolygon = [](const b2Vec2* vertices, int vertexCount, b2Color color, void* context)
            {
                auto &self = *static_cast<DebugImguiRenderer *>(context);
                self.DrawPolygonFilled(vertices, vertexCount, color, 0);
            };

            callbacks.DrawRoundedPolygon = [](const b2Vec2* vertices, int vertexCount, float radius, b2Color color, void* context)
            {
                auto &self = *static_cast<DebugImguiRenderer *>(context);
                self.DrawPolygonFilled(vertices, vertexCount, color, radius);
            };

            callbacks.DrawCircle = [](b2Vec2 center, float radius, b2Color color, void* context)
            {
                auto &self = *static_cast<DebugImguiRenderer *>(context);
                self.DrawList().AddCircle(self.Box2dToImguiPoint(center), self.Box2dToImguiLength(radius), self.ShapeColorToImguiColor(color, false), 0, self.line_thickness);
            };

            callbacks.DrawSolidCircle = [](b2Vec2 center, float radius, b2Vec2 axis, b2Color color, void* context)
            {
                auto &self = *static_cast<DebugImguiRenderer *>(context);
                self.DrawList().AddCircleFilled(self.Box2dToImguiPoint(center), self.Box2dToImguiLength(radius), self.ShapeColorToImguiColor(color, true));
                self.DrawList().AddCircle(self.Box2dToImguiPoint(center), self.Box2dToImguiLength(radius), self.ShapeColorToImguiColor(color, false), 0, self.line_thickness);

                self.DrawLine(center, b2Vec2(center.x + axis.x, center.y + axis.y), color);
            };

            callbacks.DrawCapsule = [](b2Vec2 p1, b2Vec2 p2, float radius, b2Color color, void* context)
            {
                auto &self = *static_cast<DebugImguiRenderer *>(context);
                self.DrawCapsule(p1, p2, radius, color);
            };

            callbacks.DrawSolidCapsule = [](b2Vec2 p1, b2Vec2 p2, float radius, b2Color color, void* context)
            {
                auto &self = *static_cast<DebugImguiRenderer *>(context);
                self.DrawCapsuleFilled(p1, p2, radius, color);
            };

            callbacks.DrawSegment = [](b2Vec2 p1, b2Vec2 p2, b2Color color, void* context)
            {
                auto &self = *static_cast<DebugImguiRenderer *>(context);
                self.DrawLine(p1, p2, color);
            };

            callbacks.DrawTransform = [](b2Transform xf, void* context)
            {
                auto &self = *static_cast<DebugImguiRenderer *>(context);
                ImVec2 p = self.Box2dToImguiPoint(xf.p);
                ImVec2 d = self.Box2dToImguiDir(b2Vec2(xf.q.c, xf.q.s));
                float dlen = std::sqrt(d.x * d.x + d.y * d.y);
                d.x = d.x / dlen * self.transform_axis_length;
                d.y = d.y / dlen * self.transform_axis_length;
                ImVec2 d2 = self.Box2dToImguiDir(b2Vec2(-xf.q.s, xf.q.c));
                d2.x = d2.x / dlen * self.transform_axis_length;
                d2.y = d2.y / dlen * self.transform_axis_length;
                self.DrawList().AddLine(ImVec2(p.x - 0.5f, p.y - 0.5f), ImVec2(p.x + d2.x - 0.5f, p.y + d2.y - 0.5f), ImGui::ColorConvertFloat4ToU32(self.transform_color_y), self.line_thickness);
                self.DrawList().AddLine(ImVec2(p.x - 0.5f, p.y - 0.5f), ImVec2(p.x + d.x - 0.5f, p.y + d.y - 0.5f), ImGui::ColorConvertFloat4ToU32(self.transform_color_x), self.line_thickness);
            };

            /// Draw a point.
            callbacks.DrawPoint = [](b2Vec2 p, float size, b2Color color, void* context)
            {
                auto &self = *static_cast<DebugImguiRenderer *>(context);
                // Using `size` as is, it seems to be in pixels.
                self.DrawList().AddCircleFilled(self.Box2dToImguiPoint(p), size * self.line_thickness, self.ShapeColorToImguiColor(color, true/*sic, looks better*/));
            };

            /// Draw a string.
            callbacks.DrawString = [](b2Vec2 p, const char* s, void* context)
            {
                auto &self = *static_cast<DebugImguiRenderer *>(context);

                ImVec2 text_pos = self.Box2dToImguiPoint(p);
                text_pos.x = std::round(text_pos.x);
                text_pos.y = std::round(text_pos.y);

                // If `s` starts with spaces, strip them and move the text position.
                if (*s == ' ')
                {
                    float step = ImGui::CalcTextSize(" ").x;
                    while (*s == ' ')
                    {
                        text_pos.x += step;
                        s++;
                    }
                }

                ImVec2 text_size = ImGui::CalcTextSize(s);
                self.DrawList().AddRectFilled(
                    ImVec2(text_pos.x - self.text_background_padding.x, text_pos.y - self.text_background_padding.y),
                    ImVec2(text_pos.x + self.text_background_padding.x + text_size.x, text_pos.y + self.text_background_padding.y + text_size.y),
                    ImGui::ColorConvertFloat4ToU32(self.text_bg_color),
                    (self.text_background_padding.x + self.text_background_padding.y) / 2 // Guess a decent rounding.
                );
                self.DrawList().AddText(text_pos, ImGui::ColorConvertFloat4ToU32(self.text_color), s);
            };
        }

        DebugImguiRenderer(const DebugImguiRenderer &other) : callbacks(other.callbacks) {callbacks.context = this;}
        DebugImguiRenderer &operator=(const DebugImguiRenderer &other) {callbacks = other.callbacks; callbacks.context = this; return *this;}

        // Draw all the shapes using ImGui.
        void DrawShapes(b2::WorldRef world)
        {
            world.Draw(callbacks);
        }

        // Draw checkboxes for toggling various shapes.
        void DrawModeToggles()
        {
            ImGui::Begin("box2d debug render");

            ImGui::Checkbox("Shapes", &callbacks.drawShapes);
            ImGui::Checkbox("Joints", &callbacks.drawJoints);
            ImGui::Checkbox("Joint extras", &callbacks.drawJointExtras);
            ImGui::Checkbox("AABBs", &callbacks.drawAABBs);
            ImGui::Checkbox("Mass", &callbacks.drawMass);
            ImGui::Checkbox("Contacts", &callbacks.drawContacts);
            ImGui::Checkbox("Graph colors", &callbacks.drawGraphColors);
            ImGui::Checkbox("Contact normals", &callbacks.drawContactNormals);
            ImGui::Checkbox("Contact impulses", &callbacks.drawContactImpulses);
            ImGui::Checkbox("Friction impulses", &callbacks.drawFrictionImpulses);
            ImGui::Separator(); // In case we draw multiple windows.

            ImGui::End();
        }

        // Perform mouse drag and draw a checkbox to enable/disable it.
        void MouseDrag(b2::WorldRef world)
        {
            // Draw toggle.
            ImGui::Begin("box2d debug render");

            ImGui::Checkbox("Mouse drag", &enable_mouse_drag);
            ImGui::Separator(); // In case we draw multiple windows.

            ImGui::End();

            // Initiate.
            if (enable_mouse_drag && !mouse_joint && ImGui::IsMouseClicked(mouse_drag_button))
            {
                b2Vec2 point = ImguiToBox2dPoint(ImGui::GetMousePos());
                b2::ShapeRef target;
                world.Overlap(b2Circle{.center = {}, .radius = 0}, b2Transform{.p = point, .q = {0,1}}, b2DefaultQueryFilter(),
                    [&](b2::ShapeRef shape)
                    {
                        target = shape;
                        return false;
                    }
                );

                if (target && target.GetBody().GetType() == b2_dynamicBody)
                {
                    mouse_joint_body = world.CreateBody(b2::OwningHandle, b2::Body::Params{});
                    b2::MouseJoint::Params joint_params{};
                    joint_params.bodyIdA = mouse_joint_body;
                    joint_params.bodyIdB = target.GetBody();
                    joint_params.target = point;
                    mouse_joint = world.CreateJoint(b2::DestroyWithParent, joint_params);
                }
            }
            // Finish.
            if (mouse_joint && !ImGui::IsMouseDown(mouse_drag_button))
            {
                mouse_joint = {};
                mouse_joint_body = {};
            }
            // Continue drag.
            if (mouse_joint)
            {
                mouse_joint.SetTarget(ImguiToBox2dPoint(ImGui::GetMousePos()));
                mouse_joint.GetBodyB().SetAwake(true); // Poke the body to stop it from sleeping.
                ImGui::SetNextFrameWantCaptureMouse(true);
            }
        }

        // Coordinate conversions:

        ImVec2 Box2dToImguiDir(b2Vec2 dir) const
        {
            float sign = y_axis_is_upwards ? -1 : 1;
            ImVec2 ret(dir.x * camera_rot.c + dir.y * camera_rot.s, (-dir.x * camera_rot.s + dir.y * camera_rot.c) * sign);
            ret.x *= camera_scale;
            ret.y *= camera_scale;
            return ret;
        }
        ImVec2 Box2dToImguiPoint(b2Vec2 point) const
        {
            point.x -= camera_pos.x;
            point.y -= camera_pos.y;
            ImVec2 ret = Box2dToImguiDir(point);
            ret.x += ImGui::GetIO().DisplaySize.x / 2;
            ret.y += ImGui::GetIO().DisplaySize.y / 2;
            return ret;
        }
        float Box2dToImguiLength(float len) const
        {
            return len * camera_scale;
        }

        // ImGui point back to box2d.
        b2Vec2 ImguiToBox2dPoint(ImVec2 point) const
        {
            point.x -= ImGui::GetIO().DisplaySize.x / 2;
            point.y -= ImGui::GetIO().DisplaySize.y / 2;
            point.x /= camera_scale;
            point.y /= camera_scale;
            float sign = y_axis_is_upwards ? -1 : 1;
            b2Vec2 ret(point.x * camera_rot.c - point.y * camera_rot.s * sign, point.x * camera_rot.s + point.y * camera_rot.c * sign);
            ret.x += camera_pos.x;
            ret.y += camera_pos.y;
            return ret;
        }

      protected:
        // Into which draw list to render.
        ImDrawList &DrawList() const
        {
            return *ImGui::GetBackgroundDrawList();
        }

        ImU32 ShapeColorToImguiColor(const b2Color &c, bool fill) const
        {
            return ImGui::ColorConvertFloat4ToU32(ImVec4(c.r, c.g, c.b, c.a * (fill ? shape_fill_alpha_factor : 1) * shape_alpha));
        }

        // Get i-th point in a list, but possibly backwards to allow flipping the coordinate system.
        b2Vec2 GetPolygonVertex(const b2Vec2 *data, int size, int i) const
        {
            (void)size;
            if (y_axis_is_upwards)
                return data[size - 1 - i];
            else
                return data[i];
        }

        // Drawing helpers.

        void DrawLine(b2Vec2 p1, b2Vec2 p2, b2Color color) const
        {
            ImVec2 a = Box2dToImguiPoint(p1); a.x -= 0.5f; a.y -= 0.5f; // Counteract the coordinate adjustment in `AddLine()`.
            ImVec2 b = Box2dToImguiPoint(p2); b.x -= 0.5f; b.y -= 0.5f;
            DrawList().AddLine(a, b, ShapeColorToImguiColor(color, false), line_thickness);
        }

        void PushPolygonPoints(const b2Vec2* vertices, int vertexCount, float radius) const
        {
            if (radius <= 0)
            {
                for (int i = 0; i < vertexCount; i++)
                    DrawList().PathLineTo(Box2dToImguiPoint(GetPolygonVertex(vertices, vertexCount, i)));
            }
            else
            {
                ImVec2 prev_point;
                float prev_angle = 0;
                for (int i = 0; i < vertexCount + 2; i++)
                {
                    ImVec2 point = Box2dToImguiPoint(GetPolygonVertex(vertices, vertexCount, i % vertexCount));

                    if (i != 0)
                    {
                        float angle = std::atan2(point.y - prev_point.y, point.x - prev_point.x);
                        if (i != 1)
                        {
                            if (angle < prev_angle)
                                angle += f_pi * 2;
                            DrawList().PathArcTo(prev_point, Box2dToImguiLength(radius), prev_angle - f_pi / 2, angle - f_pi / 2);
                        }
                        prev_angle = angle;
                    }

                    prev_point = point;
                }
            }
        }
        void DrawPolygon(const b2Vec2* vertices, int vertexCount, b2Color color, float radius) const
        {
            PushPolygonPoints(vertices, vertexCount, radius);
            DrawList().PathStroke(ShapeColorToImguiColor(color, false), ImDrawFlags_Closed, line_thickness);
        }
        void DrawPolygonFilled(const b2Vec2* vertices, int vertexCount, b2Color color, float radius) const
        {
            PushPolygonPoints(vertices, vertexCount, radius);
            DrawList().PathFillConvex(ShapeColorToImguiColor(color, true));
            DrawPolygon(vertices, vertexCount, color, radius);
        }

        void PushCapsulePoints(b2Vec2 p1, b2Vec2 p2, float radius) const
        {
            b2Vec2 points[] = {p1, p2};
            PushPolygonPoints(points, 2, radius);
        }
        void DrawCapsule(b2Vec2 p1, b2Vec2 p2, float radius, b2Color color) const
        {
            PushCapsulePoints(p1, p2, radius);
            DrawList().PathStroke(ShapeColorToImguiColor(color, false), ImDrawFlags_Closed, line_thickness);
            DrawLine(p1, p2, color);
        }
        void DrawCapsuleFilled(b2Vec2 p1, b2Vec2 p2, float radius, b2Color color) const
        {
            PushCapsulePoints(p1, p2, radius);
            DrawList().PathFillConvex(ShapeColorToImguiColor(color, true));
            DrawCapsule(p1, p2, radius, color);
        }

        // Mouse drag state:
        b2::MouseJointRef mouse_joint;
        b2::Body mouse_joint_body;
    };
}
