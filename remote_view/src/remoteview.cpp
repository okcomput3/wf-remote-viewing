/*
The MIT License (MIT)

Copyright (c) 2018 Iliya Bozhinov
Copyright (c) 2023 Andrew Pliatsikas
Copyright (c) 2023 Scott Moreau

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.*/

#include <memory>
#include <wayfire/core.hpp>
#include <wayfire/debug.hpp>
#include <wayfire/output-layout.hpp>
#include <wayfire/output.hpp>
#include <wayfire/per-output-plugin.hpp>
#include <wayfire/plugins/common/geometry-animation.hpp>
#include <wayfire/plugins/common/key-repeat.hpp>
#include <wayfire/plugins/common/shared-core-data.hpp>
#include "remoteview-move-drag-interface.hpp"
#include "remote-view-workspace-wall.hpp"
#include "ipc-activator.hpp"
#include "wayfire/plugins/common/input-grab.hpp"
#include "wayfire/plugins/common/util.hpp"
#include "wayfire/render-manager.hpp"
#include "wayfire/scene-input.hpp"
#include "wayfire/scene.hpp"
#include "wayfire/signal-definitions.hpp"
#include "wayfire/view.hpp"
#include "wayfire/opengl.hpp"
#include "wayfire/scene-render.hpp" // For render_pass

#include <linux/input-event-codes.h>

class wayfire_remoteview : public wf::per_output_plugin_instance_t,
                           public wf::keyboard_interaction_t,
                           public wf::pointer_interaction_t,
                           public wf::touch_interaction_t {
  private:
    // A constant point safely off-screen.
    static constexpr wf::point_t offscreen_point = {-10, -10};

    // Encapsulated state to avoid global variables.
    struct State {
        bool active = false;
        bool button_pressed = false;
        bool zoom_in = false;
        bool accepting_input = false;
        bool dragging_window = false;
        bool grab_check = false;
        bool main_workspace = false;
    } state;

    // Configuration options
    wf::option_wrapper_t<wf::color_t> background_color{"remoteview/background"};
    wf::option_wrapper_t<int> zoom_duration{"remoteview/duration"};
    wf::option_wrapper_t<int> delimiter_offset{"remoteview/offset"};
    wf::option_wrapper_t<bool> keyboard_interaction{"remoteview/keyboard_interaction"};
    wf::option_wrapper_t<double> inactive_brightness{"remoteview/inactive_brightness"};
    wf::option_wrapper_t<int> transition_length{"remoteview/transition_length"};
    wf::option_wrapper_t<bool> move_enable_snap_off{"move/enable_snap_off"};
    wf::option_wrapper_t<int> move_snap_off_threshold{"move/snap_off_threshold"};
    wf::option_wrapper_t<bool> move_join_views{"move/join_views"};
    wf::option_wrapper_t<wf::config::compound_list_t<wf::activatorbinding_t>>
        workspace_bindings{"remoteview/workspace_bindings"};

    wf::geometry_animation_t zoom_animation{zoom_duration};
    wf::shared_data::ref_ptr_t<wf::remoteview_move_drag::core_drag_t> drag_helper;

    std::vector<wf::activator_callback> keyboard_select_cbs;
    std::vector<wf::option_sptr_t<wf::activatorbinding_t>> keyboard_select_options;

    wf::point_t target_ws, initial_ws;
    std::unique_ptr<wf::remoteview_workspace_wall> wall;
    wf::key_repeat_t key_repeat;
    uint32_t key_pressed = 0;
    std::vector<std::vector<wf::animation::simple_animation_t>> ws_fade;
    std::unique_ptr<wf::input_grab_t> input_grab;
    int workspaceX_pos = 0;
    wf::point_t input_grab_origin;
    wf::point_t move_started_ws = offscreen_point;

    // Member to store the last drag finish position for diagnostics
    wf::point_t last_drag_finish_point = offscreen_point;
    // Member to store the calculated threshold for diagnostic drawing
    int diagnostic_threshold_x = 0;

  public:
    wf::plugin_activation_data_t grab_interface = {
        .name = "remoteview",
        .capabilities = wf::CAPABILITY_MANAGE_COMPOSITOR,
        .cancel = [=]() { finalize_and_exit(); },
    };

    void init() override {
        input_grab = std::make_unique<wf::input_grab_t>("remoteview", output, this, this, this);
        setup_workspace_bindings_from_config();
        wall = std::make_unique<wf::remoteview_workspace_wall>(this->output);
        drag_helper->connect(&on_drag_output_focus);
        drag_helper->connect(&on_drag_snap_off);
        drag_helper->connect(&on_drag_done);
        resize_ws_fade();
        output->connect(&on_workspace_grid_changed);

        output->render->add_effect(&pre_frame, wf::OUTPUT_EFFECT_PRE);
        output->render->add_effect(&post_paint_hook, wf::OUTPUT_EFFECT_OVERLAY);
    }

    bool handle_toggle() {
        if (!state.active) {
            return activate();
        }
        if (!zoom_animation.running() || state.zoom_in) {
            deactivate();
        }
        return true;
    }

    // --- Input Handling ---
    void handle_pointer_button(const wlr_pointer_button_event &event) override {
        if (event.button != BTN_LEFT) return;
        auto gc = output->get_cursor_position();
        handle_input_press({static_cast<int>(gc.x), static_cast<int>(gc.y)}, event.state);
    }

    void handle_pointer_motion(wf::pointf_t pointer_position, uint32_t time_ms) override {
        if (!state.dragging_window) {
            int threshold = get_drag_threshold_x();
            // Store the threshold for the rendering hook to use
            this->diagnostic_threshold_x = threshold;
            if ((int)pointer_position.x <= threshold) {
                input_grab->ungrab_input();
            }
            state.grab_check = false;
        }
        handle_input_move({(int)pointer_position.x, (int)pointer_position.y});
    }

    void handle_input_press(wf::point_t global_coords, uint32_t button_state) {
        if (zoom_animation.running() || !this->state.active) return;

        if (button_state == WLR_BUTTON_RELEASED) {
            state.button_pressed = false;
            if (drag_helper->view) {
                drag_helper->handle_input_released();
            } else {
                deactivate();
            }
        } else { // Button pressed
            state.button_pressed = true;
            input_grab_origin = global_coords;
            auto local = global_coords - wf::origin(output->get_layout_geometry());
            update_target_workspace(local.x, local.y);
        }
    }

    void handle_input_move(wf::point_t to) {
        // Attempt to start a drag if a click is held or was just initiated.
        bool drag_started_this_frame = false;
        if ((input_grab_origin != offscreen_point) && !zoom_animation.running()) {
            auto local_to = to - wf::origin(output->get_layout_geometry());
            // Start drag immediately if button is not pressed, otherwise wait for threshold.
            if (!state.button_pressed || abs(local_to - input_grab_origin) >= 5) {
                if (auto view = find_view_at_coordinates(input_grab_origin.x, input_grab_origin.y)) {
                    start_moving(view, input_grab_origin);
                    drag_started_this_frame = true;
                }
                input_grab_origin = offscreen_point; // Consume the start event
            }
        }

        if (state.button_pressed) {
            state.dragging_window = true;
        }

        // Update motion for any ongoing or newly started drag.
        if (drag_helper->view) {
            drag_helper->handle_motion(to);
        }

        auto local = to - wf::origin(output->get_layout_geometry());
        update_target_workspace(local.x, local.y);
    }


    // --- Plugin Activation & Teardown ---
    bool activate() {
        if (!output->activate_plugin(&grab_interface)) return false;

        input_grab->grab_input(wf::scene::layer::OVERLAY);
        state = {}; // Reset state, keeping all bools false.
        state.active = true;
        state.accepting_input = true;
        initial_ws = target_ws = output->wset()->get_current_workspace();
        last_drag_finish_point = offscreen_point; // Reset diagnostic marker
        diagnostic_threshold_x = 0; // Reset threshold marker

        _set_zoom_animation(ZoomType::In);

        wall->start_output_renderer();
        output->render->schedule_redraw();

        for (size_t i = 0; i < keyboard_select_cbs.size(); i++) {
            output->add_activator(keyboard_select_options[i], &keyboard_select_cbs[i]);
        }
        return true;
    }

    void deactivate() {
        if (!state.main_workspace && target_ws != initial_ws) {
            state.accepting_input = false;
            _set_zoom_animation(ZoomType::Out);
        } else if (state.main_workspace && !state.dragging_window && target_ws == initial_ws) {
            state.accepting_input = true;
            _set_zoom_animation(ZoomType::In);
        } else {
            _set_zoom_animation(ZoomType::Finish);
        }

        for (auto &cb : keyboard_select_cbs) {
            output->rem_binding(&cb);
        }
    }

    void finalize_and_exit() {
        state.active = false;
        if (drag_helper->view) {
            drag_helper->handle_input_released();
        }

        if (target_ws == initial_ws) {
            auto cws = output->wset()->get_current_workspace();
            output->wset()->set_workspace({cws.x, cws.y});
        } else {
            output->wset()->set_workspace({workspaceX_pos, target_ws.y});
        }

        output->deactivate_plugin(&grab_interface);
        input_grab->ungrab_input();
        wall->stop_output_renderer(true);
        output->render->schedule_redraw();
        key_repeat.disconnect();
        key_pressed = 0;

        // Re-add bindings, preserving original behavior.
        for (size_t i = 0; i < keyboard_select_cbs.size(); i++) {
            output->add_activator(keyboard_select_options[i], &keyboard_select_cbs[i]);
        }
    }

    void fini() override {
        if (state.active) {
            finalize_and_exit();
        }
        output->render->rem_effect(&pre_frame);
        output->render->rem_effect(&post_paint_hook);
    }

    // --- Core Logic & Rendering ---

    /**
     * This hook handles the plugin's core logic, like animations and state updates.
     */
    wf::effect_hook_t pre_frame = [=]() {
        workspaceX_pos = output->wset()->get_current_workspace().x;
        output->render->damage_whole();

        _cursor_pos_check(wf::get_core().get_cursor_position());

        if (zoom_animation.running()) {
            wall->set_viewport(zoom_animation);
        } else if (!state.zoom_in) {
            finalize_and_exit();
            return;
        }

        auto size = output->wset()->get_workspace_grid_size();
        for (int x = 0; x < size.width; x++) {
            for (int y = 0; y < size.height; y++) {
                auto &anim = ws_fade.at(x).at(y);
                if (anim.running()) {
                    wall->set_ws_dim({x, y}, anim);
                }
            }
        }
    };

    /**
     * This hook handles drawing the diagnostic overlays using the correct
     * high-level Wayfire rendering API.
     */
    wf::effect_hook_t post_paint_hook = [=] ()
    {
        if (!state.active) {
            return;
        }

        auto pass = output->render->get_current_pass();
        if (!pass) {
            return;
        }
        auto fbo = output->render->get_target_framebuffer();
        auto damage = output->render->get_swap_damage();

        // Draw the vertical threshold line in magenta
        if (this->diagnostic_threshold_x > 0)
        {
            auto og = output->get_layout_geometry();
            wf::geometry_t threshold_line = {this->diagnostic_threshold_x - 1, 0, 2, og.height};
            wf::region_t threshold_region(threshold_line);
            threshold_region &= damage;

            if (!threshold_region.empty()) {
                pass->add_rect({1.0, 0.0, 1.0, 0.9}, fbo, fbo.geometry, threshold_region);
            }
        }

        // Draw a red marker for the last drag finish position
        if (last_drag_finish_point != offscreen_point)
        {
            wf::geometry_t vertical_line = {last_drag_finish_point.x - 1, last_drag_finish_point.y - 10, 2, 20};
            wf::geometry_t horizontal_line = {last_drag_finish_point.x - 10, last_drag_finish_point.y - 1, 20, 2};

            wf::region_t crosshair_region;
            crosshair_region |= vertical_line;
            crosshair_region |= horizontal_line;
            crosshair_region &= damage;

            if (!crosshair_region.empty()) {
                pass->add_rect({1.0, 0.0, 0.0, 0.8}, fbo, fbo.geometry, crosshair_region);
            }
        }
    };

    /**
     * Checks cursor position to activate a grab on the edge of the screen.
     */
    void _cursor_pos_check(const wf::pointf_t& cursor_position) {
        int threshold = get_drag_threshold_x();

        if (cursor_position.x > threshold) {
            if (!state.grab_check) {
                output->activate_plugin(&grab_interface);
                input_grab->ungrab_input();
                input_grab = std::make_unique<wf::input_grab_t>("remoteview", output, this, this, this);
                input_grab->grab_input(wf::scene::layer::WORKSPACE);
                state.active = true;
                state.accepting_input = true;
                state.main_workspace = false;
                state.grab_check = true;
            }
        } else if (cursor_position.x <= threshold) {
            if (!state.grab_check) {
                input_grab->ungrab_input();
                state.main_workspace = false;
                output->deactivate_plugin(&grab_interface);
                state.active = true;
                state.accepting_input = true;
            }
        }
    }


    // --- View & Workspace Management ---

void start_moving(wayfire_toplevel_view view, wf::point_t grab) {
    if (!(view->get_allowed_actions() & (wf::VIEW_ALLOW_WS_CHANGE | wf::VIEW_ALLOW_MOVE))) {
        return;
    }

    auto ws_coords = input_coordinates_to_output_local_coordinates(grab);
    auto bbox = wf::view_bounding_box_up_to(view, "wobbly");

    view->damage();
    translate_wobbly(view, grab - ws_coords);

    auto [vw, vh] = output->wset()->get_workspace_grid_size();
    wf::remoteview_move_drag::drag_options_t opts;
    // Scale the initial scale by WORKSPACE_SCALE
    opts.initial_scale = std::max(vw, vh) / WORKSPACE_SCALE;
    opts.enable_snap_off = move_enable_snap_off && (view->pending_fullscreen() || view->pending_tiled_edges());
    opts.snap_off_threshold = move_snap_off_threshold;
    opts.join_views = move_join_views;

    auto output_offset = wf::origin(output->get_layout_geometry());
    drag_helper->start_drag(view, grab + output_offset,
                            wf::remoteview_move_drag::find_relative_grab(bbox, ws_coords), opts);
    move_started_ws = target_ws;
    input_grab->set_wants_raw_input(true);
}

void update_target_workspace(int x, int y) {
    auto og = output->get_layout_geometry();
    input_coordinates_to_global_coordinates(x, y);

    if (x >= 0) { // Cursor is over the workspace thumbnail grid.
        state.main_workspace = false;
        auto [vw, vh] = output->wset()->get_workspace_grid_size();
        // Scale the view down to fit in the workspace thumbnail, accounting for WORKSPACE_SCALE
        drag_helper->set_scale(std::max(vw, vh) / WORKSPACE_SCALE);
        input_grab->set_wants_raw_input(true);

        auto grid = get_grid_geometry();
        if (!(grid & wf::point_t{x, y})) return;

        target_ws = {x / og.width, y / og.height};
    } else { // Cursor is over the main desktop preview.
        state.main_workspace = true;
        // Scale the view back to its normal size (scale of 1.0) for the desktop.
        drag_helper->set_scale(1.0);
        input_grab->set_wants_raw_input(true);
    }
}

    // --- Coordinate Transformations ---

void input_coordinates_to_global_coordinates(int &sx, int &sy) {
    auto og = output->get_layout_geometry();
    auto size = output->get_screen_size();
    auto wsize = output->wset()->get_workspace_grid_size();

    float max_dim = std::max(wsize.width, wsize.height);
    float grid_start_x = get_drag_threshold_x();
    
    // Adjust Y-axis start based on WORKSPACE_SCALE
    // When WORKSPACE_SCALE = 0.5, grid is 50% height, so start at 25% (centered)
    // When WORKSPACE_SCALE = 0.75, grid is 75% height, so start at 12.5% (centered)
    float y_offset = og.height * (1.0f - WORKSPACE_SCALE) / 2.0f;
    float grid_start_y = (og.height * (max_dim - wsize.height) / max_dim / 2.0) + y_offset;

    // Apply WORKSPACE_SCALE to the coordinate transformation
    sx = (sx - grid_start_x) * max_dim / WORKSPACE_SCALE;
    sy = (sy - grid_start_y) * max_dim / WORKSPACE_SCALE;
}

    wf::point_t input_coordinates_to_output_local_coordinates(wf::point_t ip) {
        input_coordinates_to_global_coordinates(ip.x, ip.y);
        auto cws = output->wset()->get_current_workspace();
        auto og = output->get_relative_geometry();
        return {ip.x - cws.x * og.width, ip.y - cws.y * og.height};
    }

    wayfire_toplevel_view find_view_at_coordinates(int gx, int gy) {
        auto local = input_coordinates_to_output_local_coordinates({gx, gy});
        return wf::find_output_view_at(output, {(double)local.x, (double)local.y});
    }

  private:
    /**
     * NEW: Centralized function to calculate the drag activation threshold.
     */
int get_drag_threshold_x()
{
    auto size = output->get_screen_size();
    auto wsize = output->wset()->get_workspace_grid_size();
    auto og = output->get_layout_geometry();
    
    if (wsize.height == 0) {
        return 0;
    }
    
    float max_dim = std::max(wsize.width, wsize.height);
    
    // Your original formula
    float grid_start_x = (og.width * (max_dim - wsize.width + ((wsize.width / (float)wsize.width) * (wsize.width - 1))) / max_dim / 2.0) +
                         size.width / 2.0 - (size.width / (float)wsize.height / 2.0);
    
    // Fullscreen width minus original threshold, then add 25% of that difference
    float offset = (og.width - grid_start_x) * (1.0f - WORKSPACE_SCALE);
    
    return grid_start_x + offset;
}
    // --- Zoom Animation Logic ---

    // Enum to control which zoom animation to run.
    enum class ZoomType { In, Out, Finish };

    // Helper struct to cache calculations for zoom animations.
    struct ZoomCalculations {
        const wf::dimensions_t wsize;
        const wf::dimensions_t screen_size;
        const int deskstopsX, deskstopsY, gap, fullw, fullh;

        ZoomCalculations(wf::output_t *output) :
            wsize(output->wset()->get_workspace_grid_size()),
            screen_size(output->get_screen_size()),
            deskstopsX(wsize.width),
            deskstopsY(wsize.height),
            gap(0), // Value from original code
            fullw((gap + screen_size.width) * deskstopsY + gap),
            fullh((gap + screen_size.height) * deskstopsY + gap) {}
    };

    /**
     * A single, unified function to set up and start all zoom animations.
     */
    void _set_zoom_animation(ZoomType type) {
        wall->set_background_color(background_color);
        wall->set_gap_size(delimiter_offset);
        state.zoom_in = (type == ZoomType::In);

        const ZoomCalculations calc(output);
        const auto base_rect = wall->get_wall_rectangle();
        wf::geometry_t start_rect, end_rect;

        const int common_y_offset = (calc.fullh - base_rect.height) / 2;
        wf::geometry_t intermediate_rect = base_rect;
        intermediate_rect.x -= ((calc.fullw - base_rect.width + ((base_rect.width) * (calc.deskstopsX - 1) / calc.deskstopsX)) / 2);
        intermediate_rect.y -= common_y_offset;
        intermediate_rect.width = calc.fullw;
        intermediate_rect.height = calc.fullh;

        switch (type) {
        case ZoomType::In:
            start_rect = intermediate_rect;
            start_rect.x -= calc.screen_size.width;
            end_rect = intermediate_rect;
            break;
        case ZoomType::Out:
            start_rect = intermediate_rect;
            end_rect = wall->get_workspace_rectangle(target_ws);
            break;
        case ZoomType::Finish:
            start_rect = end_rect = base_rect;
            const int finish_x_offset_base = (calc.fullw - base_rect.width) / 2;
            const int finish_x_offset_extra = ((calc.screen_size.width + calc.wsize.width) * (calc.deskstopsX - 1) / 2);

            start_rect.x -= finish_x_offset_base + calc.screen_size.width + calc.wsize.width + finish_x_offset_extra;
            end_rect.x -= finish_x_offset_base + finish_x_offset_extra;

            start_rect.y -= common_y_offset;
            end_rect.y -= common_y_offset;

            start_rect.width = end_rect.width = calc.fullw;
            start_rect.height = end_rect.height = calc.fullh;
            break;
        }

        zoom_animation.set_start(start_rect);
        zoom_animation.set_end(end_rect);
        zoom_animation.start();
        wall->set_viewport(zoom_animation);
    }

    // --- Signal Handlers & Callbacks ---
   wf::signal::connection_t<wf::remoteview_move_drag::drag_focus_output_signal>
    on_drag_output_focus = [=](auto ev) {
    if ((ev->focus_output == output) && output->is_plugin_active(grab_interface.name)) {
        state.button_pressed = true;
        auto [vw, vh] = output->wset()->get_workspace_grid_size();
        // Apply WORKSPACE_SCALE to the drag scale
        drag_helper->set_scale(std::max(vw, vh) / WORKSPACE_SCALE);
        input_grab->set_wants_raw_input(true);
        state.dragging_window = true;
    }
};

    wf::signal::connection_t<wf::remoteview_move_drag::snap_off_signal>
    on_drag_snap_off = [=](auto ev) {
        if ((ev->focus_output == output) && output->is_plugin_active(grab_interface.name)) {
            wf::remoteview_move_drag::adjust_view_on_snap_off(drag_helper->view);
        }
        state.dragging_window = false;
    };

    wf::signal::connection_t<wf::remoteview_move_drag::drag_done_signal>
    on_drag_done = [=](auto ev) {
        state.dragging_window = false;
        if ((ev->focused_output == output) && output->is_plugin_active(grab_interface.name) &&
            !drag_helper->is_view_held_in_place()) {
            bool same_output = ev->main_view->get_output() == output;
            auto offset = wf::origin(output->get_layout_geometry());
            auto local = input_coordinates_to_output_local_coordinates(ev->grab_position + -offset);

            this->last_drag_finish_point = ev->grab_position - offset;

            for (auto &v : wf::remoteview_move_drag::get_target_views(ev->main_view, ev->join_views)) {
                translate_wobbly(v, local - (ev->grab_position - offset));
            }
            if (!state.main_workspace) {
                ev->grab_position = local + offset;
            }
            wf::remoteview_move_drag::adjust_view_on_output(ev);

            if (same_output && (move_started_ws != offscreen_point)) {
                wf::view_change_workspace_signal data{ev->main_view, move_started_ws, target_ws};
                output->emit(&data);
            }
            move_started_ws = offscreen_point;
        }
        input_grab->set_wants_raw_input(false);
        this->state.button_pressed = false;
    };

    wf::signal::connection_t<wf::workspace_grid_changed_signal>
    on_workspace_grid_changed = [=](auto) {
        resize_ws_fade();
        auto size = this->output->wset()->get_workspace_grid_size();
        initial_ws.x = std::min(initial_ws.x, size.width - 1);
        initial_ws.y = std::min(initial_ws.y, size.height - 1);
        target_ws.x = std::min(target_ws.x, size.width - 1);
        target_ws.y = std::min(target_ws.y, size.height - 1);
    };

    // --- Misc Helpers ---
    void setup_workspace_bindings_from_config() {
        for (const auto &[workspace, binding] : workspace_bindings.value()) {
            int ws_idx = atoi(workspace.c_str());
            auto wsize = output->wset()->get_workspace_grid_size();
            if ((ws_idx < 1) || (ws_idx > wsize.width * wsize.height)) continue;

            keyboard_select_options.push_back(wf::create_option(binding));
            keyboard_select_cbs.push_back([=](auto) {
                if (state.active && (!zoom_animation.running() || state.zoom_in)) {
                    deactivate();
                }
                return state.active;
            });
        }
    }

    void resize_ws_fade() {
        auto size = this->output->wset()->get_workspace_grid_size();
        ws_fade.resize(size.width);
        for (auto &col : ws_fade) {
            col.resize(size.height, wf::animation::simple_animation_t{transition_length});
        }
    }

    wf::geometry_t get_grid_geometry() {
        auto wsize = output->wset()->get_workspace_grid_size();
        auto full_g = output->get_layout_geometry();
        return {0, 0, full_g.width * wsize.width, full_g.height * wsize.height};
    }
};

class wayfire_remoteview_global : public wf::plugin_interface_t,
                                   public wf::per_output_tracker_mixin_t<wayfire_remoteview> {
    wf::ipc_activator_t toggle_binding{"remoteview/toggle"};
    wf::ipc_activator_t::handler_t toggle_cb =
        [=](wf::output_t *output, wayfire_view) {
        return this->output_instance[output]->handle_toggle();
    };

  public:
    void init() override {
        this->init_output_tracking();
        toggle_binding.set_handler(toggle_cb);
    }
    void fini() override { this->fini_output_tracking(); }
};

DECLARE_WAYFIRE_PLUGIN(wayfire_remoteview_global);