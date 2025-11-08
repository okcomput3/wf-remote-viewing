#pragma once

#include <any>
#include <cstdlib>
#include <glm/gtc/matrix_transform.hpp>
#include <memory>
#include "wayfire/core.hpp"
#include "wayfire/debug.hpp"
#include "wayfire/geometry.hpp"
#include "wayfire/opengl.hpp"
#include "wayfire/region.hpp"
#include "wayfire/render-manager.hpp"
#include "wayfire/scene-input.hpp"
#include "wayfire/scene-operations.hpp"
#include "wayfire/scene-render.hpp"
#include "wayfire/scene.hpp"
#include "wayfire/signal-definitions.hpp"
#include "wayfire/signal-provider.hpp"
#include "wayfire/workspace-stream.hpp"
#include "wayfire/workspace-set.hpp"
#include "remoteview.hpp"

int workspaceX_pos=0;

// Global scale factor - change this to scale everything
 // 50% scale

namespace wf
{
    wf::geometry_t add_offset_to_target(const wf::geometry_t& target, int offset_x, int offset_y)
    {
        wf::geometry_t result = target;
        result.x += offset_x;
        result.y += offset_y;
        return result;
    }

    wf::region_t add_offset_to_workspace_rect(const wf::region_t& damage,
                                              int offset_x, int offset_y)
    {
        wf::region_t adjusted_damage;

        for (auto& rect : damage)
        {
            wf::geometry_t adjusted_box = {
                .x      = rect.x1 + offset_x,
                .y      = rect.y1 + offset_y,
                .width  = rect.x2 - rect.x1,
                .height = rect.y2 - rect.y1,
            };

            adjusted_damage |= adjusted_box;
        }

        return adjusted_damage;
    }

    struct wall_frame_event_t
    {
        const wf::render_target_t& target;
        wall_frame_event_t(const wf::render_target_t& t) : target(t)
        {}
    };

    class remoteview_workspace_wall : public wf::signal::provider_t
    {
    public:
        remoteview_workspace_wall(wf::output_t *_output) : output(_output)
        {
            this->viewport = get_wall_rectangle();
        }

        ~remoteview_workspace_wall()
        {
            stop_output_renderer(false);
        }

        void set_background_color(const wf::color_t& color)
        {
            this->background_color = color;
        }

        void set_gap_size(int size)
        {
            this->gap_size = size;
        }

     

        void set_viewport(const wf::geometry_t& viewport_geometry)
        {
            this->viewport = viewport_geometry;
            if (render_node)
            {
                scene::damage_node(this->render_node,
                    this->render_node->get_bounding_box());
            }
        }

        void render_wall(const wf::render_target_t& fb, const wf::region_t& damage)
        {
            wall_frame_event_t data{fb};
            this->emit(&data);
        }

        void start_output_renderer()
        {
            wf::dassert(render_node == nullptr, "Starting workspace-wall twice?");
            render_node = std::make_shared<workspace_wall_node_second_t>(this);
            scene::add_front(wf::get_core().scene(), render_node);
        }

        void stop_output_renderer(bool reset_viewport)
        {
            if (!render_node)
            {
                return;
            }

            scene::remove_child(render_node);
            render_node = nullptr;

            if (reset_viewport)
            {
                set_viewport({0, 0, 0, 0});
            }
        }

      wf::geometry_t get_workspace_rectangle(const wf::point_t& ws) const
{
    auto size = this->output->get_screen_size();
    
    // Apply scale to workspace dimensions
    int scaled_width = static_cast<int>(size.width * WORKSPACE_SCALE);
    int scaled_height = static_cast<int>(size.height * WORKSPACE_SCALE);
    
    // Right-align: each workspace's right edge should be (ws.x + 1) * output_size.width - scaled_width; at (ws.x + 1) * size.width
    int right_edge = (ws.x + 1) * size.width / WORKSPACE_SCALE + size.width * (-2.75 * (WORKSPACE_SCALE*WORKSPACE_SCALE) + 4.625 * WORKSPACE_SCALE - 1.875);
    int x_pos = right_edge - scaled_width;

    return {
        x_pos + ws.x,
        ws.y * (scaled_height + gap_size),
        scaled_width,
        scaled_height
    };
}

        wf::geometry_t get_wall_rectangle() const
        {
            auto size = this->output->get_screen_size();
            auto workspace_size = this->output->wset()->get_workspace_grid_size();
            
            // Apply scale to wall dimensions
            int scaled_width = static_cast<int>(size.width * WORKSPACE_SCALE);
            int scaled_height = static_cast<int>(size.height * WORKSPACE_SCALE);

            return {
                0,  // Start at 0 to keep left-aligned
                0,  // Start at 0 to keep top-aligned
                workspace_size.width * (scaled_width + gap_size),
                workspace_size.height * (scaled_height + gap_size)
            };
        }

        void set_ws_dim(const wf::point_t& ws, float value)
        {
            render_colors[{ws.x, ws.y}] = value;
            if (render_node)
            {
                scene::damage_node(render_node, render_node->get_bounding_box());
            }
        }

    protected:
        bool transparent_background = true;
        wf::output_t *output;

        wf::color_t background_color = {0, 0, 0, 0};
        int gap_size = 0;

        wf::geometry_t viewport = {0, 0, 0, 0};

        std::map<std::pair<int, int>, float> render_colors;

        float get_color_for_workspace(wf::point_t ws)
        {
            auto it = render_colors.find({ws.x, ws.y});
            if (it == render_colors.end())
            {
                return 1.0;
            }

            return it->second;
        }

        std::vector<wf::point_t> get_visible_workspaces(wf::geometry_t viewport) const
        {
            std::vector<wf::point_t> visible;
            auto wsize = output->wset()->get_workspace_grid_size();
            for (int i = 0; i < wsize.width; i++)
            {
                for (int j = 0; j < wsize.height; j++)
                {
                    if (viewport & get_workspace_rectangle({i, j}))
                    {
                        visible.push_back({i, j});
                    }
                }
            }

            return visible;
        }

    protected:
        class workspace_wall_node_second_t : public scene::node_t
        {
            class wwall_render_instance_t : public scene::render_instance_t
            {
                workspace_wall_node_second_t *self;

                std::vector<std::vector<std::vector<scene::render_instance_uptr>>>
                instances;

                scene::damage_callback push_damage;
                wf::signal::connection_t<scene::node_damage_signal> on_wall_damage =
                    [=] (scene::node_damage_signal *ev)
                {
                    push_damage(ev->region);
                };

              wf::geometry_t get_workspace_rect(wf::point_t ws)
{
    auto output_size = self->wall->output->get_screen_size();
    
    // Apply scale to workspace rect
    int scaled_width = static_cast<int>(output_size.width * WORKSPACE_SCALE);
    int scaled_height = static_cast<int>(output_size.height * WORKSPACE_SCALE);
    
    // Right-align: each workspace's right edge should be at (ws.x + 1) * output_size.width / WORKSPACE_SCALE
    int right_edge = (ws.x + 1) * output_size.width / WORKSPACE_SCALE + output_size.width * (-2.75 * (WORKSPACE_SCALE*WORKSPACE_SCALE) + 4.625 * WORKSPACE_SCALE - 1.875);
    int x_pos = right_edge - scaled_width;
    
    return {
        .x     = x_pos + ws.x ,
        .y     = ws.y * (scaled_height + self->wall->gap_size),
        .width = scaled_width,
        .height = scaled_height,
    };
};

            public:
                wwall_render_instance_t(workspace_wall_node_second_t *self,
                    scene::damage_callback push_damage)
                {
                    this->self = self;
                    this->push_damage = push_damage;
                    self->connect(&on_wall_damage);

                    instances.resize(self->workspaces.size());
                    for (int i = 0; i < 1; i++)
                    {
                        instances[i].resize(self->workspaces[i].size());
                        for (int j = 0; j < (int)self->workspaces[i].size(); j++)
                        {
                            auto push_damage_child = [=] (const wf::region_t& damage)
                            {
                                wf::region_t our_damage;
                                for (auto& rect : damage)
                                {
                                    wf::geometry_t box = wlr_box_from_pixman_box(rect);
                                    box = box + wf::origin(get_workspace_rect({i, j}));
                                    auto A = self->wall->viewport;
                                    auto B = self->get_bounding_box();
                                    our_damage |= scale_box(A, B, box);
                                }

                                push_damage(our_damage);
                            };

                            self->workspaces[workspaceX_pos][j]->gen_render_instances(instances[i][j],
                                push_damage_child, self->wall->output);
                        }
                    }
                }

                using render_tag = std::tuple<int, float>;
                static constexpr int TAG_BACKGROUND = 0;
                static constexpr int TAG_WS_DIM     = 1;
                static constexpr int FRAME_EV = 2;

                void schedule_instructions(
                std::vector<scene::render_instruction_t>& instructions,
                const wf::render_target_t& target, wf::region_t& damage) override
            {
                instructions.push_back(scene::render_instruction_t{
                        .instance = this,
                        .target   = target,
                        .damage   = wf::region_t{},
                        .data     = render_tag{FRAME_EV, 0.0},
                    });

                // Scale damage to be in the workspace's coordinate system
                wf::region_t workspaces_damage;
                for (auto& rect : damage)
                {
                    auto box = wlr_box_from_pixman_box(rect);
                    wf::geometry_t A = self->get_bounding_box();
                    wf::geometry_t B = self->wall->viewport;

                    workspaces_damage |= scale_box(A, B, box);
                }

                for (int i = 0; i < 1; i++)
                {
                    for (int j = 0; j < (int)self->workspaces[i].size(); j++)
                    {
                        wf::geometry_t workspace_rect = get_workspace_rect({i, j});
                        
                        // Compute render target: render the full workspace but scale it down
                        wf::render_target_t our_target = target;
                        
                        // The geometry should be the full unscaled workspace size
                        auto full_size = self->wall->output->get_screen_size();
                        our_target.geometry = {0, 0, full_size.width, full_size.height};

                        // Use the x-offset calculation approach from original code
                        wf::geometry_t relative_to_viewport = scale_box(
                            self->wall->viewport, 
                            target.geometry,
                            { workspace_rect.x - workspace_rect.width/2,
                              workspace_rect.y,
                              workspace_rect.width,
                              workspace_rect.height});

                        relative_to_viewport = add_offset_to_target(relative_to_viewport, workspace_rect.width/2, 0);
                                         
                        our_target.subbuffer = target.framebuffer_box_from_geometry_box(relative_to_viewport);

                        // Workspace rect for damage calculation
                        wf::geometry_t workspace_rect2;
                        workspace_rect2.x = workspace_rect.x - workspace_rect.width;
                        workspace_rect2.y = workspace_rect.y;
                        workspace_rect2.width = workspace_rect.width;
                        workspace_rect2.height = workspace_rect.height;

                        // Take the damage for the workspace in workspace-local coordinates
                        wf::region_t our_damage = workspaces_damage & workspace_rect2;
                        workspaces_damage ^= our_damage;
                        our_damage += -wf::origin(workspace_rect2);

                          // The damage region in wall coordinates (scaled positions)
                       
                        for (auto& rect : workspaces_damage)
                        {
                            wf::geometry_t box = wlr_box_from_pixman_box(rect);
                            
                            // Check if this damage intersects with this workspace's scaled area
                            if (box.x < workspace_rect.x + workspace_rect.width &&
                                box.x + box.width > workspace_rect.x &&
                                box.y < workspace_rect.y + workspace_rect.height &&
                                box.y + box.height > workspace_rect.y)
                            {
                                // Transform from scaled wall coordinates to unscaled workspace coordinates
                                wf::geometry_t ws_damage;
                                ws_damage.x = static_cast<int>((box.x - workspace_rect.x) / WORKSPACE_SCALE);
                                ws_damage.y = static_cast<int>((box.y - workspace_rect.y) / WORKSPACE_SCALE);
                                ws_damage.width = static_cast<int>(box.width / WORKSPACE_SCALE);
                                ws_damage.height = static_cast<int>(box.height / WORKSPACE_SCALE);
                                
                                our_damage |= ws_damage;
                            }
                        }

                        // Dim workspaces at the end (the first instruction pushed is executed last)
                        instructions.push_back(scene::render_instruction_t{
                                .instance = this,
                                .target   = our_target,
                                .damage   = our_damage,
                                .data     = render_tag{TAG_WS_DIM,
                                    self->wall->get_color_for_workspace({i, j})},
                            });

                        // Render the workspace contents first
                        for (auto& ch : instances[i][j])
                        {
                            ch->schedule_instructions(instructions, our_target, our_damage);
                        }
                    }
                }
            }


                void render(const wf::render_target_t& target,
                    const wf::region_t& region)
                {
                    // Get the render tag from the current instruction
                    // This is a simplified version - you may need to store the tag
                    // in a member variable during schedule_instructions
                }

                void presentation_feedback(wf::output_t *output) override
                {
                    for (int i = 0; i < 1; i++)
                    {
                        for (int j = 0; j < (int)self->workspaces[i].size(); j++)
                        {
                            for (auto& ch : instances[i][j])
                            {
                                ch->presentation_feedback(output);
                            }
                        }
                    }
                }

                void compute_visibility(wf::output_t *output, wf::region_t& visible) override
                {
                    for (int i = 0; i < 1; i++)
                    {
                        for (int j = 0; j < (int)self->workspaces[i].size(); j++)
                        {
                            wf::region_t ws_region = self->workspaces[i][j]->get_bounding_box();
                            for (auto& ch : this->instances[i][j])
                            {
                                ch->compute_visibility(output, ws_region);
                            }
                        }
                    }
                }
            };

        public:
            workspace_wall_node_second_t(remoteview_workspace_wall *wall) : node_t(false)
            {
             this->wall  = wall;
            auto [w, h] = wall->output->wset()->get_workspace_grid_size();
            workspaces.resize(w);
            for (int i = 0; i < w; i++)
            {
                for (int j = 0; j < h; j++)
                {
                    auto node = std::make_shared<workspace_stream_node_t>(
                        wall->output, wf::point_t{i, j});
                    workspaces[i].push_back(node);
                }
            }
            }

            virtual void gen_render_instances(
                std::vector<scene::render_instance_uptr>& instances,
                scene::damage_callback push_damage, wf::output_t *shown_on) override
            {
                if (shown_on != this->wall->output)
                {
                    return;
                }

                instances.push_back(std::make_unique<wwall_render_instance_t>(
                    this, push_damage));
            }

            std::string stringify() const override
            {
                return "workspace-wall " + stringify_flags();
            }

            wf::geometry_t get_bounding_box() override
            {
                return wall->output->get_layout_geometry();
            }

        private:
            remoteview_workspace_wall *wall;
            std::vector<std::vector<std::shared_ptr<workspace_stream_node_t>>> workspaces;
        };

        std::shared_ptr<workspace_wall_node_second_t> render_node;
    };
}