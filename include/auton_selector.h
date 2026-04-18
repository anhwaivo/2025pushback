#pragma once

#include "pros/apix.h"  // Includes full LVGL feature set
#include "auton.h"
#include <functional>
#include <vector>
#include <string>
#include <map>

/**
 * @brief Category-based autonomous routine selector
 * 
 * Multi-step selector: Color -> Side -> Specific Auton
 */
class AutonSelector {
public:
    typedef std::function<void()> routine_action_t;
    
    enum class Color { RED, BLUE };
    enum class Side { LEFT, RIGHT };
    
    struct Routine {
        std::string name;
        routine_action_t action;
    };
    
    /**
     * @brief Create autonomous selector with categorized routines
     */
    AutonSelector();
    
    /**
     * @brief Run the selected autonomous routine
     */
    void run_auton();
    
    /**
     * @brief Get the currently selected routine index
     */
    int get_selected_index() const { return selected_auton_index; }
    
    /**
     * @brief Get the currently selected routine name
     */
    std::string get_selected_name() const;
    
    /**
     * @brief Get the currently selected color (RED or BLUE)
     */
    Color get_selected_color() const { return selected_color; }
    
    /**
     * @brief Initialize and show the selector on screen
     */
    void initialize();
    
    /**
     * @brief Update the selector (call in competition_initialize or disabled)
     */
    void update();
    
    /**
     * @brief Update telemetry display (X, Y, Theta, Temperature)
     * Call this periodically to update telemetry values
     */
    void update_telemetry();
    
    /**
     * @brief Toggle color (Red/Blue)
     */
    void toggle_color();
    
    /**
     * @brief Select next auton
     */
    void next_auton(bool wrap_around = true);
    
    /**
     * @brief Select previous auton
     */
    void prev_auton(bool wrap_around = true);
    
    /**
     * @brief Set selected auton by index (for manual selection in code)
     */
    void set_selected_index(int index);
    
    /**
     * @brief Switch from selector to telemetry mode (for match)
     */
    void show_telemetry();
    
private:
    // Selection state
    Color selected_color;
    Side selected_side;
    int selected_auton_index;
    
    // Simple flat list of all routines
    std::vector<Routine> all_routines;
    
    // Current category mode
    enum class Mode { COLOR_SELECT, SIDE_SELECT, AUTON_SELECT, SKILLS, TEST };
    Mode current_mode;
    
    // LVGL objects
    lv_obj_t *screen;
    lv_obj_t *title_label;
    lv_obj_t *main_label;
    lv_obj_t *color_btn;      // Main color button (click to proceed)
    lv_obj_t *color_toggle_btn; // Small toggle button for color
    lv_obj_t *left_btn;
    lv_obj_t *right_btn;
    lv_obj_t *prev_btn;
    lv_obj_t *next_btn;
    lv_obj_t *back_btn;
    lv_obj_t *select_btn;  // Button to select/confirm auton
    
    // Telemetry labels
    lv_obj_t *telemetry_label;
    lv_obj_t *pose_label;
    lv_obj_t *temp_label;
    lv_obj_t *dist_label;  // Distance sensor position label
    
    static AutonSelector* instance;
    
    void update_display();
    
    static void color_toggle_btn_cb(lv_event_t *event);
    static void prev_btn_cb(lv_event_t *event);
    static void next_btn_cb(lv_event_t *event);
};
