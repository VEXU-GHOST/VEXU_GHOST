#ifndef GHOST_V5_SCREEN_AUTON_SELECTOR_HPP_
#define GHOST_V5_SCREEN_AUTON_SELECTOR_HPP_

#include <atomic>
#include <cmath>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>

#include "pros/apix.h"
#include "screen_interface.hpp"

namespace ghost_v5 {

    class AutonSelector : public ScreenInterface {
    public:

        enum class Alliance {
            RED,
            BLUE
        };

        enum class Auton {
            PINKY,
            INKY,
            CV,
            MOVEMENT
        };

        struct DriveHealth {
            bool valid = false;
            double left_rpm = 0.0;
            double right_rpm = 0.0;
            double left_current_a = 0.0;
            double right_current_a = 0.0;
            double max_temp_c = 0.0;
        };

        struct Telemetry {
            DriveHealth drive;
        };

        AutonSelector();
        ~AutonSelector() override;

        // ScreenInterface overrides. None of these call LVGL; they only record state
        // that the LVGL task (uiTaskCallback) picks up, so they are safe from any task.
        void reset() override;
        void showError(const std::string& msg) override;

        // Copies a complete snapshot under a mutex. Does not call LVGL.
        // Caller supplies each page's validity and source age in milliseconds.
        void setTelemetry(const Telemetry& telemetry);

        // Robot pose in the map frame (meters, radians CCW+), drawn on the minimap. NaN hides the
        // robot. Does not call LVGL.
        void setRobotPose(float x_m, float y_m, float yaw_rad);

        Alliance getCurrentAlliance();
        Auton getCurrentAuton();

    private:
        //General
        lv_style_t style_gray_button_;

        void createUi();
        void updateUi();

        static lv_res_t buttonEventCallback(lv_obj_t* button);

        // Single LVGL render hook. Everything that draws runs from here or from
        // buttonEventCallback, both in the LVGL task.
        lv_task_t* ui_task_ = nullptr;
        static void uiTaskCallback(void* context);

        //Screen
        lv_style_t style_screen_;

        lv_obj_t* screen_ = nullptr;

        //Alliance Switching
        Alliance current_alliance_ = Alliance::RED;

        lv_style_t style_red_button_;
        lv_style_t style_blue_button_;

        lv_obj_t* button_alliance_ = nullptr;
        lv_obj_t* label_alliance_ = nullptr;

        void createAllianceButton();
        void onButtonAllianceClicked();

        //Auton Select
        Auton autons_[4] = {
            Auton::PINKY,
            Auton::INKY,
            Auton::CV,
            Auton::MOVEMENT
        };

        static constexpr int autons_last_ = sizeof(autons_) / sizeof(autons_[0]) - 1;
        int autons_current_ = 0;

        lv_style_t style_auton_name_;

        lv_obj_t* auton_name_ = nullptr;
        lv_obj_t* label_auton_name_ = nullptr;

        lv_obj_t* button_auton_left_ = nullptr;
        lv_obj_t* label_auton_left_ = nullptr;

        lv_obj_t* button_auton_right_ = nullptr;
        lv_obj_t* label_auton_right_ = nullptr;

        void createAutonSelector();
        void onButtonAutonLeftClicked();
        void onButtonAutonRightClicked();

        //Minimap. 6x6 tile field, origin bottom right, +x up the screen, +y left.
        static constexpr int minimap_size_ = 168;
        static constexpr float px_per_m_ = minimap_size_ / (6 * 0.6096f);

        lv_style_t style_minimap_;
        lv_style_t style_robot_;

        lv_obj_t* minimap_ = nullptr;
        lv_obj_t* robot_ = nullptr;
        lv_point_t robot_points_[7];  // lv_line keeps a pointer to these

        float pose_x_m_ = NAN;
        float pose_y_m_ = NAN;
        float pose_yaw_rad_ = NAN;
        pros::Mutex pose_lock_;
        uint32_t last_minimap_draw_ = 0;

        void createMinimap();
        void updateMinimap();

        //Data Panel

        static constexpr uint32_t rotation_delay_ = 1500;
        static constexpr uint32_t manual_delay_ = 3000;
        static constexpr uint32_t refresh_delay_ = 50;

        const char* data_titles_[2] = {
            "Drive Health",
            "Log"
        };
        static constexpr int log_page_ = 1;

        static constexpr int data_page_count_ = sizeof(data_titles_) / sizeof(data_titles_[0]);
        int data_current_ = 0;
        uint32_t last_page_change_ = 0;
        bool manual_pause_ = false;
        bool log_hold_ = false;

        Telemetry telemetry_;
        pros::Mutex telemetry_lock_;

        lv_style_t style_data_;

        lv_obj_t* data_panel_ = nullptr;
        lv_obj_t* label_data_panel_ = nullptr;
        lv_obj_t* label_readings_ = nullptr;

        lv_obj_t* button_data_left_ = nullptr;
        lv_obj_t* label_data_left_ = nullptr;

        lv_obj_t* button_data_right_ = nullptr;
        lv_obj_t* label_data_right_ = nullptr;

        void createDataPanel();
        void updateDataPanel();
        void onButtonDataLeftClicked();
        void onButtonDataRightClicked();
        void checkTimestamp();

        //Log (6th data page). New messages jump to this page and hold it.
        static constexpr uint32_t log_hold_delay_ = 5000;
        static constexpr int log_max_lines_ = 7;
        static constexpr int log_wrap_chars_ = 44;

        std::deque<std::string> log_lines_;
        std::atomic<bool> clear_log_pending_{false};

        lv_style_t style_log_;
        lv_obj_t* label_log_ = nullptr;

        void createLogLabel();
        void drainPrintQueue();

        //Heartbeat: blinks while the Jetson has sent a message in the last second.
        // Same timing as the old text screen: toggles every second, and the dot / the
        // bars around the status title only show while connected.
        static constexpr uint32_t heartbeat_timeout_ = 1000;
        static constexpr uint32_t heartbeat_blink_ = 1000;

        uint32_t last_blink_ = 0;
        bool blink_on_ = false;
        bool heartbeat_shown_ = false;

        lv_style_t style_heartbeat_on_;
        lv_style_t style_heartbeat_off_;
        lv_obj_t* heartbeat_ = nullptr;

        void createHeartbeat();
        void updateHeartbeat();

        //Status (settable title, shown in the data panel footer)
        std::string status_shown_;
        lv_obj_t* label_status_ = nullptr;

        void createStatusLabel();
        void updateStatus();

        //Error overlay. Built up front so a crash can still render it.
        pros::Mutex error_lock_;
        bool error_pending_ = false;
        std::string error_msg_;
        bool error_shown_ = false;

        lv_style_t style_error_;
        lv_style_t style_error_body_;
        lv_obj_t* error_overlay_ = nullptr;
        lv_obj_t* label_error_title_ = nullptr;
        lv_obj_t* label_error_body_ = nullptr;

        void createErrorOverlay();
        void updateErrorOverlay();
    };

} // namespace ghost_v5

#endif // GHOST_V5_SCREEN_AUTON_SELECTOR_HPP_

