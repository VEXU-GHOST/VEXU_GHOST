#ifndef GHOST_V5_SCREEN_AUTON_SELECTOR_HPP_
#define GHOST_V5_SCREEN_AUTON_SELECTOR_HPP_

#include <cstdint>
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

        // Data freshness at the time the complete snapshot is handed to the UI.
        struct Freshness {
            bool valid = false;
            uint32_t age_ms = 0;
        };

        struct RobotStatus {
            Freshness freshness;
            std::string mode = "UNKNOWN";
            bool field_connected = false;
            std::string active_auton = "UNKNOWN";
        };

        struct Localization {
            Freshness freshness;
            double x_m = 0.0;
            double y_m = 0.0;
            double heading_deg = 0.0;
        };

        struct DriveHealth {
            Freshness freshness;
            double left_rpm = 0.0;
            double right_rpm = 0.0;
            double left_current_a = 0.0;
            double right_current_a = 0.0;
            double max_temp_c = 0.0;
        };

        struct BallSorting {
            Freshness freshness;
            std::string color = "UNKNOWN";
            bool auto_sort = false;
            std::string sorter_state = "UNKNOWN";
        };

        struct PartnerRobot {
            Freshness freshness;
            std::string status = "UNKNOWN";
            double x_m = 0.0;
            double y_m = 0.0;
            double heading_deg = 0.0;
        };

        struct Telemetry {
            RobotStatus robot;
            Localization localization;
            DriveHealth drive;
            BallSorting sorting;
            PartnerRobot partner;
        };

        AutonSelector();
        ~AutonSelector();

        // Copies a complete snapshot under a mutex. Does not call LVGL.
        // Caller supplies each page's validity and source age in milliseconds.
        void setTelemetry(const Telemetry& telemetry);

        Alliance getCurrentAlliance();
        Auton getCurrentAuton();

    private:
        //General
        lv_style_t style_gray_button_;

        void createUi();
        void updateUi();

        static lv_res_t buttonEventCallback(lv_obj_t* button);

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

        int autons_last_ = 3;
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

        //Minimap
        lv_style_t style_minimap_;
        lv_style_t style_robot_;

        lv_obj_t* minimap_ = nullptr;
        lv_obj_t* robot_ = nullptr;

        void createMinimap();

        //Data Panel

        static constexpr uint32_t rotation_delay_ = 1500;
        static constexpr uint32_t manual_delay_ = 3000;
        static constexpr uint32_t stale_delay_ = 1000;
        static constexpr uint32_t refresh_delay_ = 50;

        const char* data_titles_[5] = {
            "Robot Status",
            "Localization",
            "Drive Health",
            "Ball Sorting",
            "Partner Robot"
        };

        static constexpr int data_page_count_ = sizeof(data_titles_) / sizeof(data_titles_[0]);
        int data_current_ = 0;
        uint32_t last_page_change_ = 0;
        bool manual_pause_ = false;

        Telemetry telemetry_;
        uint32_t telemetry_received_ = 0;
        pros::Mutex telemetry_lock_;

        lv_task_t* data_task_ = nullptr;
        static void dataTaskCallback(void* context);
        unsigned long getDataAge(const Freshness& freshness, uint32_t received);

        lv_style_t style_data_;

        lv_obj_t* data_panel_ = nullptr;
        lv_obj_t* label_data_panel_ = nullptr;
        lv_obj_t* label_readings_ = nullptr;
        lv_obj_t* label_page_ = nullptr;
        lv_obj_t* label_timestamp_ = nullptr;

        lv_obj_t* button_data_left_ = nullptr;
        lv_obj_t* label_data_left_ = nullptr;

        lv_obj_t* button_data_right_ = nullptr;
        lv_obj_t* label_data_right_ = nullptr;

        void createDataPanel();
        void updateDataPanel();
        void onButtonDataLeftClicked();
        void onButtonDataRightClicked();
        void checkTimestamp();
    };

} // namespace ghost_v5

#endif // GHOST_V5_SCREEN_AUTON_SELECTOR_HPP_

