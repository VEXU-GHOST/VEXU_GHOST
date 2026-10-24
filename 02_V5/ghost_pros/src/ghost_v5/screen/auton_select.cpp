#include "ghost_v5/screen/auton_select.hpp"
#include <cstdio>

namespace ghost_v5 {
    AutonSelector::AutonSelector() {
        createUi();
    }

    AutonSelector::~AutonSelector() {
        if (data_task_ != nullptr) {
            lv_task_del(data_task_);
        }
    }

    void AutonSelector::setTelemetry(const Telemetry& telemetry) {
        std::unique_lock<pros::Mutex> lock(telemetry_lock_);
        telemetry_ = telemetry;
        telemetry_received_ = pros::millis();
    }

    void AutonSelector::dataTaskCallback(void* context) {
        auto* self = static_cast<AutonSelector*>(context);
        self->checkTimestamp();
        self->updateDataPanel();
    }

    AutonSelector::Alliance AutonSelector::getCurrentAlliance() {
        return current_alliance_;
    }

    AutonSelector::Auton AutonSelector::getCurrentAuton() {
        return autons_[autons_current_];
    }

    void AutonSelector::createUi() {
        //Gray button reused between auton selector and data panel
        lv_style_copy(&style_gray_button_, &lv_style_btn_rel);
        style_gray_button_.body.main_color = LV_COLOR_HEX(0x404040);
        style_gray_button_.body.grad_color = LV_COLOR_HEX(0x404040);
        style_gray_button_.text.color = LV_COLOR_WHITE;

        lv_style_copy(&style_screen_, &lv_style_plain);
        style_screen_.body.main_color = LV_COLOR_HEX(0x101010);
        style_screen_.body.grad_color = LV_COLOR_HEX(0x101010);
        style_screen_.body.border.width = 0;
        style_screen_.body.radius = 0;

        screen_ = lv_obj_create(nullptr, nullptr);
        lv_obj_set_style(screen_, &style_screen_);

        createAllianceButton();
        createAutonSelector();
        createDataPanel();
        createMinimap();
        updateUi();
        lv_scr_load(screen_);
        last_page_change_ = pros::millis();
        data_task_ = lv_task_create(dataTaskCallback, refresh_delay_, LV_TASK_PRIO_LOW, this);
    }

    lv_res_t AutonSelector::buttonEventCallback(lv_obj_t* button) {
        auto* self = static_cast<AutonSelector*>(lv_obj_get_free_ptr(button));

        if (self != nullptr) {
            if (button == self->button_alliance_) {
                self->onButtonAllianceClicked();
            } else if (button == self->button_auton_left_) {
                self->onButtonAutonLeftClicked();
            } else if (button == self->button_auton_right_) {
                self->onButtonAutonRightClicked();
            } else if (button == self->button_data_left_) {
                self->onButtonDataLeftClicked();
            } else if (button == self->button_data_right_) {
                self->onButtonDataRightClicked();
            }
        }

        return LV_RES_OK;
    }

    void AutonSelector::onButtonAllianceClicked() {
        if (current_alliance_ == Alliance::RED) {
            current_alliance_ = Alliance::BLUE;
        } else {
            current_alliance_ = Alliance::RED;
        }

        updateUi();
    }

    void AutonSelector::onButtonAutonLeftClicked() {
        if (autons_current_ == 0) {
            autons_current_ = autons_last_;
        } else {
            autons_current_--;
        }

        updateUi();
    }

    void AutonSelector::onButtonAutonRightClicked() {
        if (autons_current_ == autons_last_) {
            autons_current_ = 0;
        } else {
            autons_current_++;
        }

        updateUi();
    }

    void AutonSelector::createAllianceButton() {
        lv_style_copy(&style_red_button_, &lv_style_btn_rel);
        style_red_button_.body.main_color = LV_COLOR_RED;
        style_red_button_.body.grad_color = LV_COLOR_RED;
        style_red_button_.text.color = LV_COLOR_WHITE;

        lv_style_copy(&style_blue_button_, &lv_style_btn_rel);
        style_blue_button_.body.main_color = LV_COLOR_BLUE;
        style_blue_button_.body.grad_color = LV_COLOR_BLUE;
        style_blue_button_.text.color = LV_COLOR_WHITE;

        button_alliance_ = lv_btn_create(screen_, nullptr);
        lv_obj_set_size(button_alliance_, 104, 48);
        lv_obj_set_pos(button_alliance_, 8, 8);

        label_alliance_ = lv_label_create(button_alliance_, nullptr);

        lv_obj_set_free_ptr(button_alliance_, this);

        lv_btn_set_action(
            button_alliance_,
            LV_BTN_ACTION_CLICK,
            buttonEventCallback
        );
    }

    void AutonSelector::createAutonSelector() {
        //Name
        lv_style_copy(&style_auton_name_, &lv_style_plain);
        style_auton_name_.body.main_color = LV_COLOR_HEX(0x242424);
        style_auton_name_.body.grad_color = LV_COLOR_HEX(0x242424);
        style_auton_name_.text.color = LV_COLOR_WHITE;
        style_auton_name_.body.border.width = 0;
        style_auton_name_.body.radius = 0;

        auton_name_ = lv_obj_create(screen_, nullptr);
        lv_obj_set_size(auton_name_, 240, 48);
        lv_obj_set_pos(auton_name_, 176, 8);

        label_auton_name_ = lv_label_create(auton_name_, nullptr);

        lv_obj_set_style(auton_name_, &style_auton_name_);

        //Left Button
        button_auton_left_ = lv_btn_create(screen_, nullptr);
        lv_obj_set_size(button_auton_left_, 48, 48);
        lv_obj_set_pos(button_auton_left_, 120, 8);

        label_auton_left_ = lv_label_create(button_auton_left_, nullptr);

        lv_obj_set_free_ptr(button_auton_left_, this);

        lv_btn_set_action(
            button_auton_left_,
            LV_BTN_ACTION_CLICK,
            buttonEventCallback
        );

        lv_btn_set_style(button_auton_left_, LV_BTN_STYLE_REL, &style_gray_button_);
        lv_btn_set_style(button_auton_left_, LV_BTN_STYLE_PR, &style_gray_button_);

        //Right Button
        button_auton_right_ = lv_btn_create(screen_, nullptr);
        lv_obj_set_size(button_auton_right_, 48, 48);
        lv_obj_set_pos(button_auton_right_, 424, 8);

        label_auton_right_ = lv_label_create(button_auton_right_, nullptr);

        lv_obj_set_free_ptr(button_auton_right_, this);

        lv_btn_set_action(
            button_auton_right_,
            LV_BTN_ACTION_CLICK,
            buttonEventCallback
        );

        lv_btn_set_style(button_auton_right_, LV_BTN_STYLE_REL, &style_gray_button_);
        lv_btn_set_style(button_auton_right_, LV_BTN_STYLE_PR, &style_gray_button_);
    }

    void AutonSelector::createDataPanel() {
        //Panel
        lv_style_copy(&style_data_, &lv_style_plain);
        style_data_.body.main_color = LV_COLOR_HEX(0x242424);
        style_data_.body.grad_color = LV_COLOR_HEX(0x242424);
        style_data_.body.border.color = LV_COLOR_WHITE;
        style_data_.body.border.width = 0;
        style_data_.body.radius = 0;
        style_data_.text.color = LV_COLOR_WHITE;
        style_data_.text.font = &lv_font_dejavu_20;
        style_data_.text.line_space = 2;

        data_panel_ = lv_cont_create(screen_, nullptr);
        lv_obj_set_size(data_panel_, 288, 168);
        lv_obj_set_pos(data_panel_, 8, 64);

        lv_cont_set_fit(data_panel_, false, false);
        lv_cont_set_layout(data_panel_, LV_LAYOUT_OFF);
        lv_cont_set_style(data_panel_, &style_data_);

        label_data_panel_ = lv_label_create(data_panel_, nullptr);
        label_readings_ = lv_label_create(data_panel_, nullptr);
        label_page_ = lv_label_create(data_panel_, nullptr);
        label_timestamp_ = lv_label_create(data_panel_, nullptr);

        // All coordinates below are relative to the data panel.
        lv_label_set_long_mode(label_data_panel_, LV_LABEL_LONG_CROP);
        lv_obj_set_size(label_data_panel_, 184, 24);
        lv_obj_set_pos(label_data_panel_, 52, 16);
        lv_label_set_align(label_data_panel_, LV_LABEL_ALIGN_CENTER);
        // Keep long incoming text from growing into the footer.
        lv_label_set_long_mode(label_readings_, LV_LABEL_LONG_CROP);
        lv_obj_set_size(label_readings_, 272, 88);
        lv_obj_set_pos(label_readings_, 8, 54);
        lv_obj_set_pos(label_timestamp_, 8, 146);
        lv_obj_set_pos(label_page_, 232, 146);

        //Left Button
        button_data_left_ = lv_btn_create(data_panel_, nullptr);
        lv_obj_set_size(button_data_left_, 44, 44);
        lv_obj_set_pos(button_data_left_, 4, 4);

        label_data_left_ = lv_label_create(button_data_left_, nullptr);

        lv_obj_set_free_ptr(button_data_left_, this);

        lv_btn_set_action(
            button_data_left_,
            LV_BTN_ACTION_CLICK,
            buttonEventCallback
        );

        lv_btn_set_style(button_data_left_, LV_BTN_STYLE_REL, &style_gray_button_);

        //Right Button
        button_data_right_ = lv_btn_create(data_panel_, nullptr);
        lv_obj_set_size(button_data_right_, 44, 44);
        lv_obj_set_pos(button_data_right_, 240, 4);

        label_data_right_ = lv_label_create(button_data_right_, nullptr);

        lv_obj_set_free_ptr(button_data_right_, this);

        lv_btn_set_action(
            button_data_right_,
            LV_BTN_ACTION_CLICK,
            buttonEventCallback
        );

        lv_btn_set_style(button_data_right_, LV_BTN_STYLE_REL, &style_gray_button_);
        lv_btn_set_style(button_data_left_, LV_BTN_STYLE_PR, &style_gray_button_);
        lv_btn_set_style(button_data_right_, LV_BTN_STYLE_PR, &style_gray_button_);
        lv_label_set_text(label_data_left_, "<");
        lv_obj_align(label_data_left_, nullptr, LV_ALIGN_CENTER, 0, 0);
        lv_label_set_text(label_data_right_, ">");
        lv_obj_align(label_data_right_, nullptr, LV_ALIGN_CENTER, 0, 0);
    }

    void AutonSelector::onButtonDataLeftClicked() {
        if (data_current_ == 0) {
            data_current_ = data_page_count_ - 1;
        } else {
            data_current_--;
        }
        last_page_change_ = pros::millis();
        manual_pause_ = true;
        updateDataPanel();
    }

    void AutonSelector::onButtonDataRightClicked() {
        if (data_current_ == data_page_count_ - 1) {
            data_current_ = 0;
        } else {
            data_current_++;
        }
        last_page_change_ = pros::millis();
        manual_pause_ = true;
        updateDataPanel();
    }

    void AutonSelector::checkTimestamp() {
        const uint32_t now = pros::millis();
        uint32_t interval = rotation_delay_;
        if (manual_pause_) {
            interval = manual_delay_;
        }

        // Unsigned subtraction also works when the millisecond counter wraps.
        if (static_cast<uint32_t>(now - last_page_change_) >= interval) {
            if (data_current_ == data_page_count_ - 1) {
                data_current_ = 0;
            } else {
                data_current_++;
            }
            last_page_change_ = now;
            manual_pause_ = false;
        }
    }

    unsigned long AutonSelector::getDataAge(const Freshness& freshness, uint32_t received) {
        const uint32_t elapsed = pros::millis() - received;

        // Clamp rather than let an old source age overflow back to a fresh value.
        if (freshness.age_ms > UINT32_MAX - elapsed) {
            return static_cast<unsigned long>(UINT32_MAX);
        }

        return static_cast<unsigned long>(freshness.age_ms + elapsed);
    }

    void AutonSelector::updateDataPanel() {
        Telemetry data;
        uint32_t received;
        // Copy while locked, then release before formatting or drawing.
        {
            std::unique_lock<pros::Mutex> lock(telemetry_lock_);
            data = telemetry_;
            received = telemetry_received_;
        }
        char body[192];
        Freshness freshness;
        switch (data_current_) {
        case 0:
            freshness = data.robot.freshness;
            std::snprintf(body, sizeof(body),
                "Mode: %.12s\nField link: %s\nJetson age: %lu ms\nAuton: %.16s",
                data.robot.mode.c_str(), data.robot.field_connected ? "YES" : "NO",
                getDataAge(freshness, received), data.robot.active_auton.c_str());
            break;
        case 1:
            freshness = data.localization.freshness;
            std::snprintf(body, sizeof(body),
                "X: %.2f m\nY: %.2f m\nHeading: %.0f deg\nPose age: %lu ms",
                data.localization.x_m, data.localization.y_m,
                data.localization.heading_deg, getDataAge(freshness, received));
            break;
        case 2:
            freshness = data.drive.freshness;
            std::snprintf(body, sizeof(body),
                "L/R: %.0f / %.0f rpm\nL current: %.1f A\nR current: %.1f A\nMax temp: %.0f C",
                data.drive.left_rpm, data.drive.right_rpm,
                data.drive.left_current_a, data.drive.right_current_a, data.drive.max_temp_c);
            break;
        case 3:
            freshness = data.sorting.freshness;
            std::snprintf(body, sizeof(body),
                "Ball: %.10s\nAuto-sort: %s\nSorter: %.14s\nSensor age: %lu ms",
                data.sorting.color.c_str(), data.sorting.auto_sort ? "ON" : "OFF",
                data.sorting.sorter_state.c_str(), getDataAge(freshness, received));
            break;
        default:
            freshness = data.partner.freshness;
            std::snprintf(body, sizeof(body),
                "State: %.12s\nX/Y: %.2f / %.2f m\nHeading: %.0f deg\nPeer age: %lu ms",
                data.partner.status.c_str(), data.partner.x_m, data.partner.y_m,
                data.partner.heading_deg, getDataAge(freshness, received));
            break;
        }
        if (!freshness.valid) {
            std::snprintf(body, sizeof(body), "No data received");
        }
        lv_label_set_text(label_data_panel_, data_titles_[data_current_]);
        lv_label_set_text(label_readings_, body);
        const char* status = "LIVE";
        if (!freshness.valid) {
            status = "NO DATA";
        } else if (getDataAge(freshness, received) >= stale_delay_) {
            status = "STALE";
        }
        lv_label_set_text(label_timestamp_, status);
        char page[8];
        std::snprintf(page, sizeof(page), "%d/%d", data_current_ + 1, data_page_count_);
        lv_label_set_text(label_page_, page);
    }

    void AutonSelector::createMinimap() {
        //Field
        lv_style_copy(&style_minimap_, &lv_style_plain);
        style_minimap_.body.main_color = LV_COLOR_HEX(0x242424);
        style_minimap_.body.grad_color = LV_COLOR_HEX(0x242424);
        style_minimap_.body.border.color = LV_COLOR_WHITE;
        style_minimap_.body.border.width = 2;
        style_minimap_.body.radius = 0;
        style_minimap_.body.padding.hor = 0;
        style_minimap_.body.padding.ver = 0;
        style_minimap_.body.padding.inner = 0;

        minimap_ = lv_cont_create(screen_, nullptr);
        lv_obj_set_size(minimap_, 168, 168);
        lv_obj_set_pos(minimap_, 304, 64);

        lv_cont_set_fit(minimap_, false, false);
        lv_cont_set_layout(minimap_, LV_LAYOUT_OFF);
        lv_cont_set_style(minimap_, &style_minimap_);

        //Robot
        lv_style_copy(&style_robot_, &lv_style_plain);
        style_robot_.body.main_color = LV_COLOR_HEX(0x7F00FF);
        style_robot_.body.grad_color = LV_COLOR_HEX(0x7F00FF);
        style_robot_.body.border.width = 0;
        style_robot_.body.radius = 2;

        robot_ = lv_obj_create(minimap_, nullptr);
        lv_obj_set_size(robot_, 20, 20);
        lv_obj_set_pos(robot_, 74, 74);
        lv_obj_set_style(robot_, &style_robot_);
    }

    void AutonSelector::updateUi() {
        char text_alliance[9];
        std::snprintf(text_alliance, sizeof(text_alliance), "ALLIANCE");

        char text_arrow_left[2];
        std::snprintf(text_arrow_left, sizeof(text_arrow_left), "<");

        char text_arrow_right[2];
        std::snprintf(text_arrow_right, sizeof(text_arrow_right), ">");

        char text_pinky[6];
        std::snprintf(text_pinky, sizeof(text_pinky), "PINKY");

        char text_inky[5];
        std::snprintf(text_inky, sizeof(text_inky), "INKY");

        char text_cv[3];
        std::snprintf(text_cv, sizeof(text_cv), "CV");

        char text_movement[9];
        std::snprintf(text_movement, sizeof(text_movement), "MOVEMENT");

        if (current_alliance_ == Alliance::RED) {
            lv_btn_set_style(button_alliance_, LV_BTN_STYLE_REL, &style_red_button_);
            lv_btn_set_style(button_alliance_, LV_BTN_STYLE_PR, &style_red_button_);
        } else if (current_alliance_ == Alliance::BLUE) {
            lv_btn_set_style(button_alliance_, LV_BTN_STYLE_REL, &style_blue_button_);
            lv_btn_set_style(button_alliance_, LV_BTN_STYLE_PR, &style_blue_button_);
        }

        if (autons_[autons_current_] == Auton::PINKY) {
            lv_label_set_text(label_auton_name_, text_pinky);
            lv_obj_align(label_auton_name_, nullptr, LV_ALIGN_CENTER, 0, 0);
        } else if (autons_[autons_current_] == Auton::INKY) {
            lv_label_set_text(label_auton_name_, text_inky);
            lv_obj_align(label_auton_name_, nullptr, LV_ALIGN_CENTER, 0, 0);
        } else if (autons_[autons_current_] == Auton::CV) {
            lv_label_set_text(label_auton_name_, text_cv);
            lv_obj_align(label_auton_name_, nullptr, LV_ALIGN_CENTER, 0, 0);
        } else if (autons_[autons_current_] == Auton::MOVEMENT) {
            lv_label_set_text(label_auton_name_, text_movement);
            lv_obj_align(label_auton_name_, nullptr, LV_ALIGN_CENTER, 0, 0);
        }

        lv_label_set_text(label_alliance_, text_alliance);
        lv_obj_align(label_alliance_, nullptr, LV_ALIGN_CENTER, 0, 0);

        lv_label_set_text(label_auton_left_, text_arrow_left);
        lv_obj_align(label_auton_left_, nullptr, LV_ALIGN_CENTER, 0, 0);

        lv_label_set_text(label_auton_right_, text_arrow_right);
        lv_obj_align(label_auton_right_, nullptr, LV_ALIGN_CENTER, 0, 0);

        updateDataPanel();
    }
} // namespace ghost_v5
