#include "ghost_v5/screen/auton_select.hpp"
#include <cstdio>

namespace ghost_v5 {
    AutonSelector::AutonSelector() {
        createUi();
    }

    AutonSelector::~AutonSelector() {
        if (ui_task_ != nullptr) {
            lv_task_del(ui_task_);
        }
    }

    void AutonSelector::reset() {
        clearPrintQueue();
        clear_log_pending_ = true;
    }

    void AutonSelector::showError(const std::string& msg) {
        {
            std::unique_lock<pros::Mutex> lock(error_lock_);
            error_msg_ = msg;
            error_pending_ = true;
        }
        setTitle("ERROR");
        addToPrintQueue(msg);
    }

    void AutonSelector::setTelemetry(const Telemetry& telemetry) {
        std::unique_lock<pros::Mutex> lock(telemetry_lock_);
        telemetry_ = telemetry;
    }

    void AutonSelector::uiTaskCallback(void* context) {
        auto* self = static_cast<AutonSelector*>(context);
        self->drainPrintQueue();
        self->updateErrorOverlay();
        if (self->error_shown_) {
            return;
        }
        self->checkTimestamp();
        self->updateHeartbeat();
        self->updateStatus();
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
        createLogLabel();
        createStatusLabel();
        createMinimap();
        createHeartbeat();
        // Created last so it draws on top of everything else.
        createErrorOverlay();

        // Start on the log so startup messages (IMU calibration, ports) are visible.
        data_current_ = log_page_;
        log_hold_ = true;

        updateUi();
        lv_scr_load(screen_);
        last_page_change_ = pros::millis();
        ui_task_ = lv_task_create(uiTaskCallback, refresh_delay_, LV_TASK_PRIO_LOW, this);
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

        // All coordinates below are relative to the data panel.
        lv_label_set_long_mode(label_data_panel_, LV_LABEL_LONG_CROP);
        lv_obj_set_size(label_data_panel_, 184, 24);
        lv_obj_set_pos(label_data_panel_, 52, 16);
        lv_label_set_align(label_data_panel_, LV_LABEL_ALIGN_CENTER);
        // Keep long incoming text from growing into the footer.
        lv_label_set_long_mode(label_readings_, LV_LABEL_LONG_CROP);
        lv_obj_set_size(label_readings_, 272, 88);
        lv_obj_set_pos(label_readings_, 8, 54);

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
        if (log_hold_) {
            interval = log_hold_delay_;
        } else if (manual_pause_) {
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
            log_hold_ = false;
        }
    }

    void AutonSelector::updateDataPanel() {
        lv_label_set_text(label_data_panel_, data_titles_[data_current_]);

        const bool on_log = (data_current_ == log_page_);
        lv_obj_set_hidden(label_log_, !on_log);
        lv_obj_set_hidden(label_readings_, on_log);

        if (on_log) {
            std::string text;
            for (const auto& line : log_lines_) {
                if (!text.empty()) {
                    text += "\n";
                }
                text += line;
            }
            lv_label_set_text(label_log_, text.empty() ? "No messages" : text.c_str());
            return;
        }

        Telemetry data;
        // Copy while locked, then release before formatting or drawing.
        {
            std::unique_lock<pros::Mutex> lock(telemetry_lock_);
            data = telemetry_;
        }
        // Only non-log page is Drive Health.
        char body[192];
        std::snprintf(body, sizeof(body),
            "L/R: %.0f / %.0f rpm\nL current: %.1f A\nR current: %.1f A\nMax temp: %.0f C",
            data.drive.left_rpm, data.drive.right_rpm,
            data.drive.left_current_a, data.drive.right_current_a, data.drive.max_temp_c);
        if (!data.drive.valid) {
            std::snprintf(body, sizeof(body), "No data received");
        }
        lv_label_set_text(label_readings_, body);
    }

    void AutonSelector::createLogLabel() {
        lv_style_copy(&style_log_, &style_data_);
        style_log_.text.font = &lv_font_dejavu_10;
        style_log_.text.line_space = 1;

        // Same area as the readings label; only one of the two is shown at a time.
        label_log_ = lv_label_create(data_panel_, nullptr);
        lv_label_set_style(label_log_, &style_log_);
        lv_label_set_long_mode(label_log_, LV_LABEL_LONG_CROP);
        lv_obj_set_size(label_log_, 272, 88);
        lv_obj_set_pos(label_log_, 8, 54);
        lv_obj_set_hidden(label_log_, true);
    }

    void AutonSelector::drainPrintQueue() {
        if (clear_log_pending_.exchange(false)) {
            log_lines_.clear();
        }

        auto msgs = takePrintQueue();
        if (msgs.empty()) {
            return;
        }

        for (const auto& msg : msgs) {
            for (const auto& line : wrapStringToLineLength(msg, log_wrap_chars_)) {
                log_lines_.push_back(line);
            }
        }
        while (log_lines_.size() > static_cast<std::size_t>(log_max_lines_)) {
            log_lines_.pop_front();
        }

        // Jump to the log and hold it so new messages get seen.
        data_current_ = log_page_;
        last_page_change_ = pros::millis();
        log_hold_ = true;
    }

    void AutonSelector::createHeartbeat() {
        lv_style_copy(&style_heartbeat_on_, &lv_style_plain);
        style_heartbeat_on_.body.main_color = LV_COLOR_HEX(0x00FF00);
        style_heartbeat_on_.body.grad_color = LV_COLOR_HEX(0x00FF00);
        style_heartbeat_on_.body.border.width = 0;
        style_heartbeat_on_.body.radius = LV_RADIUS_CIRCLE;

        lv_style_copy(&style_heartbeat_off_, &style_heartbeat_on_);
        style_heartbeat_off_.body.main_color = LV_COLOR_HEX(0x404040);
        style_heartbeat_off_.body.grad_color = LV_COLOR_HEX(0x404040);

        // Top-right corner of the minimap, inside the border.
        heartbeat_ = lv_obj_create(minimap_, nullptr);
        lv_obj_set_size(heartbeat_, 14, 14);
        lv_obj_set_pos(heartbeat_, 148, 6);
        lv_obj_set_style(heartbeat_, &style_heartbeat_off_);
    }

    void AutonSelector::updateHeartbeat() {
        const uint32_t now = pros::millis();
        const bool connected = (now - last_connection_time_.load()) < heartbeat_timeout_;

        if (now - last_blink_ >= heartbeat_blink_) {
            blink_on_ = !blink_on_;
            last_blink_ = now;
        }
        const bool on = connected && blink_on_;

        if (on != heartbeat_shown_) {
            heartbeat_shown_ = on;
            lv_obj_set_style(heartbeat_, on ? &style_heartbeat_on_ : &style_heartbeat_off_);
        }
    }

    void AutonSelector::createStatusLabel() {
        // Data panel footer, centered.
        label_status_ = lv_label_create(data_panel_, nullptr);
        lv_label_set_long_mode(label_status_, LV_LABEL_LONG_CROP);
        lv_obj_set_size(label_status_, 272, 24);
        lv_obj_set_pos(label_status_, 8, 146);
        lv_label_set_align(label_status_, LV_LABEL_ALIGN_CENTER);
    }

    void AutonSelector::updateStatus() {
        // Bars blink around the title like the old screen, in step with the heartbeat dot.
        std::string title = getTitle();
        if (heartbeat_shown_) {
            title = "-- " + title + " --";
        }
        if (title != status_shown_) {
            status_shown_ = title;
            lv_label_set_text(label_status_, status_shown_.c_str());
        }
    }

    void AutonSelector::createErrorOverlay() {
        lv_style_copy(&style_error_, &lv_style_plain);
        style_error_.body.main_color = LV_COLOR_RED;
        style_error_.body.grad_color = LV_COLOR_RED;
        style_error_.body.border.width = 0;
        style_error_.body.radius = 0;
        style_error_.text.color = LV_COLOR_WHITE;
        style_error_.text.font = &lv_font_dejavu_20;

        lv_style_copy(&style_error_body_, &style_error_);
        style_error_body_.text.font = &lv_font_dejavu_10;

        error_overlay_ = lv_obj_create(screen_, nullptr);
        lv_obj_set_style(error_overlay_, &style_error_);
        lv_obj_set_size(error_overlay_, 480, 240);
        lv_obj_set_pos(error_overlay_, 0, 0);

        label_error_title_ = lv_label_create(error_overlay_, nullptr);
        lv_label_set_style(label_error_title_, &style_error_);
        lv_label_set_text(label_error_title_, "ERROR");
        lv_obj_align(label_error_title_, nullptr, LV_ALIGN_IN_TOP_MID, 0, 12);

        label_error_body_ = lv_label_create(error_overlay_, nullptr);
        lv_label_set_style(label_error_body_, &style_error_body_);
        lv_label_set_long_mode(label_error_body_, LV_LABEL_LONG_BREAK);
        lv_obj_set_width(label_error_body_, 464);
        lv_obj_set_pos(label_error_body_, 8, 48);

        lv_obj_set_hidden(error_overlay_, true);
    }

    void AutonSelector::updateErrorOverlay() {
        if (error_shown_) {
            return;
        }

        std::string msg;
        {
            std::unique_lock<pros::Mutex> lock(error_lock_);
            if (!error_pending_) {
                return;
            }
            msg = error_msg_;
        }

        // Error text, then the log lines leading up to it for context.
        std::string body = msg + "\n\nRecent log:";
        for (const auto& line : log_lines_) {
            body += "\n" + line;
        }
        lv_label_set_text(label_error_body_, body.c_str());
        lv_obj_set_hidden(error_overlay_, false);
        error_shown_ = true;
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
