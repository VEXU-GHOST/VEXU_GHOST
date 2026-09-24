#include "ghost_v5/screen/auton_select.hpp"

namespace ghost_v5{
    AutonSelector::AutonSelector() {
        createUi();
    }

    void AutonSelector::createUi() {
        screen_ = lv_obj_create(nullptr);

        button_ = lv_btn_create(screen_);
        lv_obj_set_size(button_, 160, 80);
        lv_obj_center(button_);

        label_ = lv_label_create(button_);
        lv_obj_center(label_);

        lv_obj_add_event_cb(
            button_,
            buttonEventCallback,
            LV_EVENT_CLICKED,
            this
        );

        updateUi();
    }

    void AutonSelector::buttonEventCallback(lv_event_t* event) {
        auto* self = static_cast<AutonSelector*>(lv_event_get_user_data(event));

        self->onButtonClicked();
    }

    void AutonSelector::onButtonClicked() {
        ++counter_;
        updateUi();
    }

    void AutonSelector::updateUi() {
        lv_label_set_text_fmt(
            label_,
            "Count: %d",
            counter_
        );
    }
} // namespace ghost_v5
