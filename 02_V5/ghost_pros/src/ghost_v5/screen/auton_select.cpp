#include "ghost_v5/screen/auton_select.hpp"
#include <cstdio>

namespace ghost_v5{
    AutonSelector::AutonSelector() {
        createUi();
    }

    void AutonSelector::createUi() {
        screen_ = lv_obj_create(nullptr, nullptr);

        button_ = lv_btn_create(screen_, nullptr);
        lv_obj_set_size(button_, 160, 80);
        lv_obj_align(button_, nullptr, LV_ALIGN_CENTER, 0, 0);

        label_ = lv_label_create(button_, nullptr);

        lv_obj_set_free_ptr(button_, this);

        lv_btn_set_action(
            button_,
            LV_BTN_ACTION_CLICK,
            buttonEventCallback
        );

        updateUi();

        lv_scr_load(screen_);
    }

    lv_res_t AutonSelector::buttonEventCallback(lv_obj_t* button) {
        auto* self = static_cast<AutonSelector*>(lv_obj_get_free_ptr(button));

        if(self != nullptr) {
            self->onButtonClicked();
        }

        return LV_RES_OK;
    }

    void AutonSelector::onButtonClicked() {
        ++counter_;
        updateUi();
    }

    void AutonSelector::updateUi() {
        char text[32];
        std::snprintf(text, sizeof(text), "Count: %d", counter_);

        lv_label_set_text(label_, text);
        lv_obj_align(label_, nullptr, LV_ALIGN_CENTER, 0, 0);
    }
} // namespace ghost_v5
