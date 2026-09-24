#ifndef GHOST_V5_SCREEN_AUTON_SELECTOR_HPP_
#define GHOST_V5_SCREEN_AUTON_SELECTOR_HPP_

#include "pros/apix.h"
#include "screen_interface.hpp"

namespace ghost_v5 {

    class AutonSelector : public ScreenInterface {
    public:
        AutonSelector();

    private:
        int counter_ = 0;

        lv_obj_t* screen_ = nullptr;
        lv_obj_t* button_ = nullptr;
        lv_obj_t* label_ = nullptr;

        void createUi();
        void onButtonClicked();
        void updateUi();

        static void buttonEventCallback(lv_event_t* event);
    };

} // namespace ghost_v5

#endif // GHOST_V5_SCREEN_AUTON_SELECTOR_HPP_

