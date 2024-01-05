#pragma once

#include <deque>
#include <string>
#include <vector>

#include <mutex>
#include "pros/apix.h"

namespace ghost_v5 {

class ScreenInterface {
public:
	ScreenInterface();

	void addToPrintQueue(std::string msg);

	void reset();

	void updateScreen();

	void setTitle(const std::string& title);

	int getRefreshRateMilliseconds(){
		return REFRESH_RATE_MS;
	}

	static std::vector<std::string> wrapStringToLineLength(std::string str, int line_len);

private:
	void clearPrintQueue();

	std::string title_string_;
	pros::Mutex print_queue_lock_;
	std::deque<std::string> print_queue_;
	std::vector<std::string> screen_buffer_;
	uint32_t last_update_time_ = 0;
	int curr_row_ = 2;

	// V5 Screen Params
	const int MAX_ROWS = 11;
	const int LINE_PIX_HEIGHT = 20;
	const int SCREEN_PIX_WIDTH = 240;
	const int SCREEN_PIX_HEIGHT = 480;
	const int SCREEN_WIDTH_CHAR_LIM = 48;
	const int REFRESH_RATE_MS = 10;
};

} // namespace ghost_v5