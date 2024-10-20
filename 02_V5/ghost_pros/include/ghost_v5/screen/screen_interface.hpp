/*
 *   Copyright (c) 2024 Maxx Wilson
 *   All rights reserved.

 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:
 
 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.
 
 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

#pragma once

#include <deque>
#include <sstream>
#include <string>
#include <vector>

#include <mutex>
#include "pros/apix.h"

namespace ghost_v5 {

class ScreenInterface {
public:
	ScreenInterface();

	/**
	 * @brief Concatenates arbitrary arguments into single string and adds to queue
	 * e.x. addToPrintQueue("1", 2, std::string("3"), 4.00)
	 */
	template< typename ... Args >
	void addToPrintQueue(Args&& ... args){
		std::ostringstream oss;
		(oss << ... << std::forward<Args>(args));

		// Acquire queue lock and add string
		std::unique_lock lock(print_queue_lock_);
		print_queue_.emplace_back(oss.str());

		// Ensure queue keeps only latest screen worth of data
		while(print_queue_.size() > MAX_ROWS - 1){
			print_queue_.pop_front();
		}
	}

	void reset();

	void updateScreen();

	void updateLastConnectionTime();

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
	uint32_t last_connection_time_ = 0;
	int curr_row_ = 2;

	// V5 Screen Params
	const int MAX_ROWS = 11;
	const int LINE_PIX_HEIGHT = 20;
	const int SCREEN_PIX_WIDTH = 480;
	const int SCREEN_PIX_HEIGHT = 240;
	const int SCREEN_WIDTH_CHAR_LIM = 48;
	const int REFRESH_RATE_MS = 10;
};

} // namespace ghost_v5