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

#include <atomic>
#include <deque>
#include <sstream>
#include <string>
#include <vector>

#include <mutex>
#include "pros/apix.h"

namespace ghost_v5 {

/**
 * @brief Render-agnostic screen interface.
 *
 * Holds the print queue, title, and last ROS connection time. Methods here are safe
 * to call from any task and never draw; AutonSelector decides how and when to render.
 */
class ScreenInterface {
public:
	ScreenInterface() = default;
	virtual ~ScreenInterface() = default;

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
		while(print_queue_.size() > MAX_QUEUE_SIZE){
			print_queue_.pop_front();
		}
	}

	virtual void reset();

	virtual void setTitle(const std::string& title);

	/**
	 * @brief Puts the screen into an error state. Default matches the old behavior:
	 * clear, set title to ERROR, and print the message.
	 */
	virtual void showError(const std::string& msg);

	void updateLastConnectionTime();

	static std::vector<std::string> wrapStringToLineLength(std::string str, int line_len);

protected:
	void clearPrintQueue();

	std::string getTitle();

	// Swaps out and returns all queued messages.
	std::deque<std::string> takePrintQueue();

	pros::Mutex print_queue_lock_;
	std::deque<std::string> print_queue_;

	pros::Mutex title_lock_;
	std::string title_string_ = "STATUS";

	// Written by the serial reader task, read by the render task.
	std::atomic<uint32_t> last_connection_time_{0};

	static constexpr std::size_t MAX_QUEUE_SIZE = 10;
};

} // namespace ghost_v5
