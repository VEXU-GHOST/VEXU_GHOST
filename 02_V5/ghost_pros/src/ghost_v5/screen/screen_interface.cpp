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

#include "ghost_v5/screen/screen_interface.hpp"

#include <algorithm>
#include <cctype>

namespace ghost_v5 {

void ScreenInterface::clearPrintQueue(){
	std::unique_lock lock(print_queue_lock_);
	print_queue_.clear();
}

std::deque<std::string> ScreenInterface::takePrintQueue(){
	std::deque<std::string> out;
	std::unique_lock lock(print_queue_lock_);
	out.swap(print_queue_);
	return out;
}

void ScreenInterface::reset(){
	clearPrintQueue();
}

void ScreenInterface::setTitle(const std::string& title){
	std::unique_lock lock(title_lock_);
	title_string_ = title;
}

std::string ScreenInterface::getTitle(){
	std::unique_lock lock(title_lock_);
	return title_string_;
}

void ScreenInterface::showError(const std::string& msg){
	reset();
	setTitle("ERROR");
	addToPrintQueue(msg);
}

void ScreenInterface::updateLastConnectionTime(){
	last_connection_time_ = pros::millis();
}

std::vector<std::string> ScreenInterface::wrapStringToLineLength(std::string str, int line_len){
	std::vector<std::string> output;
	while(str.size() > line_len){
		// Get first X characters of string, where X is the max characters per line on the Brain Screen.
		auto string_screen_len = str.substr(0, line_len);

		// Check if we can split at spaces instead of mid-word
		auto split_index = string_screen_len.find_last_of(" ");

		if(split_index == std::string::npos){
			split_index = line_len;
		}

		// Add string to vector
		output.emplace_back(str.substr(0, split_index));

		// Remove printed portion from error string
		str = str.substr(split_index);

		// Trim leading whitespace
		str.erase(str.begin(), std::find_if(str.begin(), str.end(), [](int c){
				return !std::isspace(c);
			}));
	}
	output.emplace_back(str);
	return output;
}

} // namespace ghost_v5