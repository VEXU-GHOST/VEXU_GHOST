#include "ghost_v5/screen/screen_interface.hpp"

#include <iostream>

namespace ghost_v5 {

ScreenInterface::ScreenInterface() :
	title_string_("STATUS"){
	pros::screen::set_eraser(COLOR_BLACK);
	pros::screen::set_pen(COLOR_WHITE);

	// For some reason, we can't use screen until a certain amount of time has elapsed after initialization() begins.
	auto start = pros::millis();
	while((pros::millis() - start) < 100){
		pros::screen::erase();
		pros::delay(1);
	}

	// Update Title
	pros::screen::print(pros::text_format_e_t::E_TEXT_LARGE_CENTER, 0, title_string_.c_str());
}

void ScreenInterface::clearPrintQueue(){
	std::unique_lock lock(print_queue_lock_);
	print_queue_.clear();
}

void ScreenInterface::reset(){
	clearPrintQueue();
	pros::screen::erase();
	pros::screen::print(pros::text_format_e_t::E_TEXT_LARGE_CENTER, 0, title_string_.c_str());
	curr_row_ = 2;
}

void ScreenInterface::updateScreen(){
	static int count = 0;
	static bool heartbeat_toggle = true;

	// Only update at refresh frequency
	if(((pros::millis() - last_update_time_) < REFRESH_RATE_MS)){
		return;
	}

	// This pulses the title on and off at regular rate so we know the program is connected to ROS.
	if((count % (1000 / REFRESH_RATE_MS)) == 0){
		std::string title_str;
		// enables or disables based on time AND blinks based on heartbeat toggle
		if (last_connection_time_ + 1000 > pros::millis() && heartbeat_toggle)
			title_str = std::string("------- ") + title_string_ + std::string(" -------");
		else
			title_str = std::string("        ") + title_string_ + std::string("        ");

		pros::screen::print(pros::text_format_e_t::E_TEXT_LARGE_CENTER, 0, title_str.c_str());
		heartbeat_toggle = !heartbeat_toggle;
	}
	count++;


	// Return early if we have nothing to process
	if(print_queue_.empty()){
		return;
	}

	// Process the batch of msgs in the queue, printing until its empty
	std::unique_lock lock(print_queue_lock_);
	while(!print_queue_.empty()){
		auto new_str_lines = wrapStringToLineLength(print_queue_.front(), SCREEN_WIDTH_CHAR_LIM);
		for(const auto& line : new_str_lines){
			if(curr_row_ <= MAX_ROWS){
				pros::screen::print(pros::text_format_e_t::E_TEXT_SMALL, curr_row_++, line.c_str());
			}
			else{
				pros::screen::scroll_area(0, 2 * LINE_PIX_HEIGHT + 1, SCREEN_PIX_WIDTH, SCREEN_PIX_HEIGHT, LINE_PIX_HEIGHT);
				pros::screen::print(pros::text_format_e_t::E_TEXT_SMALL, MAX_ROWS, line.c_str());
			}
		}
		print_queue_.pop_front();
	}

	last_update_time_ = pros::millis();
}

void ScreenInterface::setTitle(const std::string& title){
	title_string_ = title;
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