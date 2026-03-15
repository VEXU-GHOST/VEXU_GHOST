/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       shark                                                     */
/*    Created:      3/12/2026, 3:07:07 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"

using namespace vex;

// A global instance of vex::brain used for printing to the V5 brain screen
vex::brain Brain;
vex::serial_link Link(PORT11, "vex_team_ghost", linkType::worker);

void receive_callback(uint8_t *buffer, int32_t length) {

	Brain.Screen.clearScreen();

	Brain.Screen.print("Received %d bytes", length);
	Brain.Screen.newLine();

	for(int i=0; i<length; i++) {
		Brain.Screen.print("%d ", buffer[i]);
	}
}

int main() {

	Link.received(receive_callback);

	while(true) {

		if(Link.isLinked()) {
			Brain.Screen.printAt(10,50,"Link: Connected");
		}
		else {
			Brain.Screen.printAt(10,50,"Link: Waiting");
		}
	}
}