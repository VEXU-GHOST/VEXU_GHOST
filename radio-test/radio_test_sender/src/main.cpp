/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       shark                                                     */
/*    Created:      3/12/2026, 3:06:37 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"

using namespace vex;

// radio plugged into Smart Port 11
vex::brain Brain;
vex::serial_link Link(PORT11, "team_radio", linkType::manager);

int main() {

	// wait until radios connect
	while(!Link.isLinked()) {
		Brain.Screen.printAt(10, 50, "Waiting for link...");
		wait(100, msec);
	}

	uint8_t data[4] = {0,1,2,3};

	while(true) {

		Link.send(data, sizeof(data));

		Brain.Screen.clearScreen();
		Brain.Screen.print("Sent packet: %d", data[0]);

		data[0]++;

		wait(1000, msec);
	}
}