/*----------------------------------------------------------------------------*/
/* Description: Example Worker VEXlink code */
/*----------------------------------------------------------------------------*/
#include "vex.h"
using namespace vex;
// Instance of message link class
vex::serial_link LinkB( PORT11, "vex_robotics_team_1234_B", linkType::worker );
// callbacks only print to terminal
void
receive_message( uint8_t *buffer, int32_t length ) {
printf("receive_message: %ld bytes were receine\n", length );
for(int i=0;i<length;i++)
printf("%02X ", buffer[i] );
printf("\n");
}
int main() {
// register callback
LinkB.received( receive_message );
// show link status
while(1) {
Brain.Screen.printAt( 10, 50, true, "Link: %s", LinkB.isLinked() ? "ok" : "--" );
// Allow other tasks to run
this_thread::sleep_for(50);
}
}