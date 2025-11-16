/*----------------------------------------------------------------------------*/
/* Description: Example Worker VEXlink code */
/*----------------------------------------------------------------------------*/
#include "vex.h"
using namespace vex;

// Instance of message link class
vex::message_link LinkA( PORT11, "vex_robotics_team_1234_A", linkType::worker );

// callbacks only print to terminal
void drive_received( const char *message, const char *linkname, double value ) 
{
  printf("%s: was received on '%s' link\n", message, linkname );
}

void go_forward( const char *message, const char *linkname, double value ) 
{
  printf("%s: was received on '%s' link with value %.2f\n", message, linkname, value );
}

void start_motor( const char *message, const char *linkname, int32_t index, double value ) 
{
  printf("%s: was received on '%s' link wth index %d and value %.2f\n", message, linkname, index, value );
}

void receive_message( const char *message, const char *linkname, int32_t index, double value ) 
{
  printf("receive_message: %s was receined on %s\n", message, linkname );
}

int main() 
{
  // register callbacks
  LinkA.received( "drive", drive_received );
  LinkA.received( "go_forward", go_forward );
  LinkA.received( "start_motor", start_motor );

  // A generic callback can be registered as well as specific message callbacks
  LinkA.received( receive_message );

  // show link status
  while(1) 
  {
    Brain.Screen.printAt( 10, 50, true, "Link: %s", LinkA.isLinked() ? "ok" : "--" );
    
    // Allow other tasks to run
    this_thread::sleep_for(50);
  }
}