/*----------------------------------------------------------------------------*/
/* Description: Example Manger VEXlink code */
/*----------------------------------------------------------------------------*/

#include "vex.h"
using namespace vex;

// Instance of message link class
vex::message_link LinkA( PORT11, "vex_robotics_team_1234_A", linkType::manager );

// Task to periodically send test messages to worker robot
int sendTask() 
{
  // wait for link
  while( !LinkA.isLinked() )
  this_thread::sleep_for(50);

  // send demo messages
  while(1) 
  {
    LinkA.send("drive");
    this_thread::sleep_for(500);

    LinkA.send("go_forward", 100 );
    this_thread::sleep_for(500);
    
    LinkA.send("start_motor", PORT3, 50.0 );
    this_thread::sleep_for(1000);
  }

  return 0;
}

int main() 
{
  // start demo task
  thread t1( sendTask );

  // show link status
  while(1) 
  {
    Brain.Screen.printAt( 10, 50, true, "Link: %s", LinkA.isLinked() ? "ok" : "--" );

    // Allow other tasks to run
    this_thread::sleep_for(10);
  }
}