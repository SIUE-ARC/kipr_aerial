#include "DroneInterface.h"

class Px4 : DroneInterface {
private:
  /* data */

public:
  Px4 (ros::NodeHandle nh);
  virtual ~Px4 ();

  void takeoff();
  void land();
  void goTo(int x, int y, int z);
};
