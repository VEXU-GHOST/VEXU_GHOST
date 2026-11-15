#ifndef CHARLIE_ONBOARDING__RICK_ROLL_HPP_
#define CHARLIE_ONBOARDING__RICK_ROLL_HPP_

#include <string>

class RickRoll
{
public:
  RickRoll();
  const std::string & next();

private:
  std::string normal_;
  std::string mirrored_;
  bool flipped_ = false;
};

#endif  // CHARLIE_ONBOARDING__RICK_ROLL_HPP_
