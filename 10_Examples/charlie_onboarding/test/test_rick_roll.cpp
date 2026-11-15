#include <string>

#include "charlie_onboarding/rick_roll.hpp"
#include "gtest/gtest.h"

TEST(RickRollTest, AlternatesBetweenOriginalAndMirrored)
{
  RickRoll rick;
  const std::string original = rick.next();
  const std::string mirrored = rick.next();
  EXPECT_NE(original, mirrored);
  EXPECT_EQ(original, rick.next());
  EXPECT_EQ(mirrored, rick.next());
  EXPECT_EQ(original.size(), mirrored.size());
}

TEST(RickRollTest, MirrorsBrailleDotsAndLineOrder)
{
  RickRoll rick;
  const std::string original = rick.next();
  const std::string mirrored = rick.next();
  // The final glyph of the first row is a full cell; the asymmetric center
  // verifies the dots are mirrored, not merely the order of bytes.
  const std::string first_row = original.substr(0, original.find('\n'));
  const std::string mirrored_row = mirrored.substr(0, mirrored.find('\n'));
  EXPECT_EQ(first_row.size(), mirrored_row.size());
  EXPECT_NE(first_row, mirrored_row);
  EXPECT_EQ(mirrored_row.substr(0, 3), "⣿");
  EXPECT_NE(mirrored_row.find("⡿"), std::string::npos);
}
