#include "mission_planner.hpp"
#include "default_planner.hpp"
#include "arrival_checker.hpp"
#include "mission_planner_runner.hpp"
#include <iostream>
#include <random>
#include <string>   // std::string
#include <cmath>    // M_PI

#include <lanelet2_projection/UTM.h>
#include <lanelet2_core/primitives/GPSPoint.h>

using namespace autoware::mission_planner_universe;

// Expose standalone main only when explicitly requested by build.
#ifdef BUILD_STANDALONE_MISSION_PLANNER
int main(int argc, char ** argv)
{
  std::string osm_file = (argc > 1) ? argv[1] : "../sample-map-planning/lanelet2_map.osm";
  mission_planner_runner::run_once(osm_file);
  return 0;
}
#endif