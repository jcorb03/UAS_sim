#include "uas_sim/data/make_csv.h"

#include <filesystem>
#include <fstream>
#include <iostream>

namespace results {

#ifndef UAS_SIM_PROJECT_DIR
#define UAS_SIM_PROJECT_DIR "."
#endif

  bool makeCsv(
    const std::vector<double>& time_history,
    const std::vector<std::vector<UAS_state>>& estimation_history)
  {
    const std::filesystem::path output_path =
      std::filesystem::path{ UAS_SIM_PROJECT_DIR } / "results" / "results.csv";

    if (estimation_history.empty()) {
      return false;
    }

    for (const auto& uav_history : estimation_history) {
      if (uav_history.size() != time_history.size()) {
        return false; // Every UAV needs one state per timestamp.
      }
    }

    std::filesystem::create_directories(output_path.parent_path());

    std::ofstream myfile(output_path);
    if (!myfile.is_open()) {
      return false;
    }

    myfile << "Time";
    for (std::size_t uav = 0; uav < estimation_history.size(); ++uav) {
      myfile << ",UAV" << uav << "_X"
        << ",UAV" << uav << "_Y"
        << ",UAV" << uav << "_V"
        << ",UAV" << uav << "_Heading";
    }
    myfile << '\n';

    for (std::size_t time = 0; time < time_history.size(); ++time) {
      myfile << time_history[time];

      for (std::size_t uav = 0; uav < estimation_history.size(); ++uav) {
        const UAS_state& state = estimation_history[uav][time];

        myfile << ',' << state.x
          << ',' << state.y
          << ',' << state.v
          << ',' << state.heading;
      }

      myfile << '\n';
    }

    return true;
  }

}

