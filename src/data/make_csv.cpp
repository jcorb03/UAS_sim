#include "uas_sim/data/make_csv.h"

#include <filesystem>
#include <fstream>
#include <iostream>

namespace results {

  bool makeCsv(const std::vector<double>& time_history,
    const std::vector<UAS_state>& estimation_history)
  {
    const std::filesystem::path output_path =
      std::filesystem::current_path() / "results" / "results.csv";

    std::ofstream myfile(output_path);

    if (!myfile.is_open()) {
      return false;
    }

    myfile << "Time,X,Y,V,Heading\n";

    for (std::size_t i = 0; i < time_history.size(); ++i) {
      myfile << time_history.at(i) << ","
        << estimation_history.at(i).x << ","
        << estimation_history.at(i).y << ","
        << estimation_history.at(i).v << ","
        << estimation_history.at(i).heading << '\n';
    }


    return true;
  }

}

