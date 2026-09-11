#pragma once

#include "uas_sim/UAS_structs.h"
#include <vector>

namespace results {

  bool makeCsv(const std::vector<double>& time_history,
    const std::vector<UAS_state>& estimation_history);

}