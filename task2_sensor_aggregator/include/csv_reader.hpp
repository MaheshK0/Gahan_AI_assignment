#pragma once
#include <string>
#include <vector>

namespace agg {

// Minimal hand-rolled CSV reader -- sufficient for the well-formed,
// comma-separated, single-line-per-record reference files used in this
// task. Not a general-purpose CSV library (no quoted-field/escaping
// support) by design, per the assignment's "avoid third-party libraries,

class CsvReader {
public:
    // Reads the whole file and returns rows (including the header row at
    // index 0) as vectors of trimmed string fields.
    static std::vector<std::vector<std::string>> readAll(const std::string& path);
};

} // namespace agg
