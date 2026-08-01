#include "csv_reader.hpp"

#include <fstream>
#include <sstream>
#include <stdexcept>

namespace agg {

std::vector<std::vector<std::string>> CsvReader::readAll(const std::string& path) {
    std::ifstream f(path);
    if (!f) {
        throw std::runtime_error("Could not open CSV file: " + path);
    }
    std::vector<std::vector<std::string>> rows;
    std::string line;
    while (std::getline(f, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (line.empty()) continue;
        std::vector<std::string> fields;
        std::stringstream ss(line);
        std::string field;
        while (std::getline(ss, field, ',')) {
            fields.push_back(field);
        }
        rows.push_back(std::move(fields));
    }
    return rows;
}

} // namespace agg
