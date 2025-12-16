#include <string>
#include <vector>
#include <fstream>
#include <algorithm>
#include "Utils/IOUtils.h"
#include <unordered_set>
#include <utility>

void split_string(std::string string, std::string delimiter, std::vector<std::string> &results)
{
    size_t first_delimiter;

    while ((first_delimiter = string.find_first_of(delimiter)) != string.npos) {
        if (first_delimiter > 0) {
            results.push_back(string.substr(0, first_delimiter));
        }
        string = string.substr(first_delimiter + 1);
    }

    if (string.length() > 0) {
        results.push_back(string);
    }
}

// Note: PairHash is now defined in Utils/Definitions.h and available globally

bool load_edge_file(std::string edge_file, std::vector<Edge> &edges_out, size_t &graph_size) {
    size_t max_node_num = 0;
    std::ifstream file2_edge;


    file2_edge.open(edge_file);
    if (!file2_edge.is_open()) {
        return false;
    }

    std::unordered_set<std::pair<size_t,size_t>, PairHash> seen;  // (s,t) we've already added
    std::string line_edge;

    while (std::getline(file2_edge, line_edge)) {
        if (line_edge.empty()) continue;

        std::vector<std::string> parts;
        split_string(line_edge, " ", parts);
        if (parts.size() < 4) continue; // or return false;

        size_t s = std::stoul(parts[0]);
        size_t t = std::stoul(parts[1]);
        double l = std::stod(parts[2]);
        const std::string& type_str = parts[3];

        // If you also want to drop self-loops:
        // if (s == t) continue;

        // If you want the graph to be *undirected* unique, canonicalize:
        // if (t < s) std::swap(s, t);

        auto key = std::make_pair(s, t);
        if (seen.find(key) != seen.end()) {
            continue; // duplicate (s,t) → skip
        }
        seen.insert(key);

        double type = (type_str == "on") ? 1.0 : 0.0; // on-road / off-road
        if(type_str != "on" && type_str != "off"){
            std::cout << "!! type is not on or off, try change CRLF -> LF" << std::endl;
        }
        Edge e(s, t, {type, l});
        edges_out.push_back(e);

        max_node_num = std::max({max_node_num, e.source, e.target});
    }

    // If node indices start at 0, the graph "size" is usually max_index + 1.
    graph_size = max_node_num + 1;

    return true;
}

// Load multiple .gr files for multi-objective pathfinding
// File format:
//   - First (n-1) files: contain SAFETY probabilities (values in [0,1])
//     These are converted to RISK probabilities using: risk = 1 - safety
//   - Last file: contains LENGTH/COST values (numeric, not probabilities)
// Example: if you have 5 .gr files:
//   - Files 0-3: safety probabilities → converted to risk
//   - File 4: length values → used as-is
bool load_gr_files(std::vector<std::string> gr_files, std::vector<Edge> &edges_out, size_t &graph_size){
  size_t          max_node_num = 0;
  auto number_of_files = gr_files.size();
  size_t count = 0;

  for (auto gr_file: gr_files){
    count++;
    std::ifstream file(gr_file.c_str());
    
    if (file.is_open() == false){
      std::cerr << "cannot open the gr file " << gr_file << std::endl;
      return false;
    }

    std::string line;
    int idx_edge = 0;
    while (file.eof() == false) {
        std::getline(file, line);

        if (line == "") {
            break;
        }

        std::vector<std::string> decomposed_line;
        split_string(line, " ", decomposed_line);

        std::string type = decomposed_line[0];
        if ((std::strcmp(type.c_str(),"c") == 0) || (std::strcmp(type.c_str(),"p") == 0)) {
            continue; //comment or problem lines, not part of the graph
        }

        if (std::strcmp(type.c_str(),"a") == 0) { //arc
          if (idx_edge < (int)edges_out.size() - 1){
            try {
              if (
                  (stoul(decomposed_line[1]) != edges_out[idx_edge].source) ||
                  (stoul(decomposed_line[2]) != edges_out[idx_edge].target)) {
                // arc_sign src dest should be same in both files
                std::cerr << "file inconsistency" << std::endl;
                return false;
              }
            } catch (const std::invalid_argument& e) {
              std::cerr << "Error parsing line in file " << gr_file << ": '" << line << "'" << std::endl;
              std::cerr << "Failed to parse number from decomposed_line. Line parts:" << std::endl;
              for (size_t i = 0; i < decomposed_line.size(); i++) {
                std::cerr << "  [" << i << "]: '" << decomposed_line[i] << "'" << std::endl;
              }
              throw;
            }
            // For NAMOAdr: first n-1 files are risk values (convert to 1-risk), last file is cost
            if (count < number_of_files){
                edges_out[idx_edge].cost.push_back(1 - std::stod(decomposed_line[3]));
            }else{
                edges_out[idx_edge].cost.push_back(std::stod(decomposed_line[3]));
            }
          }else{
            // For NAMOAdr: first n-1 files are risk values (convert to 1-risk), last file is cost
            double cost;
            if (count < number_of_files){
                cost = 1 - std::stod(decomposed_line[3]);
            }else{
                cost = std::stod(decomposed_line[3]);
            }
            try {
              size_t s = std::stoul(decomposed_line[1]);
              size_t t = std::stoul(decomposed_line[2]);
              Edge e(s, t, {cost});
              edges_out.push_back(e);
              max_node_num = std::max({max_node_num, e.source, e.target});
            } catch (const std::invalid_argument& e) {
              std::cerr << "Error parsing edge in file " << gr_file << ": '" << line << "'" << std::endl;
              std::cerr << "Failed to parse source/target node. Line parts:" << std::endl;
              for (size_t i = 0; i < decomposed_line.size(); i++) {
                std::cerr << "  [" << i << "]: '" << decomposed_line[i] << "'" << std::endl;
              }
              throw;
            }
          }
        }
        idx_edge ++;
    }
    file.close();
  }
  graph_size = max_node_num;
  return true;
}

bool load_queries(std::string query_file, std::vector<std::pair<size_t, size_t>> &queries_out) {
    std::ifstream   file(query_file.c_str());

    if (file.is_open() == false) {
        return false;
    }

    std::string line;
    while (file.eof() == false) {
        std::getline(file, line);

        if (line == "") {
            break;
        } else if (line[0] == '#') {
            continue; // Commented out queries
        }

        std::vector<std::string> decomposed_line;
        split_string(line, ",", decomposed_line);

        std::pair<size_t, size_t> query = {std::stoul(decomposed_line[0]), std::stoul(decomposed_line[1])};
        queries_out.push_back(query);
    }
    return true;
}

bool load_edge_doration_file(std::string gr_file, std::vector<Edge> &edges_out){
    std::ifstream file(gr_file.c_str());
    if (file.is_open() == false){
      std::cerr << "cannot open the gr file " << gr_file << std::endl;
      return false;
    }
    std::string line;
    int idx_edge = 0;
    while (file.eof() == false) {
        std::getline(file, line);

        if (line == "") {
            break;
        }
        if (line.back() == '\r') {
            line.pop_back();
        }
        std::vector<std::string> decomposed_line;
        split_string(line, " ", decomposed_line);

        std::string type = decomposed_line[0];
        if ((std::strcmp(type.c_str(),"c") == 0) || (std::strcmp(type.c_str(),"p") == 0)) {
            continue; //comment or problem lines, not part of the graph
        }

        if (std::strcmp(type.c_str(),"a") == 0) {
            if (idx_edge < ((int)edges_out.size() - 1)){
                edges_out[idx_edge].set_doration(std::stod(decomposed_line[3]));
                idx_edge++;
            }
        }
    }
    std::cout << "Finished loading edge collision-doration file: " << gr_file << std::endl;
    return true;
}