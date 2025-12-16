#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <memory>
#include <cmath>
#include <chrono>
#include <iomanip>

// Point structure for 2D coordinates
struct Point {
    double x, y;
    Point(double x = 0, double y = 0) : x(x), y(y) {}
};

// Shadow structure representing obstacle uncertainty region
// Input format: top_left_x, top_left_y, width, height, risk
struct Shadow {
    Point bottom_left;
    Point top_left;
    Point top_right;
    Point bottom_right;
    double risk;

    Shadow(Point tl, double width, double height, double r)
        : top_left(tl), risk(r) {
        bottom_left = Point(tl.x, tl.y - height);
        top_right = Point(tl.x + width, tl.y);
        bottom_right = Point(tl.x + width, tl.y - height);
    }
};

// Obstacle containing multiple shadows
struct Obstacle {
    std::vector<Shadow> shadows;
};

// Edge structure
struct Edge {
    size_t source, target;
    double length;
    double computation_time_us; // microseconds
};

// Utility: split string by delimiter
void split_string(const std::string& str, const std::string& delim, std::vector<std::string>& tokens) {
    tokens.clear();
    size_t start = 0, end = 0;
    while ((end = str.find(delim, start)) != std::string::npos) {
        if (end != start) {
            tokens.push_back(str.substr(start, end - start));
        }
        start = end + delim.length();
    }
    if (start < str.length()) {
        tokens.push_back(str.substr(start));
    }
}

// Geometry: Check if point q lies on segment pr
bool onSegment(Point p, Point q, Point r) {
    return q.x <= std::max(p.x, r.x) && q.x >= std::min(p.x, r.x) &&
           q.y <= std::max(p.y, r.y) && q.y >= std::min(p.y, r.y);
}

// Geometry: Find orientation of ordered triplet (p, q, r)
// Returns: 0 = colinear, 1 = clockwise, 2 = counterclockwise
double orientation(Point p, Point q, Point r) {
    double val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);
    if (std::abs(val) < 1e-9) return 0.0;
    return (val > 0) ? 1.0 : 2.0;
}

// Geometry: Check if segments p1-q1 and p2-q2 intersect
bool doIntersect(Point p1, Point q1, Point p2, Point q2) {
    double o1 = orientation(p1, q1, p2);
    double o2 = orientation(p1, q1, q2);
    double o3 = orientation(p2, q2, p1);
    double o4 = orientation(p2, q2, q1);

    if (o1 != o2 && o3 != o4) return true;
    if (o1 == 0 && onSegment(p1, p2, q1)) return true;
    if (o2 == 0 && onSegment(p1, q2, q1)) return true;
    if (o3 == 0 && onSegment(p2, p1, q2)) return true;
    if (o4 == 0 && onSegment(p2, q1, q2)) return true;

    return false;
}

// Geometry: Check if point is inside shadow (rectangle)
bool is_in_shadow(const Shadow& shadow, const Point& p) {
    return shadow.top_left.x <= p.x && p.x <= shadow.top_right.x &&
           shadow.top_left.y >= p.y && p.y >= shadow.bottom_left.y;
}

// Load obstacles with shadows from file
std::vector<std::shared_ptr<Obstacle>> load_obstacles(const std::string& filepath) {
    std::vector<std::shared_ptr<Obstacle>> obstacles;
    std::ifstream file(filepath);

    if (!file.is_open()) {
        std::cerr << "Error: Could not open obstacles file: " << filepath << std::endl;
        return obstacles;
    }

    std::string line;
    std::shared_ptr<Obstacle> current_obstacle = nullptr;
    int line_number = 0;

    while (std::getline(file, line)) {
        line_number++;
        if (line.empty()) continue;

        std::vector<std::string> tokens;
        split_string(line, " ", tokens);

        if (tokens.empty()) continue;

        if (tokens[0] == "#") {
            // Start new obstacle
            if (current_obstacle != nullptr) {
                obstacles.push_back(current_obstacle);
            }
            current_obstacle = std::make_shared<Obstacle>();
        } else if (tokens.size() >= 5) {
            // Parse shadow: x y width height risk
            if (current_obstacle == nullptr) {
                std::cerr << "Warning: Shadow data at line " << line_number
                          << " appears before first obstacle marker '#'. Creating implicit obstacle." << std::endl;
                current_obstacle = std::make_shared<Obstacle>();
            }

            double x = std::stod(tokens[0]);
            double y = std::stod(tokens[1]);
            double width = std::stod(tokens[2]);
            double height = std::stod(tokens[3]);
            double risk = std::stod(tokens[4]);

            current_obstacle->shadows.emplace_back(Point(x, y), width, height, risk);
        }
    }

    // Add last obstacle
    if (current_obstacle != nullptr) {
        obstacles.push_back(current_obstacle);
    }

    file.close();

    // Check for empty obstacles
    for (size_t i = 0; i < obstacles.size(); ++i) {
        if (obstacles[i]->shadows.empty()) {
            std::cerr << "Warning: Obstacle " << i << " has no shadows!" << std::endl;
        }
    }

    std::cout << "Loaded " << obstacles.size() << " obstacles" << std::endl;
    return obstacles;
}

// Load vertex positions from file
std::vector<Point> load_vertex_positions(const std::string& filepath) {
    std::vector<Point> vertices;
    std::ifstream file(filepath);

    if (!file.is_open()) {
        std::cerr << "Error: Could not open vertex positions file: " << filepath << std::endl;
        return vertices;
    }

    std::string line;
    while (std::getline(file, line)) {
        if (line.empty()) continue;

        std::vector<std::string> tokens;
        split_string(line, " ", tokens);

        if (tokens.size() >= 3) {
            // vertex_id x y
            double x = std::stod(tokens[1]);
            double y = std::stod(tokens[2]);
            vertices.emplace_back(x, y);
        }
    }

    file.close();
    std::cout << "Loaded " << vertices.size() << " vertices" << std::endl;
    return vertices;
}

// Load edges from adjacency list file and compute lengths
std::vector<Edge> load_edges(const std::string& filepath, const std::vector<Point>& vertices) {
    std::vector<Edge> edges;
    std::ifstream file(filepath);

    if (!file.is_open()) {
        std::cerr << "Error: Could not open edges file: " << filepath << std::endl;
        return edges;
    }

    std::string line;
    size_t vertex_id = 0;

    while (std::getline(file, line)) {
        if (line.empty()) continue;

        std::vector<std::string> tokens;
        split_string(line, " ", tokens);

        if (tokens.size() < 2) continue;

        // First token is the source vertex (though we use line number as source)
        size_t source = vertex_id;

        // Remaining tokens are neighbor vertices
        for (size_t i = 1; i < tokens.size(); ++i) {
            size_t target = std::stoull(tokens[i]);

            // Bounds check
            if (target >= vertices.size()) {
                std::cerr << "Warning: Edge references invalid vertex " << target
                          << " (max: " << vertices.size() - 1 << ")" << std::endl;
                continue;
            }

            // Compute Euclidean distance as edge length
            double dx = vertices[target].x - vertices[source].x;
            double dy = vertices[target].y - vertices[source].y;
            double length = std::sqrt(dx * dx + dy * dy);

            Edge e;
            e.source = source;
            e.target = target;
            e.length = length;
            e.computation_time_us = 0.0; // Will be computed later
            edges.push_back(e);
        }

        vertex_id++;
    }

    file.close();
    std::cout << "Loaded " << edges.size() << " edges" << std::endl;
    return edges;
}

// Compute safety vector for an edge (safety = 1 - risk)
// Finds the maximum risk among all shadows that the edge intersects
std::vector<double> compute_edge_safety_vector(
    Point p1, Point p2,
    const std::vector<std::shared_ptr<Obstacle>>& obstacles) {

    std::vector<double> safety_vector;

    for (const auto& obstacle : obstacles) {
        double max_risk = 0.0;

        for (const auto& shadow : obstacle->shadows) {
            // Check if edge intersects shadow boundary
            bool intersects =
                doIntersect(shadow.top_left, shadow.top_right, p1, p2) ||
                doIntersect(shadow.top_right, shadow.bottom_right, p1, p2) ||
                doIntersect(shadow.bottom_right, shadow.bottom_left, p1, p2) ||
                doIntersect(shadow.bottom_left, shadow.top_left, p1, p2);

            if (intersects) {
                if (shadow.risk > max_risk) {
                    max_risk = shadow.risk;
                }
            } else {
                // Check if endpoint (point2) is inside shadow
                bool inside = is_in_shadow(shadow, p2);
                if (inside) {
                    if (shadow.risk > max_risk) {
                        max_risk = shadow.risk;
                    }
                }
            }
        }

        // Convert risk to safety: safety = 1 - risk
        safety_vector.push_back(1.0 - max_risk);
    }

    return safety_vector;
}

// Write .gr file in the correct format
void write_gr_file(const std::string& filepath,
                   const std::vector<Edge>& edges,
                   const std::vector<std::vector<double>>& edge_values,
                   size_t num_vertices) {
    std::ofstream file(filepath);

    if (!file.is_open()) {
        std::cerr << "Error: Could not create file: " << filepath << std::endl;
        return;
    }

    // Write header with comments (matching the format from examples)
    file << "c\n";
    file << "p sp " << num_vertices << " " << edges.size() << "\n";
    file << "c graph contains " << num_vertices << " nodes and " << edges.size() << " arcs\n";
    file << "c\n";

    // Write edges (use default precision, removes trailing zeros)
    for (size_t i = 0; i < edges.size(); ++i) {
        double value = edge_values[i][0];
        // Format value: if it's a whole number, print as integer; otherwise use appropriate precision
        file << "a " << edges[i].source << " " << edges[i].target << " ";
        if (std::abs(value - std::round(value)) < 1e-9) {
            file << static_cast<int>(std::round(value)) << ".0\n";
        } else {
            file << std::defaultfloat << value << "\n";
        }
    }

    file.close();
    std::cout << "Written: " << filepath << std::endl;
}

int main(int argc, char* argv[]) {
    if (argc < 5) {
        std::cout << "Usage: " << argv[0]
                  << " <obstacle_shadows_file> <vertex_positions_file> <edges_file> <output_prefix>\n";
        std::cout << "\nExample:\n";
        std::cout << "  " << argv[0]
                  << " obstacle_shadow_definitions.txt vertex_positions.txt edges.txt output/file\n";
        std::cout << "\nOutput files will be: output/file_0.gr, output/file_1.gr, ..., output/file_N.gr\n";
        return 1;
    }

    std::string obstacles_file = argv[1];
    std::string vertices_file = argv[2];
    std::string edges_file = argv[3];
    std::string output_prefix = argv[4];

    // Load data
    auto obstacles = load_obstacles(obstacles_file);
    auto vertices = load_vertex_positions(vertices_file);
    auto edges = load_edges(edges_file, vertices);

    if (obstacles.empty() || vertices.empty() || edges.empty()) {
        std::cerr << "Error: Failed to load input data" << std::endl;
        return 1;
    }

    size_t num_objectives = obstacles.size();
    size_t num_edges = edges.size();

    // Storage for all edge values: [edge_idx][objective_idx]
    std::vector<std::vector<std::vector<double>>> all_values(num_objectives);
    for (size_t obj = 0; obj < num_objectives; ++obj) {
        all_values[obj].resize(num_edges);
    }

    // Storage for edge lengths and computation times
    std::vector<std::vector<double>> lengths(num_edges);
    std::vector<std::vector<double>> edge_times(num_edges);

    std::cout << "\nComputing safety vectors for " << num_edges << " edges..." << std::endl;

    auto start_time = std::chrono::high_resolution_clock::now();

    // Compute safety vectors for all edges
    for (size_t i = 0; i < num_edges; ++i) {
        auto& edge = edges[i];

        // Bounds check for source vertex
        if (edge.source >= vertices.size()) {
            std::cerr << "Error: Edge " << i << " has invalid source vertex " << edge.source << std::endl;
            continue;
        }

        Point p1 = vertices[edge.source];
        Point p2 = vertices[edge.target];

        // Measure computation time for this edge
        auto edge_start = std::chrono::high_resolution_clock::now();
        auto safety_vec = compute_edge_safety_vector(p1, p2, obstacles);
        auto edge_end = std::chrono::high_resolution_clock::now();

        // Store computation time in microseconds
        edge.computation_time_us = std::chrono::duration_cast<std::chrono::microseconds>(
            edge_end - edge_start).count();

        // Store per-objective values
        for (size_t obj = 0; obj < num_objectives; ++obj) {
            all_values[obj][i] = {safety_vec[obj]};
        }

        // Store length and computation time (convert microseconds to milliseconds)
        lengths[i] = {edge.length};
        edge_times[i] = {edge.computation_time_us / 1000.0};

        if ((i + 1) % 1000 == 0) {
            std::cout << "  Processed " << (i + 1) << "/" << num_edges << " edges" << std::endl;
        }
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time).count();

    std::cout << "Computation completed in " << duration << " seconds" << std::endl;

    // Write .gr files for each obstacle (safety objectives)
    std::cout << "\nWriting .gr files..." << std::endl;
    for (size_t obj = 0; obj < num_objectives; ++obj) {
        std::string filename = output_prefix + "_" + std::to_string(obj) + ".gr";
        write_gr_file(filename, edges, all_values[obj], vertices.size());
    }

    // Write .gr file for length objective
    std::string length_file = output_prefix + "_" + std::to_string(num_objectives) + ".gr";
    write_gr_file(length_file, edges, lengths, vertices.size());

    // Write .gr file for edge computation times (optional, for --e_time parameter)
    std::string time_file = output_prefix + "_edge_times.gr";
    write_gr_file(time_file, edges, edge_times, vertices.size());

    std::cout << "\nPreprocessing complete!" << std::endl;
    std::cout << "Generated " << (num_objectives + 2) << " .gr files:" << std::endl;
    std::cout << "  - " << num_objectives << " safety objective files" << std::endl;
    std::cout << "  - 1 length objective file" << std::endl;
    std::cout << "  - 1 edge computation time file (for --e_time)" << std::endl;

    return 0;
}
