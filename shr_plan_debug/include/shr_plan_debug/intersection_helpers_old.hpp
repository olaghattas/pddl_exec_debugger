//
// Created by olagh48652 on 2/8/25.
//

#ifndef PLAN_SOLVER_INTERSECTION_HELPERS_HPP
#define PLAN_SOLVER_INTERSECTION_HELPERS_HPP


#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <unordered_set>
#include "pddl_parser/pddl_parser.hpp"
// check how ot ddd  #include "pddl_parser/pddl_parser.hpp" to sue hte below wothout adding them #include "bt_shr_actions.hpp"
struct Parameter {
    std::string name;
    std::string type;

    bool operator==(const Parameter &other) const {
        return name == other.name && type == other.type;
    }
};

struct InstantiatedParameter {
    std::string name;
    std::string type;

    bool operator==(const InstantiatedParameter &other) const {
        return name == other.name && type == other.type;
    }
};

struct Predicate {
    std::string name;
    std::vector<Parameter> parameters;

    bool operator==(const Predicate &other) const {
        return name == other.name && parameters == other.parameters;
    }
};

struct InstantiatedPredicate {
    std::string name;
    std::vector<InstantiatedParameter> parameters;

    bool operator==(const InstantiatedPredicate &other) const {
        return name == other.name && parameters == other.parameters;
    }
};

std::vector<std::pair<std::string, std::string>> read_predicates_from_file(const std::string &outputFile){
    std::ifstream file(outputFile);
    if (!file.is_open()) {
        std::cerr << "Error opening file!" << std::endl;
        return {};
    }

    std::vector<std::pair<std::string, std::string>> predicates;

    std::string line;
    while (std::getline(file, line)) {
        if (line.empty()) continue;  // Skip empty lines

        // Remove surrounding parentheses safely
        if (line.front() == '(' && line.back() == ')' && line.size() > 2) {
            line = line.substr(1, line.size() - 2);
        }

        std::istringstream iss(line);
        std::string first, second;
        if (iss >> first >> second) {  // Ensure two words are extracted
            predicates.emplace_back(first, second);
        }
    }

    file.close();
    return predicates;
}

std::unordered_set<std::string> readKeywords(const std::string &keywords_filename) {
    std::unordered_set<std::string> keywords;
    std::ifstream file(keywords_filename);
    if (!file.is_open()) {
        std::cerr << "Error opening keywords file!" << std::endl;
        return keywords;
    }

    std::string line;
    while (std::getline(file, line)) {
        if (!line.empty()) {
            keywords.insert(line);
        }
    }
    file.close();
    return keywords;
}

std::string extractInitBlock(const std::string &content) {
    size_t start = content.find("(:init");
    if (start == std::string::npos) {
        std::cerr << "No init block found!" << std::endl;
        return "";
    }

    std::string initBlock;
    int balance = 0;
    for (size_t i = start; i < content.size(); i++) {
        initBlock += content[i];
        if (content[i] == '(') balance++;
        if (content[i] == ')') balance--;

        if (balance == 0) break; // Found matching closing parenthesis
    }

    if (balance != 0) {
        std::cerr << "Unmatched parentheses in init block!" << std::endl;
        return "";
    }

    return initBlock;
}

std::vector<std::string> extractMatchingPredicates(const std::string &initBlock, const std::unordered_set<std::string> &keywords) {
    std::vector<std::string> predicates;
    std::istringstream stream(initBlock);
    std::string token;

    while (std::getline(stream, token, '(')) { // Split by '('
        size_t end = token.find(')');
        if (end != std::string::npos) {
            std::string predicate = token.substr(0, end);
            std::istringstream predStream(predicate);
            std::string firstWord;
            predStream >> firstWord; // Extract first word

            if (keywords.find(firstWord) != keywords.end()) {
                predicates.push_back("(" + predicate + ")"); // Restore parentheses
            }
        }
    }

    return predicates;
}

// Function to trim leading and trailing spaces
std::string trim(const std::string &str) {
    size_t start = str.find_first_not_of(" \t\n\r");
    size_t end = str.find_last_not_of(" \t\n\r");
    return (start == std::string::npos || end == std::string::npos) ? "" : str.substr(start, end - start + 1);
}

// Function to write unique predicates to a file
void writeToFile(const std::string &filename, const std::vector<std::string> &predicates) {
    std::unordered_set<std::string> existingPredicates;

    // Read existing content from the file and add to the set
    std::ifstream file(filename);
    std::string line;
    while (std::getline(file, line)) {
        existingPredicates.insert(trim(line));  // Trim and insert each line from the file
    }
    file.close();

    // Open the file again in append mode
    std::ofstream outFile(filename, std::ios::app);
    if (!outFile.is_open()) {
        std::cerr << "Error opening output file!" << std::endl;
        return;
    }

    // Only append unique predicates (after trimming)
    for (const auto &predicate : predicates) {
        std::string trimmedPredicate = trim(predicate);  // Trim the current predicate
        if (existingPredicates.find(trimmedPredicate) == existingPredicates.end()) {
            outFile << predicate << "\n";  // Append if it's not already in the set
            existingPredicates.insert(trimmedPredicate);  // Mark this predicate as written
        }
    }

    outFile.close();
}

int write_from_problem_file(const std::string &problemFile,
                            const std::string &keywordsFile,
                            const std::string &outputFile){

    // Read keywords
    std::unordered_set<std::string> keywords = readKeywords(keywordsFile);
    if (keywords.empty()) {
        std::cerr << "NO keywords in file!" << std::endl;
        return 0;
    }
    // Read problem file
    std::ifstream file(problemFile);
    if (!file.is_open()) {
        std::cerr << "Error opening problem file!" << std::endl;
        return 0;
    }

    std::stringstream buffer;
    buffer << file.rdbuf();
    std::string content = buffer.str();
    file.close();

    // Extract (:init ...) block
    std::string initBlock = extractInitBlock(content);
    if (initBlock.empty()) return 0;

    // Extract matching predicates
    std::vector<std::string> matchingPredicates = extractMatchingPredicates(initBlock, keywords);

    // Write intersection to output file
    writeToFile(outputFile, matchingPredicates);

    std::cout << "Intersection saved to " << outputFile << std::endl;
    return 1;
}

//void write_to_intersection(const std::vector<InstantiatedPredicate>& predicates,
//                             const std::filesystem::path& outputFile) {
//    std::ofstream ofs(outputFile);
//    if (!ofs) {
//        std::cerr << "Failed to open output file: " << outputFile << std::endl;
//        return;
//    }
//
//    // Write each predicate and its associated parameters
//    for (const auto& pred : predicates) {
//        ofs << "Keyword: " << pred.keyword << "\n";
//        for (const auto& param : pred.parameters) {
//            ofs << "\tProtocol Name: " << param.protocolName
//                << ", Protocol Type: " << param.protocolType << "\n";
//        }
//        ofs << "\n";
//    }
//    ofs.close();
//    std::cout << "Predicates written to " << outputFile << std::endl;
//}

void write_to_intersection(const std::string& filePath,
                           const std::vector<InstantiatedPredicate>& predicates) {
    std::ofstream ofs(filePath);
    if (!ofs) {
        std::cerr << "Failed to open output file: " << filePath << std::endl;
        return;
    }
    // For each predicate, write each protocol parameter on its own line.
    for (const auto& pred : predicates) {
        for (const auto& param : pred.parameters) {
            ofs << pred.keyword << " "
                << param.protocolName << " "
                << param.protocolType << "\n";
        }
    }
    ofs.close();
    std::cout << "Predicates successfully written to " << filePath << std::endl;
}

// --- Function: read_from_intersection ---
// Reads from the file at the given file path and prints each line.
void read_from_intersection(const std::string& filePath) {
    std::ifstream ifs(filePath);
    if (!ifs) {
        std::cerr << "Failed to open file for reading: " << filePath << std::endl;
        return;
    }
    std::cout << "Contents of " << filePath << ":\n";
    std::string line;
    while (std::getline(ifs, line)) {
        std::cout << line << "\n";
    }
    ifs.close();
}

#endif //PLAN_SOLVER_INTERSECTION_HELPERS_HPP
