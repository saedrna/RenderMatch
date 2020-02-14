/*!
 * \file misc.h
 *
 * \author Han
 * \date 2017/03/27
 *
 *
 */
#pragma once
#include <base/common.h>

#include <base/filesystem.h>
#include <base/string.h>

#include <fstream>

#include <QVariant>

namespace h2o {

#ifndef STRINGIFY
#define STRINGIFY(s) STRINGIFY_(s)
#define STRINGIFY_(s) #s
#endif // STRINGIFY

// Append trailing slash to string if it does not yet end with a slash.
std::string ensure_trailing_slash(const std::string &str);

// Split the path into its root and extension, for example,
// "dir/file.jpg" into "dir/file" and ".jpg".
void split_file_extension(const std::string &path, std::string *root, std::string *ext);

// Print first-order heading with over- and underscores to `std::cout`.
void print_heading1(const std::string &heading);

// Print second-order heading with underscores to `std::cout`.
void print_heading2(const std::string &heading);

// Check if vector contains elements.
template <typename T> bool vector_contains_value(const std::vector<T> &vector, const T value);

template <typename T> bool vector_contains_duplicate_values(const std::vector<T> &vector);

// Parse CSV line to a list of values.
template <typename T> std::vector<T> csv_to_vector(const std::string &csv);

// Concatenate values in list to comma-separated list.
template <typename T> std::string vector_to_csv(const std::vector<T> &values);

// Check the order in which bytes are stored in computer memory.
bool is_big_endian();

// Read contiguous binary blob from file.
template <typename T> void read_binary_blob_from_file(const std::string &path, std::vector<T> *data);

// Write contiguous binary blob to file.
template <typename T> void write_binary_blob(const std::string &path, const std::vector<T> &data);

// Read each line of a text file into a separate element. Empty lines are
// ignored and leading/trailing whitespace is removed.
std::vector<std::string> read_text_file_lines(const std::string &path);

#ifdef _WIN32
// return the available memory in MB
double get_system_memory();
#endif // _WIN32

////////////////////////////////////////////////////////////////////////////////
// Implementation
////////////////////////////////////////////////////////////////////////////////

template <typename T> bool vector_contains_value(const std::vector<T> &vector, const T value) {
    return std::find_if(vector.begin(), vector.end(), [value](const T element) { return element == value; }) !=
           vector.end();
}

template <typename T> bool vector_contains_duplicate_values(const std::vector<T> &vector) {
    std::vector<T> unique_vector = vector;
    return std::unique(unique_vector.begin(), unique_vector.end()) != unique_vector.end();
}

template <typename T> std::vector<T> csv_to_vector(const std::string &csv) {
    auto elems = string_split(csv, ",;");
    std::vector<T> values;
    values.reserve(elems.size());
    for (auto &elem : elems) {
        string_trim(&elem);
        if (elem.empty()) {
            continue;
        }
        try {
            values.push_back(qvariant_cast<T>(QString::fromStdString(elem)));
            // values.push_back(boost::lexical_cast<T>(elem));
        } catch (std::exception) {
            return std::vector<T>(0);
        }
    }
    return values;
}

template <typename T> std::string vector_to_csv(const std::vector<T> &values) {
    std::string string;
    for (const T value : values) {
        string += std::to_string(value) + ", ";
    }
    return string.substr(0, string.length() - 2);
}

template <typename T> void ReadBinaryBlob(const std::string &path, std::vector<T> *data) {
    std::ifstream file(path, std::ios_base::binary | std::ios::ate);
    CHECK(file.is_open()) << path;
    file.seekg(0, std::ios::end);
    const size_t num_bytes = file.tellg();
    CHECK_EQ(num_bytes % sizeof(T), 0);
    data->resize(num_bytes / sizeof(T));
    file.seekg(0, std::ios::beg);
    file.read(reinterpret_cast<char *>(data->data()), num_bytes);
}

template <typename T> void write_binary_blob(const std::string &path, const std::vector<T> &data) {
    std::ofstream file(path, std::ios_base::binary);
    CHECK(file.is_open()) << path;
    file.write(reinterpret_cast<const char *>(data.data()), data.size() * sizeof(T));
}

} // namespace h2o
