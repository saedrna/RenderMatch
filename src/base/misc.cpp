/*!
 * \file misc.cpp
 *
 * \author Han
 * \date 2017/03/27
 *
 *
 */
#ifdef _WIN32
#include <Windows.h>
#endif

#include <base/filesystem.h>
#include <base/misc.h>

#include <algorithm>
#include <cstdarg>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <sstream>


namespace h2o {

std::string ensure_trailing_slash(const std::string &str) {
    if (str.length() > 0) {
        if (str.back() != '/') {
            return str + "/";
        }
    } else {
        return str + "/";
    }
    return str;
}

void split_file_extension(const std::string &path, std::string *root, std::string *ext) {
    const auto parts = string_split(path, ".");
    CHECK_GT(parts.size(), 0);
    if (parts.size() == 1) {
        *root = parts[0];
        *ext = "";
    } else {
        *root = "";
        for (size_t i = 0; i < parts.size() - 1; ++i) {
            *root += parts[i] + ".";
        }
        *root = root->substr(0, root->length() - 1);
        if (parts.back() == "") {
            *ext = "";
        } else {
            *ext = "." + parts.back();
        }
    }
}

void print_heading1(const std::string &heading) {
    std::cout << std::endl << std::string(78, '=') << std::endl;
    std::cout << heading << std::endl;
    std::cout << std::string(78, '=') << std::endl << std::endl;
}

void print_heading2(const std::string &heading) {
    std::cout << std::endl << heading << std::endl;
    std::cout << std::string(std::min<int>(heading.size(), 78), '-') << std::endl;
}

bool is_big_endian() {
    union {
        uint32_t i;
        char c[4];
    } bin = {0x01020304};
    return bin.c[0] == 1;
}

std::vector<std::string> read_text_file_lines(const std::string &path) {
    std::ifstream file(path.c_str());
    CHECK(file.is_open());

    std::string line;
    std::vector<std::string> lines;
    while (std::getline(file, line)) {
        string_trim(&line);

        if (line.empty()) {
            continue;
        }

        lines.push_back(line);
    }

    return lines;
}

#ifdef _WIN32
double get_system_memory() {
    MEMORYSTATUSEX memory_status;
    ZeroMemory(&memory_status, sizeof(MEMORYSTATUSEX));
    memory_status.dwLength = sizeof(MEMORYSTATUSEX);
    if (GlobalMemoryStatusEx(&memory_status)) {
        double bytes = memory_status.ullAvailPhys;
        return bytes / 1024.0 / 1024.0;
    } else {
        return 0.0;
    }
}
#endif // _WIN32

} // namespace h2o
