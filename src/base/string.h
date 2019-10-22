/*!
 * \file string.h
 *
 * \author Han
 * \date 2017/03/27
 *
 *
 */
#pragma once

#include <string>
#include <vector>

namespace h2o {

// Format string by replacing embedded format specifiers with their respective
// values, see `printf` for more details. This is a modified implementation
// of Google's BSD-licensed StringPrintf function.
std::string string_printf(const char *format, ...);

// Replace all occurrences of `old_str` with `new_str` in the given string.
std::string string_replace(const std::string &str, const std::string &old_str, const std::string &new_str);

// Split string into list of words using the given delimiters.
std::vector<std::string> string_split(const std::string &str, const std::string &delim);

// Check whether a string starts with a certain prefix.
bool string_start_with(const std::string &str, const std::string &prefix);

// Remove whitespace from string on both, left, or right sides.
void string_trim(std::string *str);
void string_left_trim(std::string *str);
void string_right_trim(std::string *str);
std::string string_to_lower(const std::string &str);
///\brief 将一个utf-8转换为各个平台 fstream 都可以接受的路径
std::string string_utf8_to_fstream(const std::string &filepath);
} // namespace h2o
