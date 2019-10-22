/*!
 * \file filesystem.h
 *
 * \author Han
 * \date 2017/03/27
 *
 *
 */
#pragma once
#include <QDir>

namespace h2o {

///\brief 文件是否存在,utf路径
bool file_exist(const std::string &path);

///\brief 获取文件名后缀，如".txt"
std::string get_extension_name(const std::string &path);

std::string get_directory(const std::string &path);

///\brief "C:/file.txt" -> "file.txt"
std::string get_filename(const std::string &path);

///\brief "C:/file.txt" -> "file"
std::string get_filename_noext(const std::string &path);

// Check whether file name has the file extension (case insensitive).
bool has_file_extension(const std::string &file_name, const std::string &ext);

bool create_directory(const std::string &path);
// Join multiple paths into one path.
template <typename... T> std::string join_paths(T const &... paths);

// Create the directory if it does not exist.
void create_dir_if_not_exist(const std::string &path);

// Extract the base name of a path, e.g., "image.jpg" for "/dir/image.jpg".
std::string get_path_base_name(const std::string &path);

// Return list of files, recursively in all sub-directories.
std::vector<std::string> get_recursive_file_list(const std::string &path);

bool delete_file(const std::string &path);

/* Implementation */
template <typename... T> std::string join_paths(T const &... paths) {
    QString result;
    int unpack[]{0, (result = QDir(result).filePath(QString::fromStdString(std::string(paths))), 0)...};
    static_cast<void>(unpack);
    return result.toStdString();
}
} // namespace h2o