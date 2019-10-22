/*!
 * \file filesystem.cpp
 *
 * \author Han
 * \date 2017/03/27
 *
 *
 */
#include <base/filesystem.h>
#include <base/string.h>
#include <glog/logging.h>

#include <QDir>
#include <QDirIterator>
#include <QFile>
#include <QFileInfo>

namespace h2o {
bool file_exist(const std::string &path) {
    bool result = QFile::exists(QString::fromStdString(path));
    return result;
}

std::string get_extension_name(const std::string &path) {
    QString qpath = QString::fromStdString(path);
    QFileInfo info(qpath);
    QString qextension = info.suffix();
    std::string extension = qextension.toStdString();
    extension = "." + extension;
    return extension;
}

std::string get_directory(const std::string &path) {
    QFileInfo file(QString::fromStdString(path));
    QString dir = file.absoluteDir().absolutePath();
    return dir.toStdString();
}

std::string get_filename(const std::string &path) {
    return QFileInfo(QString::fromStdString(path)).fileName().toStdString();
}

std::string get_filename_noext(const std::string &path) {
    return QFileInfo(QString::fromStdString(path)).baseName().toStdString();
}

bool has_file_extension(const std::string &file_name, const std::string &ext) {
    CHECK(!ext.empty());
    CHECK_EQ(ext.at(0), '.');
    std::string ext_lower = string_to_lower(ext);
    ;
    if (file_name.size() >= ext_lower.size() &&
        file_name.substr(file_name.size() - ext_lower.size(), ext_lower.size()) == ext_lower) {
        return true;
    }
    return false;
}
bool create_directory(const std::string &path) { return QDir().mkdir(QString::fromStdString(path)); }

void create_dir_if_not_exist(const std::string &path) { create_directory(path); }

std::string get_path_base_name(const std::string &path) {
    const std::vector<std::string> names = string_split(string_replace(path, "\\", "/"), "/");
    if (names.size() > 1 && names.back() == "") {
        return names[names.size() - 2];
    } else {
        return names.back();
    }
}

std::vector<std::string> get_recursive_file_list(const std::string &path) {
    std::vector<std::string> file_list;
    QDirIterator iterator(QDir(QString::fromStdString(path)), QDirIterator::Subdirectories);
    while (iterator.hasNext()) {
        QString filename = iterator.next();
        if (QFileInfo(filename).isFile()) {
            file_list.push_back(filename.toStdString());
        }
    }

    return file_list;
}

bool delete_file(const std::string &path) {
    bool is_directory = QFileInfo(QString::fromStdString(path)).isDir();
    if (is_directory) {
        return QDir(QString::fromStdString(path)).removeRecursively();
    } else {
        return QFile::remove(QString::fromStdString(path));
    }

    return true;
}

} // namespace h2o
