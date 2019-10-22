/*
 * @Author: Han Hu
 * @Date: 2019-01-22 14:06:00
 * 读取json文件
 */
#pragma once

#include <nlohmann/json.hpp>

#include <QFile>
#include <QTextStream>
#include <sstream>
#include <string>

namespace h2o {
inline nlohmann::json read_json_file(const std::string &filename) {
    QFile file(QString::fromStdString(filename));
    file.open(QFile::ReadOnly);
    std::string text = file.readAll().toStdString();
    std::istringstream ss(text);

    nlohmann::json j;
    ss >> j;
    return j;
}
} // namespace h2o
