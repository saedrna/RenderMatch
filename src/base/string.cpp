/*!
 * \file string.cpp
 *
 * \author Han
 * \date 2017/03/27
 *
 *
 */
#include <base/string.h>

#include <cstdarg>
#include <cstdio>
#include <fstream>
#include <memory>
#include <sstream>

#include <QRegExp>
#include <QString>
#include <QStringList>
#include <QTextCodec>

#ifdef _MSC_VER
#include <filesystem>
#endif

namespace h2o {
namespace {

void StringAppendV(std::string *dst, const char *format, va_list ap) {
    // First try with a small fixed size buffer.
    static const int kFixedBufferSize = 1024;
    char fixed_buffer[kFixedBufferSize];

    // It is possible for methods that use a va_list to invalidate
    // the data in it upon use.  The fix is to make a copy
    // of the structure before using it and use that copy instead.
    va_list backup_ap;
    va_copy(backup_ap, ap);
    int result = vsnprintf(fixed_buffer, kFixedBufferSize, format, backup_ap);
    va_end(backup_ap);

    if (result < kFixedBufferSize) {
        if (result >= 0) {
            // Normal case - everything fits.
            dst->append(fixed_buffer, result);
            return;
        }

#ifdef _MSC_VER
        // Error or MSVC running out of space.  MSVC 8.0 and higher
        // can be asked about space needed with the special idiom below:
        va_copy(backup_ap, ap);
        result = vsnprintf(nullptr, 0, format, backup_ap);
        va_end(backup_ap);
#endif

        if (result < 0) {
            // Just an error.
            return;
        }
    }

    // Increase the buffer size to the size requested by vsnprintf,
    // plus one for the closing \0.
    const int variable_buffer_size = result + 1;
    std::unique_ptr<char> variable_buffer(new char[variable_buffer_size]);

    // Restore the va_list before we use it again.
    va_copy(backup_ap, ap);
    result = vsnprintf(variable_buffer.get(), variable_buffer_size, format, backup_ap);
    va_end(backup_ap);

    if (result >= 0 && result < variable_buffer_size) {
        dst->append(variable_buffer.get(), result);
    }
}

bool IsNotWhiteSpace(const int character) {
    return character != ' ' && character != '\n' && character != '\r' && character != '\t';
}

} // namespace

std::string string_printf(const char *format, ...) {
    va_list ap;
    va_start(ap, format);
    std::string result;
    StringAppendV(&result, format, ap);
    va_end(ap);
    return result;
}

std::string string_replace(const std::string &str, const std::string &old_str, const std::string &new_str) {
    if (old_str.empty()) {
        return str;
    }
    size_t position = 0;
    std::string mod_str = str;
    while ((position = mod_str.find(old_str, position)) != std::string::npos) {
        mod_str.replace(position, old_str.size(), new_str);
        position += new_str.size();
    }
    return mod_str;
}

std::vector<std::string> string_split(const std::string &str, const std::string &delim) {

    QString pattern;
    bool first = true;
    for (char c : delim) {
        if (!first) {
            pattern += QString("|");
        }
        first = false;
        pattern += QString("\\%1").arg(QString(c));
    }
    // QRegExp rx("(\\ |\\,|\\.|\\:|\\t)"); // RegEx for ' ' or ',' or '.' or ':' or '\t'
    QRegExp rx(pattern);
    QStringList query = QString::fromStdString(str).split(rx);

    std::vector<std::string> elems;
    for (const auto &s : query) {
        elems.push_back(s.toStdString());
    }

    return elems;
}

bool string_start_with(const std::string &str, const std::string &prefix) {
    return !prefix.empty() && prefix.size() <= str.size() && str.substr(0, prefix.size()) == prefix;
}

void string_left_trim(std::string *str) {
    str->erase(str->begin(), std::find_if(str->begin(), str->end(), IsNotWhiteSpace));
}

void string_right_trim(std::string *str) {
    str->erase(std::find_if(str->rbegin(), str->rend(), IsNotWhiteSpace).base(), str->end());
}

std::string string_to_lower(const std::string &str) { return QString::fromStdString(str).toLower().toStdString(); }

std::string string_utf8_to_fstream(const std::string &filepath) {
#ifdef _MSC_VER
    // MSVC 需要转换
    std::filesystem::path path = std::filesystem::path(QString::fromStdString(filepath).toStdWString());
    std::string result = path.string();

    return result;
#else
    // TODO: 测试其他平台是不是不需要转换
    return filepath;
#endif // _MSC_VER
}

void string_trim(std::string *str) {
    string_left_trim(str);
    string_right_trim(str);
}

} // namespace h2o
