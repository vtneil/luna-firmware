#ifndef FILE_UTILITY_H
#define FILE_UTILITY_H

#include <Arduino.h>
#include <Arduino_Extended.h>
#include <SdFat.h>

enum class FsMode : uint8_t {
    READ = 0,
    WRITE,
    APPEND
};

template<typename SdClass = SdExFat, typename FileClass = ExFile>
class FsUtil {
protected:
    SdClass &m_sd;

    struct ls_helper {
        SdClass &m_sd;
        const char *m_path;

        constexpr explicit ls_helper(SdClass &sd, const char *path) : m_sd{sd}, m_path{path} {}

        ls_helper &operator>>(Stream &stream) {
            FileClass root = m_sd.open(m_path);
            FileClass entry;
            while ((entry = root.openNextFile())) {
                char tmp[256];
                entry.getName(tmp, sizeof(tmp));
                if (entry.isFile()) {
                    stream.print("File: ");
                    stream.print(tmp);
                    stream.print("\tSize = ");
                    stream.print(entry.size());
                    stream.println(" bytes");
                } else if (entry.isDir()) {
                    stream.print("Dir:  ");
                    stream.println(tmp);
                }
                entry.close();
            }
            return *this;
        }
    };

public:
    explicit FsUtil(SdClass &sd) : m_sd(sd) {}

    [[nodiscard]] ls_helper ls(const String &path) const {
        return ls_helper(m_sd, path.c_str());
    }

    [[nodiscard]] ls_helper ls(const char *path = "/") const {
        return ls_helper(m_sd, path);
    }

    void rm(const char *path) const {
    }

    template<FsMode Mode>
    [[nodiscard]] FileClass open(const String &filename) const {
        return open<Mode>(filename.c_str());
    }

    template<FsMode Mode>
    [[nodiscard]] FileClass open(const char *filename) const {
        FileClass file;

        if constexpr (Mode == FsMode::READ) {
            file = m_sd.open(filename, FILE_READ);
        } else if constexpr (Mode == FsMode::WRITE) {
            file = m_sd.open(filename, FILE_WRITE);
        } else if constexpr (Mode == FsMode::APPEND) {
            file = m_sd.open(filename, FILE_WRITE);
            file.seekEnd();
        }

        return file;
    }
};

#endif  //FILE_UTILITY_H
