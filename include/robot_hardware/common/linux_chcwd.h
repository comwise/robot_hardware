#ifndef __COMMON__LINUX_CHCWD__H__
#define __COMMON__LINUX_CHCWD__H__

#include <string>
#include <exception>
#include <set>
#include <algorithm>
#include <string.h>
#include <iostream>

#if defined __linux__

#include <unistd.h>
#include <libgen.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <dirent.h>
//#include <sys/io.h>
// http://man7.org/linux/man-pages/man2/readlink.2.html
// http://man7.org/linux/man-pages/man3/dirname.3.html
// http://man7.org/linux/man-pages/man2/chdir.2.html

namespace common {

inline std::string get_exe_dir()
{
    char buf[256];
    int len = readlink("/proc/self/exe", buf, 256);
    if (len < 0 || len >= 256)
        throw std::runtime_error("Buffer size to get exe path is too small.");
    buf[len] = '\0';
    char *str = dirname(buf);
    return std::string(str);
}

inline std::string change_cwd()
{
    auto d = get_exe_dir();
    chdir(d.c_str());
    return d;
}

inline std::string get_file_name(const std::string &filename)
{
    std::string exe_name;
    auto npos = filename.rfind("/");
    if(npos >=0 && npos < filename.length()) {
        exe_name = filename.substr(npos+1);
    } else {
        exe_name = filename;
    }
    return std::move(exe_name);
}

inline std::string get_base_name(const std::string &url)
{
    std::string filename(url);
    auto pos = url.rfind("/");
    if (pos >= 0 && pos < url.length()) {
        filename = url.substr(pos + 1);
    }
    pos = filename.find(".");
    if (pos != std::string::npos) {
        filename = filename.substr(0, pos);
    }

    return std::move(filename);
}

inline std::string get_file_path(const std::string &file)
{
    std::string filename(file);
    auto pos = file.rfind("/");
    if (pos >= 0 && pos < file.length()) {
        filename = file.substr(0, pos);
    }
    return std::move(filename);
}

inline std::string get_absolute_path(const std::string &path)
{
    // '$' in the beginning of path means the exe dir
    // "$/abc.log" means the log file in the dir of the binary file
    // "./abc.log" means the log file in the real working dir
    if (path.length() > 1 && path[0] == '$') {
        return get_exe_dir() + path.substr(1);
    } else {
        return path;
    }
}

inline bool create_dir(const std::string &path, uint32_t mode = 0777)
{
    bool ret = true;
    int is_created = mkdir(path.c_str(), mode);
    if (is_created != 0) {
        printf("create path failed: %s\n", path.c_str());
        ret = false;
    }
    return ret;
}

inline bool is_file_exist(const std::string &path)
{
    return access(path.c_str(), F_OK) == 0;
}

inline bool is_dir_exist(const std::string &path)
{
#if 1
    auto handle = opendir(path.c_str());
    if(NULL == handle) {
        return false;
    }
    closedir(handle);
    return true;
#else
    struct stat my_stat;
    if (stat(path.c_str(), &my_stat) != 0)
        return false;
    return ((my_stat.st_mode & S_IFDIR) != 0);
#endif
}

inline bool get_dir_files(const std::string &cfg_dir, std::set<std::string> &files)
{
    bool ret = true;
    DIR *dir_obj = nullptr;
    try {
        if ((dir_obj = opendir(cfg_dir.c_str())) == nullptr) {
            std::cout << "open dir(" << cfg_dir << ") error" << std::endl;
            return false;
        }

        struct dirent *ptr = nullptr;
        while ((ptr = readdir(dir_obj)) != NULL) {
            if (strcmp(ptr->d_name, ".") == 0 ||
                strcmp(ptr->d_name, "..") == 0) {
                //current dir or parrent dir
                continue;
            } else {
                switch (ptr->d_type)
                {
                case 8:
                {
                    std::string url = cfg_dir + "/" + std::string(ptr->d_name);
#ifdef _DEBUG
                    std::cout << url << std::endl;
#endif
                    files.insert(url);
                    break;
                }
                case 10:
                {
#ifdef _DEBUG
                    std::cout << "file_name: " << cfg_dir << "/" << ptr->d_name << std::endl;
#endif
                    break;
                }
                case 4:
                {
                    std::string new_url = cfg_dir + "/" + std::string(ptr->d_name);
                    return get_dir_files(new_url, files);
                }
                default:
                    break;
                }
            }
        }
        ret = true;
    } catch(...) {
        std::cout << "open dir(" << cfg_dir << ") exception" << std::endl;
        ret = false;
    }
    if(dir_obj) {
        closedir(dir_obj);
    }
    return true;
}

} // namespace common

#endif

#endif //__COMMON__LINUX_CHCWD__H__