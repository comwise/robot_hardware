#include "etc/etc_launch.h"
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include "log/log.h"
#include "common/linux_chcwd.h"
#include "etc/etc_pool.h"


namespace comwise {
namespace etc {

etc_launch::etc_launch(const std::string &dir)
    : etc_cfg(dir)
{

}

etc_launch::~etc_launch()
{

}

bool etc_launch::load(std::string &error)
{
    bool ret = true;
    std::set<std::string> files;
    if(!common::get_dir_files(cfg_file_, files)) {
        error = "read launch dir(" + cfg_file_ + ") error";
        LOG_STREAM_ERROR << error;
        return false;
    }
    for (auto &file : files) {
        std::string xml;
        bool read_ret = read_file(file, xml);
        ret &= read_ret;
        if(read_ret) {
            std::string key = common::get_base_name(file);
            ETC_POOL->set_value(key, xml);
        } else {
            error += "read file(" + file + ") error \n";
        }
    }

    return ret;
}

bool etc_launch::read_file(const std::string &file, std::string &xml)
{
    bool ret = true;
    std::ifstream ifs;
    try {    
        ifs.open(file, std::ios::binary);
        if(ifs.is_open()) {
            std::stringstream buffer;
            buffer << ifs.rdbuf();
            xml = std::string(buffer.str());
            LOG_STREAM_INFO << "read file(" << file << ") success";
        } else {
            LOG_STREAM_ERROR << "read file(" << file << ") error";
            ret = false;
        }
        ifs.close();
    } catch (std::exception &e) {
        LOG_STREAM_ERROR << "read file(" << file << ") exception, " << e.what();
        ifs.close();
        ret = false;
    }
    return ret;
}

bool etc_launch::write_file(const std::string &file, const std::string &xml)
{
    return false;
}

} // namespace etc
} // namespace comwise
