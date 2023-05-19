#include "process/ros_ctrler.h"
#include <Poco/Process.h>
#include <Poco/PipeStream.h>
#include <Poco/UUIDGenerator.h>
#include <tinyxml.h>
#include <signal.h>
#include <fstream>
#include "log/log.h"
#include "common/make_thread.h"
#include "common/signal.hpp"
#include "ros/ros_node.h"

namespace comwise {
namespace process {

bool generate_xacro_xml(const std::string &xml_str, const std::map< std::string, std::string > &args,
                 std::vector< std::string > &fail_args, std::string &out_file_name)
{
    std::map< std::string, std::string > real_args;

    TiXmlDocument doc;
    doc.Parse(xml_str.c_str(), 0, TIXML_ENCODING_UTF8);
    auto root = doc.RootElement();
    for(auto c = root->FirstChild(); c != NULL; c = c->NextSibling())
    {
        if(c->Value() == std::string("xacro:property"))
        {
            auto name = c->ToElement()->Attribute("name");
            auto result = args.find(name);
            if(result != args.end()) {
                c->ToElement()->SetAttribute("value", result->second.c_str());
                real_args[result->first] = result->second;
            }
        }
    }

    TiXmlPrinter printer;
    doc.Accept(&printer);

    auto out_xml = printer.CStr();

    bool rtn = true;
    for(auto k = args.begin(); k != args.end(); k++)
    {
        if(real_args.find(k->first) == real_args.end()) {
            rtn = false;
            fail_args.push_back(k->first);
        }
    }

    if(rtn) {
        out_file_name = std::string("/tmp/") + Poco::UUIDGenerator().createOne().toString() + ".urdf.xacro";
        std::ofstream f(out_file_name);
        if (f.good()) {
            f << out_xml;
            f.close();
        }
        else {
            LOG_STREAM_ERROR << "Tmp xacro file store fails...";
            rtn = false;
        }
    }

    return rtn;
}

ros_ctrler::ros_ctrler(const std::string &name, const std::string &args)
    : process_ctrler(name, args)
{

}

ros_ctrler::~ros_ctrler()
{
    wait_for_end();
}

bool ros_ctrler::start()
{
    Poco::Process::Args args;
    args.push_back("-oL");
    args.push_back("-eL");
    args.push_back("roslaunch");
    args.push_back("-");
    args.push_back(std::string("tag:=") + name_);  // make subprocess command have a humanable tag which can be seen by ps, pgrep, etc.
    args.push_back("--wait");                      // roslaunch needn't start roscore when the core has some problem, just wait

    Poco::Pipe in_pipe, out_pipe, err_pipe;
    Poco::PipeOutputStream ostr(in_pipe);

    LOG_STREAM_INFO << "[ros:" << name_ << "] start to launch sub process ...";

    ph_ = std::make_shared<process_handle_t>(
        Poco::Process::launch("stdbuf", args, &in_pipe, &out_pipe, &err_pipe));

    out_s_ = std::make_shared<pipe_input_stream_t>(out_pipe);
    err_s_ = std::make_shared<pipe_input_stream_t>(err_pipe);

    ostr << arg_;
    ostr.close();

    std::signal(SIG_WAKE_SYS_CALL_BLOCK, [](int sig){});

    is_loop_ = true;

    out_thr_ = std::move(common::make_thread(std::string("Rinf_") + name_, [&, this](){
        while(is_loop_ && out_s_ && out_s_->good()) {
            std::string info;
            if(std::getline(*out_s_, info)) {
                LOG_STREAM_INFO << "[ros:" << name_ << "] "<< info;
            }
        }
    }));

    err_thr_ = std::move(common::make_thread(std::string("Rerr_") + name_, [&, this](){
        while(is_loop_ && err_s_ && err_s_->good()) {
            std::string info;
            if(std::getline(*err_s_, info)) {
                LOG_STREAM_ERROR << "[ros:" << name_ << "] "<< info;
            }
        }
    }));

    return this->is_running();
}

bool ros_ctrler::stop()
{
    try {
        if(ph_ && is_running())
            Poco::Process::requestTermination(ph_->id());
    } catch (const Poco::NotFoundException &e) {
        LOG_STREAM_WARN << "requestTermination subprocess (" << name_ << ") exception, " << e.what();
    }
    return true;
}

bool ros_ctrler::is_running()
{
    if(ph_) {
        return ::kill(ph_->id(), 0) == 0;
    }
    return false;
}

int ros_ctrler::wait_for_end()
{
    if(ph_ == nullptr)
        return 0;

    int rtn_code = 0;
    if(is_running()) {
        rtn_code = Poco::Process::wait(*ph_);
    }

    is_loop_ = false;

    if(out_thr_) {
        pthread_kill(out_thr_->native_handle(), SIG_WAKE_SYS_CALL_BLOCK);   // wake the possible block of read() in getline()
    }
    if(out_s_) {
        out_s_->close();
    }
    out_s_ = nullptr;

    if(err_thr_) {
        pthread_kill(err_thr_->native_handle(), SIG_WAKE_SYS_CALL_BLOCK);   // wake the possible block of read() in getline()
    }
    if(err_s_) {
        err_s_->close();
    }
    err_s_ = nullptr;

    if(out_thr_ && out_thr_->joinable()) {
        out_thr_->join();
    }
    out_thr_ = nullptr;

    if(err_thr_ && err_thr_->joinable()) {
        err_thr_->join();
    }
    err_thr_ = nullptr;

    ph_ = nullptr;

    return rtn_code;
}

bool ros_ctrler::set_args(const std::map<std::string, std::string> &args, std::vector<std::string> &fail_args)
{
    std::map<std::string, std::string> real_args;

    TiXmlDocument doc;
    doc.Parse(arg_.c_str(), 0, TIXML_ENCODING_UTF8);
    auto root = doc.RootElement();
    if(nullptr == root) {
        std::string err = "xml root element is null";
        fail_args.push_back(err);
        LOG_STREAM_ERROR << err << "," << arg_.c_str();
        return false;
    }

    for(auto c = root->FirstChild(); c != NULL; c = c->NextSibling())
    {
        if(c->Value() == std::string("arg"))
        {
            auto name = c->ToElement()->Attribute("name");
            auto result = args.find(name);
            if(result != args.end()) {
                c->ToElement()->SetAttribute("value", result->second.c_str());
                real_args[result->first] = result->second;
            }
        }
    }

    TiXmlPrinter printer;
    doc.Accept(&printer);

    arg_ = printer.CStr();

    bool rtn = true;
#ifdef XML_ARGS_MATCH // temporarily disable, because (args num) > (xml args num)
    for(auto k = args.begin(); k != args.end(); k++)
    {
        if(real_args.find(k->first) == real_args.end()) {
            rtn = false;
            fail_args.push_back(k->first);
        }
    }
#endif
    return rtn;
}

}
} // 
