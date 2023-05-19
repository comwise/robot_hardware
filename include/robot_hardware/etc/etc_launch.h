#ifndef __COMWISE_ETC__ETC_LAUNCH__H__
#define __COMWISE_ETC__ETC_LAUNCH__H__

#include <string>
#include "etc_cfg.h"

namespace comwise {
namespace etc {

class etc_launch : public etc_cfg
{
public:
    explicit etc_launch(const std::string &dir);
    virtual ~etc_launch();

    virtual bool load(std::string &error) override;

private:
    bool read_file(const std::string &file, std::string &xml);
    bool write_file(const std::string &file, const std::string &xml);
};

} // namespace etc
} // namespace comwise

#endif // __COMWISE_ETC__ETC_LAUNCH__H__
