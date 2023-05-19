#ifndef __COMWISE_SERIAL__SERIAL_FACTORY__H__
#define __COMWISE_SERIAL__SERIAL_FACTORY__H__

#include <cstdint>
#include <memory>
#include <map>
#include "common/singleton.h"

namespace comwise {
namespace serial {

template<typename SerialType>
class serial_factory
{
public:
  using serial_ptr_t = std::shared_ptr<SerialType>;
  using serial_id_t = std::map<std::string, serial_ptr_t>;
  using serial_index_t = std::map<uint16_t, serial_ptr_t>;

public:
  std::shared_ptr<SerialType> create(const std::string &id, uint16_t index = 0) {
    return serial_id_[id] = std::make_shared<SerialType>(id, type);
  }

  std::shared_ptr<SerialType> get_serial(const std::string &id){
    auto cit = serial_id_.find(id);
    if(cit != serial_id_.end()) {
      return cit->second;
    }
    return nullptr;
  }
  
  std::shared_ptr<SerialType> get_serial(uint16_t index){
    auto cit = serial_index_.find(index);
    if(cit != serial_index_.end()) {
      return cit->second;
    }
    return nullptr;
  }

private:
  serial_id_t serial_id_;
  serial_index_t serial_index_;
};

} // namespace serial
} // namespace comwise

#endif // __COMWISE_SERIAL__SERIAL_FACTORY__H__