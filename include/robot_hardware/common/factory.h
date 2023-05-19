/**
 * Copyright (c) 2023 comwise. All rights reserved.
 */
#ifndef __COMWISE_COMMON__FACTORY__H__
#define __COMWISE_COMMON__FACTORY__H__

#include <memory>
#include <map>

namespace common {

template <typename IdentifierType, class AbstractProduct, 
          class ProductCreator = AbstractProduct *(*)(),
          class MapContainer = std::map<IdentifierType, ProductCreator>>
class Factory
{
public:
  bool Register(const IdentifierType &id, ProductCreator creator) {
    return producers_.insert(std::make_pair(id, creator)).second;
  }

  bool Unregister(const IdentifierType &id) {
    return producers_.erase(id) == 1;
  }

  void Clear() { producers_.clear(); }

  bool Contains(const IdentifierType &id) {
    return producers_.find(id) != producers_.end();
  }

  bool Empty() const { return producers_.empty(); }

  template <typename... Args>
  std::unique_ptr<AbstractProduct> CreateObjectOrNull(const IdentifierType &id,
                                                      Args &&... args) {
    auto id_iter = producers_.find(id);
    if (id_iter != producers_.end()) {
      return std::unique_ptr<AbstractProduct>(
          (id_iter->second)(std::forward<Args>(args)...));
    }
    return nullptr;
  }

  template <typename... Args>
  std::unique_ptr<AbstractProduct> CreateObject(const IdentifierType &id,
                                                Args &&... args) {
    auto result = CreateObjectOrNull(id, std::forward<Args>(args)...);
    if (!result) {
      printf("Factory could not create Object of type");
    }
    return result;
  }

private:
  MapContainer producers_;
};

}  // namespace common

#endif  // __COMWISE_COMMON__FACTORY__H__
