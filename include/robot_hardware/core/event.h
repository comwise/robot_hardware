/**
 * Copyright (c) 2023 comwise. All rights reserved.
 */
#ifndef __COMWISE__CORE_EVENT__H__
#define __COMWISE__CORE_EVENT__H__

#include <atomic>
#include <memory>
#include <mutex>
#include <map>
#include <vector>
#include <list>
#include <set>
#include <functional>
#include <future>
#include "common/any.h"

namespace core {

class event
{
public:
    using event_type_t = std::string;
    using event_vector_t = std::vector<event_type_t>;
    using event_handler_t = std::function<void(const common::any &)>;
    using handler_queue_t = std::vector<event_handler_t>;
    using handler_map_t = std::map<event_type_t, handler_queue_t>;

public:
    virtual ~event() { }

    void on(const event_type_t &type, event_handler_t handler)
    {
        if (handlers_.find(type) == handlers_.end()) {
            std::vector<event_handler_t> event_set;
            handlers_[type] = event_set;
        }
        handlers_[type].push_back(handler);
        listeners_.emplace_back(type);
    }

    void off(const event_type_t &type, event_handler_t handler = nullptr)
    {
        auto cit = handlers_.find(type);
        if (cit != handlers_.end()) {
            handlers_.erase(cit);
        }
        auto listener_cit = listeners_.begin();
        for ( ; listener_cit != listeners_.end(); listener_cit++) {
            if (*listener_cit == type) {
                listeners_.erase(listener_cit);
            }
        }
    }

    void emit(const event_type_t &type, const any &arg = {})
    {
        if (handlers_.find(type) == handlers_.end()) {
            return;
        }

        if (type.empty()) {
            return;
        }

        handler_queue_t handlers = handlers_[type];
        for ( auto &handler : handlers) {
            handler(arg);
        }
    }

    const event_vector_t& listener() const {
        return listeners_;
    }

private:
    handler_map_t handlers_;
    event_vector_t listeners_;
};

class event_core
{
public:
    using event_type_t = std::string;
    using event_handler_t = std::function<void()>;
    using handler_queue_t = std::vector<event_handler_t>;
    using handler_map_t = std::map<event_type_t, handler_queue_t>;

public:
    virtual ~event_core() { }

    template <class F, class... Args>
    auto on(const event_type_t &type, F &&f, Args &&...args)
        -> std::future<typename std::result_of<F(Args...)>::type>
    {
        using return_type = typename std::result_of<F(Args...)>::type;

        auto task = std::make_shared<std::packaged_task<return_type()>>(
            std::bind(std::forward<F>(f), std::forward<Args>(args)...));

        std::future<return_type> ret = task->get_future();
        {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            if (handlers_.find(type) == handlers_.end()) {
                std::vector<event_handler_t> event_set;
                handlers_[type] = event_set;
            }
            handlers_[type].emplace_back(
                [task]() { (*task)(); });
        }
        return ret;
    }

    template <class... Args>
    void emit(const event_type_t &type, Args &&...args)
    {
        if (type.empty() || handlers_.find(type) == handlers_.end()) {
            return;
        }

        handler_queue_t handlers = handlers_[type];
        for (auto &handler : handlers) {
            handler(std::ref(std::forward<Args>(args)...));
            //std::ref(handler, std::forward<Args>(args)...);
        }
    }

private:
    handler_map_t handlers_;
    std::mutex queue_mutex_;
};

} //namespace core

#endif  // __COMWISE__CORE_EVENT__H__
