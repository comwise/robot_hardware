#ifndef __COMMON_ANY__H__
#define __COMMON_ANY__H__

#include <cstddef>
#include <memory>
#include <string>
#include <string>
#include <vector>
#include <map>
#include <typeinfo>
#include <initializer_list>


namespace common {

    template <bool B, class T = void>
    struct disable_if_c
    {
        typedef T type;
    };

    template <class T>
    struct disable_if_c<true, T>
    {
    };

    template <class Cond, class T = void>
    struct disable_if : public disable_if_c<Cond::value, T>
    {
    };

    struct holder
    {

        virtual std::shared_ptr<holder> clone() const = 0;
        virtual std::string type_name() const = 0;
        virtual std::size_t value_size() const = 0;
        virtual ~holder() {}
    };

    template <typename T>
    struct place_holder : public holder
    {
        using value_type = typename std::remove_reference<T>::type;
        using reference_type = value_type &;
        using const_reference_type = const value_type &;
        using pointer_type = value_type *;

        template <typename... Args>
        place_holder(Args &&...args) : value(std::forward<Args>(args)...) {}

        place_holder(const place_holder &other) : value(other.value) {}
        place_holder(place_holder &&other) : value(std::move(other.value)) {}

        std::string type_name() const override
        {
            return typeid(value_type).name();
        }

        std::shared_ptr<holder> clone() const override
        {
            return std::make_shared<place_holder>(value);
        }

        std::size_t value_size() const override
        {
            return sizeof(value);
        }

        value_type value;
    };

    class any
    {
    public:
        using array_t = std::vector<any>;
        using object_t = std::map<std::string, any>;

        constexpr any() noexcept {}

        any(const char *str) : _holder(std::make_shared<place_holder<std::string>>(str))
        {
        }

        any(const any &other) : _holder(other._holder)
        {
        }

        any(any &&other) noexcept : _holder(std::move(other._holder))
        {
        }

        template <typename T,
                  typename D1 = typename disable_if<std::is_same<typename std::remove_reference<T>::type &, T>>::type,
                  typename D2 = typename disable_if<std::is_const<T>>::type>
        any(T &&value) : _holder(std::make_shared<place_holder<T>>(std::forward<T>(value)))
        {
        }

        template <typename T>
        any(const T &value)
            : _holder(std::make_shared<place_holder<T>>(value))
        {
        }

        any(const std::initializer_list<any> &list)
            : _holder(std::make_shared<place_holder<array_t>>(list))
        {
        }

        any& operator=(const any& other) 
        {
            any(other).swap(*this);

            return *this;
        }
        any& operator=(any&& other)
        {
            any(std::move(other)).swap(*this);
            return *this;
        }

        template <typename T,
                  typename D1 = typename disable_if<std::is_same<typename std::remove_reference<T>::type &, T>>::type,
                  typename D2 = typename disable_if<std::is_const<T>>::type>
        any &operator=(T &&value)
        {
            if (_holder == nullptr || _holder->type_name() != typeid(typename std::remove_reference<T>::type).name())
            {
                *this = any(std::forward<T>(value));
            }
            else
            {
                std::dynamic_pointer_cast<place_holder<T>>(_holder)->value = std::forward<T>(value);
            }
            return *this;
        }

        template <typename T>
        any &operator=(const T &value)
        {
            if (_holder == nullptr || _holder->type_name() != typeid(typename std::remove_reference<T>::type).name())
            {
                *this = any(value);
            }
            else
            {
                std::dynamic_pointer_cast<place_holder<T>>(_holder)->value = value;
            }
            return *this;
        }

        any &operator=(decltype(nullptr))
        {
            reset();
            return *this;
        }

        any &operator=(const char *str)
        {
            if (_holder == nullptr || _holder->type_name() != typeid(std::string).name())
            {
                *this = any(str);
            }
            else
            {
                std::dynamic_pointer_cast<place_holder<std::string>>(_holder)->value = str;
            }
            return *this;
        }

        void reset() noexcept
        {
            _holder.reset();
        }

        void swap(any& other) noexcept
        {
            other._holder.swap(_holder);
        }


        std::vector<std::string> all_keys() const noexcept
        {
            if(is_object()) {
                auto& value = std::dynamic_pointer_cast<place_holder<object_t>>(_holder)->value;
                std::vector<std::string> keys;
                keys.reserve(value.size());
                for(const auto& pair: value)
                {
                    keys.emplace_back(pair.first);
                }
                return keys;
            }
            return {};
        }

        int count() const
        {
            if (is_object())
            {
                return std::dynamic_pointer_cast<place_holder<object_t>>(_holder)->value.size();
            }
            if (is_array())
            {
                return std::dynamic_pointer_cast<place_holder<array_t>>(_holder)->value.size();
            }

            return is_null() ? 0 : 1;
        }

        template <typename ValueType>
        ValueType value(typename std::remove_reference<ValueType>::type &&def) const noexcept
        {
            if (_holder && typeid(ValueType).name() == _holder->type_name())
            {
                return std::dynamic_pointer_cast<place_holder<ValueType>>(_holder)->value;
            }
            else
            {
                return def;
            }
        }

        void push_back(const any &value)
        {
            if (_holder == nullptr || _holder->type_name() != typeid(array_t).name())
            {
                _holder = std::make_shared<place_holder<array_t>>();
            }

            std::dynamic_pointer_cast<place_holder<array_t>>(_holder)->value.push_back(value);
        }

        template <typename... Args>
        void emplace_back(Args &&...args)
        {
            if (_holder == nullptr || _holder->type_name() != typeid(array_t).name())
            {
                _holder = std::make_shared<place_holder<array_t>>();
            }

            std::dynamic_pointer_cast<place_holder<array_t>>(_holder)->value.emplace_back(std::forward<Args>(args)...);
        }

        void insert(const char *key, const any &value)
        {
            insert(std::string(key), value);
        }

        void insert(const std::string &key, const any &value)
        {
            if (_holder == nullptr || _holder->type_name() != typeid(object_t).name())
            {
                _holder = std::make_shared<place_holder<object_t>>();
            }
            (std::dynamic_pointer_cast<place_holder<object_t>>(_holder)->value)
                .insert(std::make_pair(key, value));
        }

        const any &operator[](const std::string &key) const
        {
            if (_holder == nullptr || _holder->type_name() != typeid(object_t).name())
            {
                return any_null();
            }
            else
            {
                return (std::dynamic_pointer_cast<place_holder<object_t>>(_holder)->value)[key];
            }
        }

        const any &operator[](const char *key) const
        {
            return (*this)[std::string(key)];
        }

        any &operator[](const std::string &key)
        {
            if (_holder == nullptr || _holder->type_name() != typeid(object_t).name())
            {
                _holder = std::make_shared<place_holder<object_t>>(object_t{{key, any{}}});
            }
            return (std::dynamic_pointer_cast<place_holder<object_t>>(_holder)->value)[key];
        }

        any &operator[](const char *key)
        {
            return (*this)[std::string(key)];
        }

        any &operator[](int index)
        {
            if (!is_array())
            {
                throw std::invalid_argument("operator[](index) only use for array");
            }
            if (index >= count())
            {
                throw std::out_of_range("index is much more than size");
            }
            return (std::dynamic_pointer_cast<place_holder<array_t>>(_holder)->value)[index];
        }

        const any &operator[](int index) const
        {
            if (!is_array())
            {
                throw std::invalid_argument("operator[](index) only use for array");
            }
            if (index >= count())
            {
                throw std::out_of_range("index is much more than size");
            }
            return (std::dynamic_pointer_cast<place_holder<array_t>>(_holder)->value)[index];
        }

        std::string value(const char *def) const noexcept
        {
            return is_string() ? std::dynamic_pointer_cast<place_holder<std::string>>(_holder)->value : def;
        }

        bool is_string() const noexcept
        {
            return _holder && _holder->type_name() == typeid(std::string).name();
        }

        template <typename T>
        T value() const
        {
            static_assert(not std::is_reference<T>::value,
                          "value() cannot be used with reference types");
            static_assert(std::is_copy_constructible<T>::value,
                          "types must be CopyConstructible when used with value()");

            if (_holder && typeid(T).name() == _holder->type_name())
            {
                auto holder = std::dynamic_pointer_cast<place_holder<T>>(_holder);
                if (holder)
                    return holder->value;
            }

            throw std::bad_cast();
        }

        template <typename T>
        bool try_get(T &value)
        {
            if (_holder && typeid(T).name() == _holder->type_name())
            {
                value = std::dynamic_pointer_cast<place_holder<T>>(_holder)->value;
                return true;
            }
            else
            {
                return false;
            }
        }

        std::string type_name() const
        {
            return is_null() ? "null" : _holder->type_name();
        }

        template <typename T>
        operator T() const
        {
            return value<T>();
        }

        bool is_null() const noexcept
        {
            return _holder == nullptr;
        }

        bool is_number() const noexcept
        {
            return is_number_integer() || is_number_unsigned() || is_number_float();
        }

        template <typename T>
        T *pointer() const
        {
            if (_holder && typeid(T).name() == _holder->type_name())
            {
                return &(std::dynamic_pointer_cast<place_holder<T>>(_holder)->value);
            }
            return nullptr;
        }

        bool is_number_integer() const noexcept
        {
            if (is_null())
                return false;
            auto type = _holder->type_name();
            return type == typeid(char).name() || type == typeid(short).name() || type == typeid(int).name() || type == typeid(long).name() || type == typeid(long long).name();
        }

        bool is_boolean() const noexcept
        {
            if (is_null())
                return false;
            auto type = _holder->type_name();
            return type == typeid(bool).name();
        }

        bool is_number_unsigned() const noexcept
        {
            if (is_null())
                return false;
            auto type = _holder->type_name();
            return type == typeid(unsigned short).name() || type == typeid(unsigned char).name() || type == typeid(unsigned).name() || type == typeid(unsigned long).name() || type == typeid(unsigned long long).name();
        }

        bool is_number_float() const noexcept
        {
            if (is_null())
                return false;
            auto type = _holder->type_name();
            return type == typeid(float).name() || type == typeid(double).name();
        }

        bool is_array() const noexcept
        {
            return _holder && _holder->type_name() == typeid(array_t).name();
        }

        bool is_object() const noexcept
        {
            return _holder && _holder->type_name() == typeid(object_t).name();
        }

        struct internal_iterator
        {
            object_t::iterator object_iterator{};
            array_t::iterator array_iterator{};
        };

        struct const_iterator
        {
            const enum value_type {
                array,
                object
            } type;

            const_iterator(object_t &obj, bool begin = true) : type(object)
            {
                it.object_iterator = begin ? obj.begin() : obj.end();
            }

            const_iterator(array_t &arr, bool begin = true) : type(array)
            {
                it.array_iterator = begin ? arr.begin() : arr.end();
            }

            const_iterator(const const_iterator &other) : type(other.type), it{other.it} {}

            const_iterator(const_iterator &&other) : type(other.type), it{other.it}
            {
                other.it = internal_iterator{};
            }

            const std::string &key() const
            {
                if (type == object)
                {
                    return (*it.object_iterator).first;
                }
                throw std::runtime_error("any value type does not has key");
            }

            const any &value() const
            {
                return type == object ? (*it.object_iterator).second : *it.array_iterator;
            }

            bool operator==(const const_iterator &other) const
            {
                if (type != other.type)
                    return false;

                switch (type)
                {
                case object:
                    return it.object_iterator == other.it.object_iterator;
                case array:
                    return it.array_iterator == other.it.array_iterator;
                default:
                    break;
                }
                return false;
            }

            bool operator!=(const const_iterator &other) const
            {
                return !(*this == other);
            }

            const_iterator &operator++()
            {
                switch (type)
                {
                case object:
                    ++it.object_iterator;
                    break;
                case array:
                    ++it.array_iterator;
                    break;
                default:
                    break;
                }
                return *this;
            }

            const_iterator operator++(int)
            {
                auto result = *this;
                ++result;
                return result;
            }

            const_iterator &operator--()
            {
                switch (type)
                {
                case object:
                    --it.object_iterator;
                    break;
                case array:
                    --it.array_iterator;
                    break;
                default:
                    break;
                }
                return *this;
            }

            const_iterator operator--(int)
            {
                auto result = *this;
                ++result;
                return result;
            }

            const any &operator*() const
            {
                return type == array ? *it.array_iterator : (*it.object_iterator).second;
            }

            const any *operator->() const
            {
                const any &n = type == array ? *it.array_iterator : (*it.object_iterator).second;
                return &n;
            }

        protected:
            internal_iterator it;
        };

        struct iterator : public const_iterator
        {
            iterator(object_t &obj, bool begin = true) : const_iterator(obj, begin) {}
            iterator(array_t &arr, bool begin = true) : const_iterator(arr, begin) {}
            iterator(const iterator &other) : const_iterator(other) {}
            iterator(iterator &&other) : const_iterator(std::move(other)) {}

            any &operator*()
            {
                return type == array ? *it.array_iterator : (*it.object_iterator).second;
            }

            any *operator->()
            {
                any &n = type == array ? *it.array_iterator : (*it.object_iterator).second;
                return &n;
            }

            any &value()
            {
                return type == object ? (*it.object_iterator).second : *it.array_iterator;
            }
        };

        iterator begin()
        {
            if (is_object())
            {
                return iterator(std::dynamic_pointer_cast<place_holder<object_t>>(_holder)->value);
            }

            if (is_array())
            {
                return iterator(std::dynamic_pointer_cast<place_holder<array_t>>(_holder)->value);
            }

            throw std::runtime_error("any value has not begin iterator");
        }

        iterator end()
        {
            if (is_object())
            {
                return iterator(std::dynamic_pointer_cast<place_holder<object_t>>(_holder)->value, false);
            }

            if (is_array())
            {
                return iterator(std::dynamic_pointer_cast<place_holder<array_t>>(_holder)->value, false);
            }

            throw std::runtime_error("any value has end not iterator");
        }

        const_iterator begin() const
        {
            if (is_object())
            {
                return const_iterator(std::dynamic_pointer_cast<place_holder<object_t>>(_holder)->value, true);
            }

            if (is_array())
            {
                return const_iterator(std::dynamic_pointer_cast<place_holder<array_t>>(_holder)->value, true);
            }

            throw std::runtime_error("any value has not const begin iterator");
        }

        const_iterator end() const
        {
            if (is_object())
            {
                return const_iterator(std::dynamic_pointer_cast<place_holder<object_t>>(_holder)->value, false);
            }

            if (is_array())
            {
                return const_iterator(std::dynamic_pointer_cast<place_holder<array_t>>(_holder)->value, false);
            }

            throw std::runtime_error("any value has not const end iterator");
        }

        static const any &any_null()
        {
            static any null{};
            return null;
        }

    private:
        std::shared_ptr<holder> _holder;
    };

    template <typename T>
    T any_cast(const any &a)
    {
        return a.value<T>();
    }

    template <class T>
    T any_cast(any &a)
    {
        return a.value<T>();
    }

    template <class T>
    T any_cast(any &&a)
    {
        return a.value<T>();
    }

    template <class T>
    const T *any_cast(const any *a) noexcept
    {
        return a->pointer<T>();
    }

    template <class T>
    T *any_cast(any *a) noexcept
    {
        return a->pointer<T>();
    }
}

using any = common::any;

#endif // __COMMON_ANY__H__
