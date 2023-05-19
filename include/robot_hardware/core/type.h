#ifndef __COMWISE_CORE__HARDWARE_TYPE__H__
#define __COMWISE_CORE__HARDWARE_TYPE__H__

#include <cstdint>
#include <memory>
#include <string>

namespace core {

class hw_type
{
    enum type_const_t {
        TYPE_INIT_MASK = 0x00000000,
        TYPE_HB_MASK = 0xffff0000,
        TYPE_LB_MASK = 0x0000ffff,
        TYPE_MOVE_BIT = 16
    };
public:
    using hw_type_t = uint32_t;
    using hw_id_t = std::string;

public:
    explicit hw_type(hw_id_t id, hw_type_t type) : id_(id) { set_major_type(type); }
    virtual ~hw_type() { }

    //! hardware type
    virtual void set_type(hw_type_t type) { set_major_type(type); }
    virtual hw_type_t get_type() const { return get_major_type(); }

    //! hardware id
    virtual void set_id(const hw_id_t &id) { id_ = id; }
    virtual hw_id_t get_id() const { return id_; }

    //! set/get major type 
    void set_major_type(hw_type_t major) { type_ = (type_&TYPE_LB_MASK)|((major<<16)&TYPE_HB_MASK); }
    hw_type_t get_major_type() const { return (type_>>16)&TYPE_LB_MASK; }

    //!  set/get minor type
    void set_minor_type(hw_type_t minor) { type_ = (type_&TYPE_HB_MASK)|(minor&TYPE_LB_MASK); }
    hw_type_t get_minor_type() const { return type_&TYPE_LB_MASK; }

protected:
    hw_type_t type_ {TYPE_INIT_MASK};
    hw_id_t id_;
};

} // namespace core
 
#endif // __COMWISE_CORE__HARDWARE_TYPE__H__
