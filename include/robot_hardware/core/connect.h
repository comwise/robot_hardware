#ifndef __COMWISE__CORE__CONNECT__H__
#define __COMWISE__CORE__CONNECT__H__

#include <atomic>
#include <memory>
#include <mutex>
#include <map>
#include <vector>
#include <functional>
#include <future>

namespace core {

template <typename TParam>
class slot_base
{
public:
    virtual ~slot_base() = default;

    virtual void slot_callback(TParam) = 0;
};

template <class TRecver, typename TParam>
class slot : public slot_base<TParam>
{
public:
    slot(TRecver *pObj, void (TRecver::*recverFunc)(TParam)) {
        this->m_pSlotObj = pObj;       //使用类外的接收者类的对象指针进行初始化。
        this->m_slotFunc = recverFunc; //使用类外的接收者类中的成员函数指针进行初始化。
    }

    void slot_callback(TParam param) override {
        (m_pSlotObj->*m_slotFunc)(param); //成员对象指针调用类内的成员函数
    }

private:
    TRecver *m_pSlotObj;                 //定义一个接收者（serder）的指针，在构造中对其初始化。
    void (TRecver::*m_slotFunc)(TParam); //定义一个接收者类中的成员函数指针，在构造中对其初始化。
};

template <typename TParam>
class signal
{
public:
    template <class TRecver>
    void add_slot(TRecver *pSlotObj, void (TRecver::*slotFunc)(TParam)) {
        auto slotObj = new slot<TRecver, TParam>(pSlotObj, slotFunc);
        signal_vector.push_back(slotObj);
    }

    void operator()(TParam param) {
        for (auto p : signal_vector) {
            if(p) {
                p->slot_callback(param);
            }
        }
    }

private:
    std::vector<slot_base<TParam> *> signal_vector;
};

#define connect(sender, signal, recver, slotFunc) (sender)->signal.add_slot(recver, slotFunc)

} // namespace core

#endif // __COMWISE__CORE__CONNECT__H__
