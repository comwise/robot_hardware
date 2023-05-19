/*!
  \file     endpoint.h
  \brief    ip address, port and protocol
  \author   lichanglin
  \version  v1.0
*/

#ifndef __COMWISE_NET__NET_ENDPOINT__H__
#define __COMWISE_NET__NET_ENDPOINT__H__

#include <sstream>
#include <vector>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <algorithm>

namespace comwise {
namespace net {

typedef unsigned char octet_t;

enum endianness_t {
    LITTLEEND = 0x0, //! Little endianness.
    BIGEND = 0x1, //! Big endianness.
};

enum kind_t {
    ENDPOINT_KIND_INVALID = -1, //! Invalid Tcp/IP protocol.
    ENDPOINT_KIND_RESERVED = 0, //! Reserved Tcp/IP protocol .
    ENDPOINT_KIND_IPv4 = 1,  //! IPv4.
    ENDPOINT_KIND_IPv6 = 2,  //! IPv6.
};

#define ENDPOINT_PORT_INVALID 0
#define ENDPOINT_ADDRESS_INVALID(addr) { std::memset(addr, 0x00, 16 * sizeof(octet_t)); }
#define ENDPOINT_INVALID(ep) {            \
    ep.kind = ENDPOINT_KIND_INVALID;      \
    ep.port = ENDPOINT_PORT_INVALID;      \
    ENDPOINT_ADDRESS_INVALID(ep.address); }


//!@brief class endpoint, uniquely identifies a communication channel for a particular transport. 
//For example, an address+port combination in the case of UDP.
class endpoint 
{
public:

    kind_t kind;
    uint16_t port;
    char address[16];

    endpoint()
        : kind(ENDPOINT_KIND_IPv4), port(ENDPOINT_PORT_INVALID) {
        ENDPOINT_ADDRESS_INVALID(address);
    }

    endpoint(endpoint &&ep)
        : kind(ep.kind), port(ep.port) {
        std::memcpy(address, ep.address, 16 * sizeof(octet_t));
    }

    endpoint(const endpoint &ep)
        : kind(ep.kind), port(ep.port) {
        std::memcpy(address, ep.address, 16 * sizeof(octet_t));
    }

    endpoint(uint16_t port)
        : kind(ENDPOINT_KIND_IPv4), port(port) {
        ENDPOINT_ADDRESS_INVALID(address);
    }

    endpoint &operator=(const endpoint &ep) {
        if (this != &ep) {
            kind = ep.kind;
            port = ep.port;
            std::memcpy(address, ep.address, 16 * sizeof(octet_t));
        }
        return *this;
    }

    bool set_ip4_address(octet_t o1, octet_t o2, octet_t o3, octet_t o4) {
        ENDPOINT_ADDRESS_INVALID(address);
        address[12] = o1;
        address[13] = o2;
        address[14] = o3;
        address[15] = o4;
        return true;
    }

    bool set_ip4_address(const std::string &in_address) {
        std::stringstream ss(in_address);
        uint8_t a=0, b=0, c=0, d=0; //to store the 4 ints
        char ch=0;        //to temporarily store the '.'
        ss >> a >> ch >> b >> ch >> c >> ch >> d;
        ENDPOINT_ADDRESS_INVALID(address);
        address[12] = (octet_t)a;
        address[13] = (octet_t)b;
        address[14] = (octet_t)c;
        address[15] = (octet_t)d;
        return true;
    }

    std::string to_ip4_string() const {
        std::stringstream ss;
        ss  << (int)address[12] << "."
            << (int)address[13] << "." 
            << (int)address[14] << "." 
            << (int)address[15];
        return ss.str();
    }

    uint32_t to_ip4_long() {
        uint32_t addr = 0;
        octet_t* oaddr = (octet_t*)&addr;
#if __BIG_ENDIAN__
        std::memcpy(oaddr,address+12,4*sizeof(octet_t));
#else
        oaddr[0] = address[15];
        oaddr[1] = address[14];
        oaddr[2] = address[13];
        oaddr[3] = address[12];
#endif
        return addr;
    }

    bool set_ip6_address(
        uint16_t group0,
        uint16_t group1,
        uint16_t group2,
        uint16_t group3, 
        uint16_t group4,
        uint16_t group5,
        uint16_t group6,
        uint16_t group7) {
       address[0]  = (octet_t) (group0 >> 8);
       address[1]  = (octet_t) group0;
       address[2]  = (octet_t) (group1 >> 8);
       address[3]  = (octet_t) group1;
       address[4]  = (octet_t) (group2 >> 8);
       address[5]  = (octet_t) group2;
       address[6]  = (octet_t) (group3 >> 8);
       address[7]  = (octet_t) group3;
       address[8]  = (octet_t) (group4 >> 8);
       address[9]  = (octet_t) group4;
       address[10] = (octet_t) (group5 >> 8);
       address[11] = (octet_t) group5;
       address[12] = (octet_t) (group6 >> 8);
       address[13] = (octet_t) group6;
       address[14] = (octet_t) (group7 >> 8);
       address[15] = (octet_t) group7;
       return true;
    }

   std::string to_ip6_string() const {
       std::stringstream ss;
       ss << std::hex;
       for (int i = 0; i != 14; i += 2) {
           auto field = (address[i] << 8) + address[i + 1];
           ss << field << ":";
       }
       auto field = address[14] + (address[15] << 8);
       ss << field;
       return ss.str();
   }
};

inline bool is_address_defined(const endpoint &ep)
{
    if (ep.kind == ENDPOINT_KIND_IPv4) {
        for (uint8_t i = 12; i < 16; ++i)
            if (ep.address[i] != 0)
                return true;
    } else if (ep.kind == ENDPOINT_KIND_IPv6) {
        for (uint8_t i = 0; i < 16; ++i)
            if (ep.address[i] != 0)
                return true;
    } else {
        // todo
    }
    return false;
}

inline bool is_endpoint_valid(const endpoint &ep)
{
    if (ep.kind < 0)
        return false;
    return true;
}

inline bool operator==(const endpoint &ep1, const endpoint &ep2)
{
    if (ep1.kind != ep2.kind)
        return false;
    if (ep1.port != ep2.port)
        return false;
    if (!std::equal(ep1.address, ep1.address + 16, ep2.address))
        return false;
    return true;
}

inline std::ostream &operator<<(std::ostream &output, const endpoint &ep)
{
    if (ep.kind == ENDPOINT_KIND_IPv4) {
        output 
            << (int)ep.address[12] << "."
            << (int)ep.address[13] << "."
            << (int)ep.address[14] << "."
            << (int)ep.address[15] << ":"
            << ep.port;
    }
    else if (ep.kind == ENDPOINT_KIND_IPv6) {
        for (uint8_t i = 0; i < 16; ++i) {
            output << (int)ep.address[i];
            if (i < 15)  {
                output << ".";
            }
        }
        output << ":" << ep.port;
    }
    else {
        // todo
    }
    return output;
}

} // net
} // comwise

#endif // __COMWISE_NET__NET_ENDPOINT__H__
