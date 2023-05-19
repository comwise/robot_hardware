/*!
  \file     net_impl.h
  \brief    net c++ interface implement
  \author   lichanglin
  \version  v1.0
*/
#ifndef __COMWISE_NET__NET_IMPL__H__
#define __COMWISE_NET__NET_IMPL__H__

#include <string>
#include <functional>
#include "net_def.h"

namespace comwise {
namespace net {

class endpoint;

class net_base {

public:
  net_base(net_protoc_t protoc) 
    : net_protoc_(NET_PROTOC_NONE)
  { }

  virtual ~net_base() 
  { }

  virtual net_protoc_t net_protoc() const
  { 
    return net_protoc_; 
  }

protected:
  net_protoc_t net_protoc_;

};

class net_tcp : public net_base {

public:
  net_tcp() : net_base(NET_PROTOC_TCP) { };
  net_tcp(const net_link_t link);
  virtual ~net_tcp();

public:
  int create();
  int create(const char *ep, int16_t port);
  int create(const endpoint &ep);
  int close();

  int connect(const char *ep, int16_t port);
  int connect(const endpoint &ep);
  int listen();
  int send(int size, const std::function<int( void *, int) > &data);
  int send(const std::string &data);
  int send(const void *data, int size);

protected:
  virtual void on_closed(net_link_t link);
  virtual void on_recvdata(const std::string &data);
  virtual void on_accepted(net_link_t lnk);

private:
  net_tcp(const net_tcp &) = delete;
  net_tcp(net_tcp &&) = delete;
  net_tcp &operator=(const net_tcp &) = delete;

};


class net_udp : public net_base {

public:
  public:
  net_udp() : net_base(NET_PROTOC_UDP) { };
  net_udp(const net_link_t link);
  virtual ~net_udp();

public:
  int create();
  int create(const char *ep);
  int create(const endpoint &ep);
  int close();

  int send(int size, const std::function<int( void *, int) > &data, const endpoint &ep);
  int send(const std::string &data, const endpoint &ep);
  int send(const void *data, int size, const endpoint &ep);

protected:
  virtual void on_closed(net_link_t link);
  virtual void on_recvdata(const std::string &data, const endpoint &ep);

private:
  net_udp(const net_udp &) = delete;
  net_udp(net_udp &&) = delete;
  net_udp &operator=(const net_udp &) = delete;

};

} // namespace net
} // namespace comwise

#endif // __COMWISE_NET__NET_IMPL__H__
