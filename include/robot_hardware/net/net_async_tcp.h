#ifndef __COMWISE_NET__NET_ASYNC_TCP__H__
#ifndef __COMWISE_NET__NET_ASYNC_TCP__H__

#include <cstdint>
#include <functional>
#include <iostream>
#include <thread>
#include <mutex>
#include <condition_variable>

#include <boost/array.hpp>
#include <boost/asio.hpp>
#include "common/any.h"

namespace comwise {
namespace net {

class AsyncTCPClient
{
public:
  typedef boost::function<void(const any&)> PacketHandler;

  /*!
   * \brief Constructor of the asynchronous tcp client.
   *
   * \param packet_handler Function which handles the packets on processing.
   * \param io_service The boost io_service instance.
   * \param server_ip The IP address of the server to connect to.
   * \param server_port The port on the server to connect to.
   */
  AsyncTCPClient(const PacketHandler& packet_handler,
                 boost::asio::io_service& io_service,
                 const boost::asio::ip::address_v4& server_ip,
                 const uint16_t& server_port);
  virtual ~AsyncTCPClient();

  void connect();
  void disconnect();

  void doSendAndReceive(const std::vector<uint8_t>& sendBuffer);
  void init();
  void set_data_callback(const PacketHandler& packet_handler);

private:
  void handleReceive(const boost::system::error_code& error, const std::size_t& bytes_transferred);

  AsyncTCPClient(AsyncTCPClient&); // block default copy constructor

  void handleSendAndReceive(const boost::system::error_code& error,
                            const std::size_t& bytes_transferred);

private:
  datastructure::PacketBuffer::ArrayBuffer m_recv_buffer;

  PacketHandler m_packet_handler;

  std::shared_ptr<boost::asio::io_service::work> io_work_;
  boost::asio::io_service& m_io_service;
  std::shared_ptr<boost::asio::ip::tcp::socket> tcp_socket_;
  boost::asio::ip::tcp::endpoint remote_endpoint_;
  std::thread m_service_thread;

  std::condition m_connect_condition;
  std::mutex m_connect_mutex;
  std::mutex socket_mtx_;
};

} // namespace net
} // namespace comwise

#endif // __COMWISE_NET__NET_ASYNC_TCP__H__
