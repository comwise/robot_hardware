#ifndef __COMWISE_NET__NET_ASYNC_UDP__H__
#define __COMWISE_NET__NET_ASYNC_UDP__H__

#include <cstdint>
#include <functional>
#include <iostream>
#include <thread>
#include <condition_variable>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include "common/any.h"

namespace comwise {
namespace net {

/*!
 * \brief An asynchronous udp client.
 */
class AsyncUDPClient
{
public:
  /*!
   * \brief Typedef to a reference to a function. Will be used to process the incoming packets.
   */
  typedef boost::function<void(const any&)> PacketHandler;

  /*!
   * \brief Constructor of the asynchronous udp client.
   *
   * \param packet_handler Function to handle incoming packets.
   * \param io_service Instance of the boost io_service.
   * \param local_port The local port, where the udp packets will arrive.
   */
  AsyncUDPClient(const PacketHandler& packet_handler,
                 boost::asio::io_service& io_service,
                 const uint16_t& local_port = 0);
  virtual ~AsyncUDPClient();

  void runService();
  unsigned short getLocalPort();

private:
  AsyncUDPClient(AsyncUDPClient&); // block default copy constructor

  void startReceive();
  void handleReceive(const boost::system::error_code& error, const std::size_t& bytes_transferred);

private:
  datastructure::PacketBuffer::ArrayBuffer m_recv_buffer;

  PacketHandler m_packet_handler;

  std::shared_ptr<boost::asio::io_service::work> m_io_work_ptr;
  boost::asio::io_service& m_io_service;
  std::shared_ptr<boost::asio::ip::udp::socket> m_socket_ptr;
  boost::asio::ip::udp::endpoint m_remote_endpoint;

};

} // namespace net
} // namespace comwise

#endif // __COMWISE_NET__NET_ASYNC_UDP__H__
