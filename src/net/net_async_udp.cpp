#include "net/net_async_udp.h"

namespace comwise {
namespace net {

AsyncUDPClient::AsyncUDPClient(
    const PacketHandler &packet_handler,
    boost::asio::io_service &io_service,
    const uint16_t &local_port)
    : m_packet_handler(packet_handler)
    , m_io_service(io_service)
{
  // Keep io_service busy
  m_io_work_ptr = std::make_shared<boost::asio::io_service::work>(m_io_service);
  try {
    auto endpoint = boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), local_port);
    m_socket_ptr = std::make_shared<boost::asio::ip::udp::socket>(m_io_service, endpoint);
  } catch (const std::exception &e) {
    ROS_ERROR("Exception while creating socket: %s", e.what());
  }
  ROS_INFO("UDP client is setup");
}

AsyncUDPClient::~AsyncUDPClient()
{
  m_io_service.stop();
}

void AsyncUDPClient::startReceive()
{
  m_socket_ptr->async_receive_from(boost::asio::buffer(m_recv_buffer),
                                   m_remote_endpoint,
                                   [this](boost::system::error_code ec, std::size_t bytes_recvd)
                                   {
                                     this->handleReceive(ec, bytes_recvd);
                                   });
}

void AsyncUDPClient::handleReceive(const boost::system::error_code &error,
                                   const std::size_t &bytes_transferred)
{
  if (!error) {
    sick::datastructure::PacketBuffer packet_buffer(m_recv_buffer, bytes_transferred);
    m_packet_handler(packet_buffer);
  } else {
    ROS_ERROR("Error in UDP handle receive: %i", error.value());
  }
  startReceive();
}

void AsyncUDPClient::runService()
{
  startReceive();
}

unsigned short AsyncUDPClient::getLocalPort()
{
  if (m_socket_ptr) {
    return m_socket_ptr->local_endpoint().port();
  }
  return 0;
}

} // namespace net
} // namespace comwise
