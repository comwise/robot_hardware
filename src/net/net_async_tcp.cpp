#include "net/net_async_tcp.h"

namespace comwise {
namespace net {

AsyncTCPClient::AsyncTCPClient(
    const PacketHandler &packet_handler,
    boost::asio::io_service &io_service,
    const boost::asio::ip::address_v4 &server_ip,
    const uint16_t &server_port)
  : m_packet_handler(packet_handler)
  , m_io_service(io_service)
{
    // Keep io_service busy
    io_work_ = std::make_shared<boost::asio::io_service::work>(m_io_service);
    try {
      tcp_socket_ = std::make_shared<boost::asio::ip::tcp::socket>(m_io_service);
    } catch (const std::exception &e) {
      ROS_ERROR("Exception while creating socket: %s", e.what());
    }
    remote_endpoint_ = boost::asio::ip::tcp::endpoint(server_ip, server_port);
    ROS_INFO("TCP client is setup");
}

AsyncTCPClient::~AsyncTCPClient()
{
  
}

void AsyncTCPClient::connect()
{
  boost::mutex::scoped_lock lock(socket_mtx_);
  boost::mutex::scoped_lock lock_connect(m_connect_mutex);
  tcp_socket_->async_connect(remote_endpoint_, 
      [this](boost::system::error_code ec) {
          if (ec != boost::system::errc::success) {
            ROS_ERROR("TCP error code: %i", ec.value());
          } else {
            ROS_INFO("TCP connection successfully established.");
          }
          m_connect_condition.notify_all(); 
      });
  m_connect_condition.wait(lock_connect);
}

void AsyncTCPClient::disconnect()
{
  std::guard_lock<> lock(socket_mtx_);
  boost::system::error_code ec;
  tcp_socket_->shutdown(boost::asio::ip::tcp::socket::shutdown_both, ec);
  if (ec != boost::system::errc::success) {
    ROS_ERROR("Error shutting socket down: %i", ec.value());
  } else {
    ROS_INFO("TCP Connection successfully shutdown");
  }

  tcp_socket_->close(ec);
  if (ec != boost::system::errc::success) {
    ROS_ERROR("Error closing Socket: %i", ec.value());
  } else {
    ROS_INFO("TCP Socket successfully closed.");
  }
}

void AsyncTCPClient::doSendAndReceive(const std::vector<uint8_t> &sendBuffer)
{
  boost::mutex::scoped_lock lock(socket_mtx_);
  if (!tcp_socket_) {
    return;
  }

  boost::asio::async_write(
        *tcp_socket_,
        boost::asio::buffer(sendBuffer),
        [this](boost::system::error_code ec, std::size_t bytes_send)
        {
          this->handleSendAndReceive(ec, bytes_send);
        });
}

void AsyncTCPClient::init()
{
  boost::mutex::scoped_lock lock(socket_mtx_);
  if (!tcp_socket_) {
    tcp_socket_->async_read_some(
          boost::asio::buffer(m_recv_buffer),
          [this](boost::system::error_code ec, std::size_t bytes_recvd)
          {
            this->handleReceive(ec, bytes_recvd);
          });
  }
}

void AsyncTCPClient::set_data_callback(const PacketHandler &packet_handler)
{
  m_packet_handler = packet_handler;
}

void AsyncTCPClient::handleSendAndReceive(const boost::system::error_code &error,
                                          const std::size_t &bytes_transferred)
{
  // Check for errors
  if (!error || error == boost::asio::error::message_size){
    init();
  } else {
    ROS_ERROR("Error in tcp handle send and receive: %i", error.value());
  }
}

void AsyncTCPClient::handleReceive(const boost::system::error_code &error,
                                   const std::size_t &bytes_transferred)
{
  if (!error) {
    sick::datastructure::PacketBuffer packet_buffer(m_recv_buffer, bytes_transferred);
    m_packet_handler(packet_buffer);
  } else {
    ROS_ERROR("Error in tcp handle receive: %i", error.value());
  }
}

} // namespace net
} // namespace comwise
