#ifndef DRIVERS__SOCKETCAN__SOCKET_CAN_RECEIVER_HPP_
#define DRIVERS__SOCKETCAN__SOCKET_CAN_RECEIVER_HPP_

#include <socketcan/socket_can_id.hpp>

#include <array>
#include <chrono>
#include <cstring>
#include <string>

namespace joypilot
{
namespace drivers
{
namespace socketcan
{

/// Simple RAII wrapper around a raw CAN receiver
class SocketCanReceiver
{
public:
  /// Constructor
  explicit SocketCanReceiver(const std::string & interface = "can0");
  /// Destructor
  ~SocketCanReceiver() noexcept;

  /// Receive CAN data
  /// \param[out] data A buffer to be written with data bytes. Must be at least 8 bytes in size
  /// \param[in] timeout Maximum duration to wait for data on the file descriptor. Negative
  ///                    durations are treated the same as zero timeout
  /// \return The CanId for the received can_frame, with length appropriately populated
  /// \throw SocketCanTimeout On timeout
  /// \throw std::runtime_error on other errors
  CanId receive(
    void * const data,
    const std::chrono::nanoseconds timeout = std::chrono::nanoseconds::zero()) const;
  /// Receive typed CAN data. Slightly less efficient than untyped interface; has extra copy and
  /// branches
  /// \tparam Type of data to receive, must be 8 bytes or smaller
  /// \param[out] data A buffer to be written with data bytes. Must be at least 8 bytes in size
  /// \param[in] timeout Maximum duration to wait for data on the file descriptor. Negative
  ///                    durations are treated the same as zero timeout
  /// \return The CanId for the received can_frame, with length appropriately populated
  /// \throw SocketCanTimeout On timeout
  /// \throw std::runtime_error If received data would not fit into provided type
  /// \throw std::runtime_error on other errors
  template<typename T, typename = std::enable_if_t<!std::is_pointer<T>::value>>
  CanId receive(
    T & data,
    const std::chrono::nanoseconds timeout = std::chrono::nanoseconds::zero()) const
  {
    static_assert(sizeof(data) <= MAX_DATA_LENGTH, "Data type too large for CAN");
    std::array<uint8_t, MAX_DATA_LENGTH> data_raw{};
    const auto ret = receive(&data_raw[0U], timeout);
    if (ret.length() != sizeof(data)) {
      throw std::runtime_error{"Received CAN data is of size incompatible with provided type!"};
    }
    (void)std::memcpy(&data, &data_raw[0U], ret.length());
    return ret;
  }

private:
  // Wait for file descriptor to be available to send data via select()
  void wait(const std::chrono::nanoseconds timeout) const;

  int32_t m_file_descriptor;
};  // class SocketCanReceiver

}  // namespace socketcan
}  // namespace drivers
}  // namespace joypilot

#endif  // DRIVERS__SOCKETCAN__SOCKET_CAN_RECEIVER_HPP_
