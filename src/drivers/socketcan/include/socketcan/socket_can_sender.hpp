#ifndef DRIVERS__SOCKETCAN__SOCKET_CAN_SENDER_HPP_
#define DRIVERS__SOCKETCAN__SOCKET_CAN_SENDER_HPP_

#include <joy_common/joy_types.hpp>
#include <socketcan/socket_can_id.hpp>

#include <chrono>
#include <string>

using joypilot::common::types::char8_t;

namespace joypilot
{
namespace drivers
{
namespace socketcan
{

/// Simple RAII wrapper around a raw CAN sender
class SocketCanSender
{
public:
  /// Constructor
  explicit SocketCanSender(
    const std::string & interface = "can0",
    const CanId & default_id = CanId{});
  /// Destructor
  ~SocketCanSender() noexcept;

  /// Send raw data with the default id
  /// \param[in] data A pointer to the beginning of the data to send
  /// \param[in] timeout Maximum duration to wait for file descriptor to be free for write. Negative
  ///                    durations are treated the same as zero timeout
  /// \param[in] length The amount of data to send starting from the data pointer
  /// \throw std::domain_error If length is > 8
  /// \throw SocketCanTimeout On timeout
  /// \throw std::runtime_error on other errors
  void send(
    const void * const data,
    const std::size_t length,
    const std::chrono::nanoseconds timeout = std::chrono::nanoseconds::zero()) const;
  /// Send raw data with an explicit CAN id
  /// \param[in] data A pointer to the beginning of the data to send
  /// \param[in] timeout Maximum duration to wait for file descriptor to be free for write. Negative
  ///                    durations are treated the same as zero timeout
  /// \param[in] id The id field for the CAN frame
  /// \param[in] length The amount of data to send starting from the data pointer
  /// \throw std::domain_error If length is > 8
  /// \throw SocketCanTimeout On timeout
  /// \throw std::runtime_error on other errors
  void send(
    const void * const data,
    const std::size_t length,
    const CanId id,
    const std::chrono::nanoseconds timeout = std::chrono::nanoseconds::zero()) const;
  /// Send typed data with the default id
  /// \tparam Type of data to send, must be 8 bytes or smaller
  /// \param[in] data The data to send
  /// \param[in] timeout Maximum duration to wait for file descriptor to be free for write. Negative
  ///                    durations are treated the same as zero timeout
  /// \throw SocketCanTimeout On timeout
  /// \throw std::runtime_error on other errors
  template<typename T, typename = std::enable_if_t<!std::is_pointer<T>::value>>
  void send(
    const T & data,
    const std::chrono::nanoseconds timeout = std::chrono::nanoseconds::zero()) const
  {
    send(data, m_default_id, timeout);
  }

  /// Send typed data with an explicit CAN Id
  /// \tparam Type of data to send, must be 8 bytes or smaller
  /// \param[in] data The data to send
  /// \param[in] timeout Maximum duration to wait for file descriptor to be free for write. Negative
  ///                    durations are treated the same as zero timeout
  /// \param[in] id The id field for the CAN frame
  /// \throw SocketCanTimeout On timeout
  /// \throw std::runtime_error on other errors
  template<typename T, typename = std::enable_if_t<!std::is_pointer<T>::value>>
  void send(
    const T & data,
    const CanId id,
    const std::chrono::nanoseconds timeout = std::chrono::nanoseconds::zero()) const
  {
    static_assert(sizeof(data) <= MAX_DATA_LENGTH, "Data type too large for CAN");
    //lint -e586 I have to use reinterpret cast because I'm operating on bytes, see below NOLINT
    send_impl(reinterpret_cast<const char8_t *>(&data), sizeof(data), id, timeout);
    // reinterpret_cast to byte, or (unsigned) char is well defined;
    // all pointers can implicitly convert to void *
  }

  /// Get the default CAN id
  CanId default_id() const noexcept;

private:
  // Underlying implementation of sending, data is assumed to be of an appropriate length
  void send_impl(
    const void * const data,
    const std::size_t length,
    const CanId id,
    const std::chrono::nanoseconds timeout) const;
  // Wait for file descriptor to be available to send data via select()
  void wait(const std::chrono::nanoseconds timeout) const;

  int32_t m_file_descriptor{};
  CanId m_default_id;
};  // class SocketCanSender

}  // namespace socketcan
}  // namespace drivers
}  // namespace joypilot

#endif  // DRIVERS__SOCKETCAN__SOCKET_CAN_SENDER_HPP_
