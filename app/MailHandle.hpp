/**
 * @file app/MailHandle.hpp
 * @brief MailHandle class for managing mail messages.
 */

#ifndef APP_MAILHANDLE_HPP
#define APP_MAILHANDLE_HPP

#include "rtos/Mail.h"

namespace app {

/**
 * @brief MailHandle class for managing mail messages.
 */
template <typename T, uint32_t N> class MailHandle {
public:
  /**
   * @brief Constructor.
   * @param mail Mail queue.
   */
  explicit MailHandle(rtos::Mail<T, N> &mail) noexcept
      : _mailQueue(&mail), _msg(mail.try_alloc()) {}

  /**
   * @brief Constructor.
   * @param mail Mail queue.
   * @param existing Existing message.
   */
  explicit MailHandle(rtos::Mail<T, N> &mail, T *existing) noexcept
      : _mailQueue(&mail), _msg(existing) {}

  ~MailHandle() noexcept { reset(); }
  MailHandle(const MailHandle &) noexcept = delete;
  MailHandle &operator=(const MailHandle &) noexcept = delete;

  /**
   * @brief Move constructor.
   * @param other Other MailHandle.
   */
  MailHandle(MailHandle &&other) noexcept
      : _mailQueue(other.releaseMsgQueue()), _msg(other.releaseMsg()) {}

  /**
   * @brief Move assignment operator.
   * @param other Other MailHandle.
   * @return This MailHandle.
   */
  MailHandle &operator=(MailHandle &&other) noexcept {
    if (this != &other) {
      reset();
      _mailQueue = other.releaseMsgQueue();
      _msg = other.releaseMsg();
    }
    return *this;
  }

  /**
   * @brief Get the message.
   * @return Message.
   */
  T *getMsg() const noexcept { return _msg; }

  /**
   * @brief Release the message.
   * @return Message.
   */
  T *releaseMsg() noexcept {
    T *tmp = _msg;
    _msg = nullptr;
    return tmp;
  }

  /**
   * @brief Check if the message is valid.
   * @return True if the message is valid, false otherwise.
   */
  explicit operator bool() const noexcept { return _msg != nullptr; }

private:
  /**
   * @brief Release the message queue.
   * @return Message queue.
   */
  rtos::Mail<T, N> *releaseMsgQueue() noexcept {
    rtos::Mail<T, N> *tmp = _mailQueue;
    _mailQueue = nullptr;
    return tmp;
  }

  /**
   * @brief Reset the message.
   */
  void reset() noexcept {
    if (_msg && _mailQueue) {
      _mailQueue->free(_msg);
      _msg = nullptr;
    }
  }

  rtos::Mail<T, N> *_mailQueue;
  T *_msg;
};

} // namespace app

#endif // APP_MAILHANDLE_HPP
