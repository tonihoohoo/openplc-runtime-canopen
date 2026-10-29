// SPDX-License-Identifier: Apache-2.0
//
// pi_mutex.h - a mutex with priority inheritance, for state shared between a bus thread (which
// may run at a real-time priority) and a thread at normal scheduling. Usable with
// std::lock_guard and std::unique_lock.

#ifndef CANOPEN_PI_MUTEX_H
#define CANOPEN_PI_MUTEX_H

#include <pthread.h>

namespace canopen_plugin
{

class PiMutex
{
  public:
    PiMutex()
    {
        pthread_mutexattr_t attr;
        pthread_mutexattr_init(&attr);
        // Falls back to a plain mutex where the protocol is not supported.
        pthread_mutexattr_setprotocol(&attr, PTHREAD_PRIO_INHERIT);
        if (pthread_mutex_init(&m_, &attr) != 0)
            pthread_mutex_init(&m_, nullptr);
        pthread_mutexattr_destroy(&attr);
    }
    ~PiMutex()
    {
        pthread_mutex_destroy(&m_);
    }
    PiMutex(const PiMutex &)            = delete;
    PiMutex &operator=(const PiMutex &) = delete;

    void lock()
    {
        pthread_mutex_lock(&m_);
    }
    void unlock()
    {
        pthread_mutex_unlock(&m_);
    }

  private:
    pthread_mutex_t m_;
};

} // namespace canopen_plugin

#endif // CANOPEN_PI_MUTEX_H
