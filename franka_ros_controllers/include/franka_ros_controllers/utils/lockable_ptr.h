#pragma once

#include <memory>
#include <mutex>
#include <shared_mutex>

namespace franka_ros_controllers {

template <typename T, typename Lock, typename Mutex>
class shared_ptr_with_mutex;

/**
 * Pointer helper-class that holds a lockable object to realize easy concurrent access to an object.
 * The object can be locked by calling lock/try_lock. This will return a shared_ptr. As long as this
 * shared_ptr instance exists, the object is "locked" and other calls to lock() will wait for its
 * destruction. The returned shared_ptr should **not** be shared between threads. Otherwise the
 * locking effect is circumvented. The shared_ptr should also not be stored as a class member since
 * the object is locked during the life-cycle of the shared_ptr.
 *
 * @note The default mutex that is being used is the std::recursive_mutex. This mutex is needed for
 * usages like `my_func(lockable_ptr.lock()->member_a, lockable_ptr.lock()->member_b);`
 * since the lock is only freed after completion of the full line and this will lead to deadlocks
 * with the normal mutex. The mutex can be changed via the template parameter Mutex. It is possible
 * to use a standard mutex if the above usage is avoided.
 *
 * @tparam T Type of the contained object
 * @tparam Mutex Type of mutex to be used
 * @tparam WriteLock Lock type for lock-calls that return a non-const object
 * @tparam ReadLock Lock type for lock-calls that return a const object.
 *
 * Usage:
 * @verbatim
    lockable_ptr<std::string> ptr(new std::string("foobar"));
    // or lockable_ptr<std::string> ptr("foobar"); // uses varargs
    {
      // store as temporary shared_ptr with lock() or try_lock()
      auto locked_ptr = ptr.try_lock();
    }
    {
      // direct usage
      ptr.lock()->clear();
    }
  @endverbatim
 *
 */
template <typename T,
          typename Mutex = std::recursive_mutex,
          typename WriteLock = std::unique_lock<Mutex>,
          typename ReadLock = std::unique_lock<Mutex>>

class lockable_ptr {
 public:
  using Ptr = shared_ptr_with_mutex<T, Mutex, WriteLock>;
  using ConstPtr = shared_ptr_with_mutex<const T, Mutex, ReadLock>;

  /**
   * Constructor that takes the arguments of the class's constructor and creates the object on the
   * heap.
   */
  template <typename... Params>
  lockable_ptr(Params&&... v) : ptr_(new T(std::forward<Params>(v)...)) {}
  /**
   * Constructor that takes a pointer to the object that should mutex-protected.
   * @note It is suggested to use the var-args constructor to avoid wrong usage.
   * @param ptr Pointer to the object. Must not be used anywhere else. Must be on the heap. This
   * class takes ownership of the object.
   */
  lockable_ptr(T* ptr) : ptr_(ptr) {}

  /**
   * Tries to lock the object, if it is not locked yet.
   * @return Returns pointer to object if locked successfully, otherwise nullptr.
   */
  ConstPtr try_lock() const;
  Ptr try_lock();
  /**
   * Explicit read-lock that returns a const object. Convenience function if the calling scope is
   * not const, but the returned object should be const.
   * @return  Returns pointer to const object if locked successfully, otherwise nullptr.
   */
  ConstPtr try_read_lock() const;

  /**
   * Locks the objects. If already locked, will wait for unlock indefinitely.
   * @return Returns the pointer to the object.
   */
  ConstPtr lock() const { return read_lock(); }
  Ptr lock() { return Ptr(mutex_, ptr_); }
  /**
   * Explicit const-lock that returns a const object. Convenience function if the calling scope is
   * not const, but the returned object should be const.
   * @return  Returns the pointer to the const object.
   */
  ConstPtr read_lock() const { return ConstPtr(mutex_, ptr_); }

 private:
  std::shared_ptr<T> ptr_;
  mutable Mutex mutex_;
};

/**
 * This class extends the normal shared_ptr class with a lock,
 * that locks a mutex on construction and unlocks it on destruction.
 */
template <typename T,
          typename Mutex = std::recursive_mutex,
          typename Lock = std::unique_lock<Mutex>>
class shared_ptr_with_mutex : public std::shared_ptr<T> {
 public:
  shared_ptr_with_mutex(std::nullptr_t) { lock_ = std::make_shared<Lock>(); }
  shared_ptr_with_mutex() { lock_ = std::make_shared<Lock>(); }
  shared_ptr_with_mutex(const shared_ptr_with_mutex&) = default;
  shared_ptr_with_mutex& operator=(shared_ptr_with_mutex& rhs) = default;
  shared_ptr_with_mutex(Mutex& mutex, const std::shared_ptr<T>& ptr)
      : std::shared_ptr<T>(ptr), lock_(std::make_shared<Lock>(mutex)) {}
  shared_ptr_with_mutex(const std::shared_ptr<Lock>& lock, const std::shared_ptr<T>& ptr)
      : std::shared_ptr<T>(ptr), lock_(lock) {}

  /**
   * Unlocks the mutex. The mutex is also unlocked in the detructor of this class.
   * This function can be called, if the mutex should already be unlocked before the destructor.
   */
  void unlock() { lock_->unlock(); }

  /**
   * Calls lock on the underlying lock.
   */
  void lock() { lock_->lock(); }

  /**
   * Provides access to the private lock of this class to access non-forwarded functions.
   */
  Lock& getLock() { return *lock_; }

  /**
   * Provides const access to the private lock of this class to access non-forwarded functions.
   */
  const Lock& getLock() const { return *lock_; }

 private:
  std::shared_ptr<Lock> lock_;
};

template <typename T, typename Mutex, typename WriteLock, typename ReadLock>
typename lockable_ptr<T, Mutex, WriteLock, ReadLock>::ConstPtr
lockable_ptr<T, Mutex, WriteLock, ReadLock>::try_lock() const {
  auto lock = std::make_shared<ReadLock>(mutex_, std::defer_lock);
  if (lock->try_lock()) {
    return ConstPtr(lock, ptr_);
  }
  return nullptr;
}

template <typename T, typename Mutex, typename WriteLock, typename ReadLock>
typename lockable_ptr<T, Mutex, WriteLock, ReadLock>::Ptr
lockable_ptr<T, Mutex, WriteLock, ReadLock>::try_lock() {
  auto lock = std::make_shared<WriteLock>(mutex_, std::defer_lock);
  if (lock->try_lock()) {
    return Ptr(lock, ptr_);
  }
  return nullptr;
}

template <typename T, typename Mutex, typename Lock, typename ReadLock>
typename lockable_ptr<T, Mutex, Lock, ReadLock>::ConstPtr
lockable_ptr<T, Mutex, Lock, ReadLock>::try_read_lock() const {
  const auto& obj = this;
  return obj.try_lock();
}

}  // namespace franka_ros_controllers