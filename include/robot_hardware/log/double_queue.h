
#ifndef _UTILS_DBQUEUE_H_
#define _UTILS_DBQUEUE_H_

#include <queue>
#include <mutex>

namespace comwise {
namespace log {

/**
 * Double buffered, threadsafe queue for MPSC (multi-producer, single-consumer) comms.
 */
template<class T>
class double_queue {

public:
   double_queue():
      foreground_queue_(&queue_alpha_),
      background_queue_(&queue_beta_)
   {}

   //! Clears foreground queue and swaps queues.
   void swap()
   {
      std::unique_lock<std::mutex> fgGuard(foreground_mtx_);
      std::unique_lock<std::mutex> bgGuard(background_mtx_);

      // Clear the foreground queue.
      std::queue<T>().swap(*foreground_queue_);

      auto* swap       = background_queue_;
      background_queue_ = foreground_queue_;
      foreground_queue_ = swap;
   }

   //! Pushes to the background queue.
   void push(const T& item) 
   {
      std::unique_lock<std::mutex> guard(background_mtx_);
      background_queue_->push(item);
   }

   //! Returns a reference to the front element
   //! in the foregrund queue.
   T& front()
   {
      std::unique_lock<std::mutex> guard(foreground_mtx_);
      return foreground_queue_->front();
   }

   const T& front() const
   {
      std::unique_lock<std::mutex> guard(foreground_mtx_);
      return foreground_queue_->front();
   }

   //! Pops from the foreground queue.
   void pop() 
   {
      std::unique_lock<std::mutex> guard(foreground_mtx_);
      foreground_queue_->pop();
   }

   //! Reports whether the foreground queue is empty.
   bool empty() const
   {
      std::unique_lock<std::mutex> guard(foreground_mtx_);
      return foreground_queue_->empty();
   }

   //! Reports the size of the foreground queue.
   size_t size() const
   {
      std::unique_lock<std::mutex> guard(foreground_mtx_);
      return foreground_queue_->size();
   }

   //! Clears foreground and background.
   void clear()
   {
      std::unique_lock<std::mutex> fgGuard(foreground_mtx_);
      std::unique_lock<std::mutex> bgGuard(background_mtx_);
      std::queue<T>().swap(*foreground_queue_);
      std::queue<T>().swap(*background_queue_);
   }

private:
   // Underlying queues
   std::queue<T> queue_alpha_;
   std::queue<T> queue_beta_;

   // Front and background queue references (double buffering)
   std::queue<T>* foreground_queue_;
   std::queue<T>* background_queue_;

   mutable std::mutex foreground_mtx_;
   mutable std::mutex background_mtx_;
};

} // namespace log
} // namespace comwise

#endif
