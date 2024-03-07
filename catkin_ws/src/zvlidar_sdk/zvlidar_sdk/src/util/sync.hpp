
#include <functional>
#include <queue>
#include <mutex>
#include <condition_variable>

namespace zv_processor{

 template <typename T>
    class SynchronizedQueue
    {
    public:
        SynchronizedQueue() :
            queue_(), 
            mutex_(), 
            cond_(), 
            request_to_end_(false), 
            enqueue_data_(true)
        {
        }

        bool enqueue(const T& data)
        {
            std::unique_lock<std::mutex> lock(mutex_);

            if (enqueue_data_)
            {
                queue_.push(data);
                cond_.notify_one();
                return true;
            }
            else
            {
                return false;
            }
        }

        bool dequeue(T& result)
        {
            std::unique_lock<std::mutex> lock(mutex_);

            while (queue_.empty() && (!request_to_end_))
            {
                cond_.wait(lock);
            }

            if (request_to_end_)
            {
                doEndActions();
                return false;
            }

            result = queue_.front();
            queue_.pop();

            return true;
        }

        void stopQueue()
        {
            std::unique_lock<std::mutex> lock(mutex_);
            request_to_end_ = true;
            cond_.notify_one();
        }

        unsigned int size()
        {
            std::unique_lock<std::mutex> lock(mutex_);
            return static_cast<unsigned int>(queue_.size());
        }

        bool isEmpty() const
        {
            std::unique_lock<std::mutex> lock(mutex_);
            return (queue_.empty());
        }

    private:
        void doEndActions()
        {
            enqueue_data_ = false;
            while (!queue_.empty())
            {
                queue_.pop();
            }
        }

        std::queue<T> queue_;
        mutable std::mutex mutex_;
        std::condition_variable cond_;

        bool request_to_end_;
        bool enqueue_data_;
    };

}


