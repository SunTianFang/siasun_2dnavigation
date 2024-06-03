#include "common.h"

#include "DebugLog.h"

#include <pthread.h>

template<class T>
class CircularBuffer
{

public:
	CircularBuffer();
	~CircularBuffer();

	void push(const T&obj);
	int pop(T* obj);
	int pop_timed_wait(T* obj, unsigned int ms);
	unsigned int get_count();
	unsigned int get_over_flow_count();
	static void cleanup(void *arg);

private:
	T buffer[2048];
	unsigned int buffer_size;
	unsigned int count;
	int front;
	int tail;
	unsigned int overflow_count;
		
	pthread_mutex_t op_lock;
	pthread_cond_t push_event;
};

template <class T> CircularBuffer<T>::CircularBuffer()
:buffer_size(2048)
,count(0)
,front(0)
,tail(0)
,overflow_count(0)
,op_lock(PTHREAD_MUTEX_INITIALIZER)
,push_event(PTHREAD_COND_INITIALIZER)
{
}

template <class T> unsigned int CircularBuffer<T>::get_count()
{
	return count;
}

template <class T> unsigned int CircularBuffer<T>::get_over_flow_count()
{
	return overflow_count;
}

template <class T> CircularBuffer<T>::~CircularBuffer()
{
	pthread_cond_destroy(&push_event);
	pthread_mutex_destroy ( &op_lock );
}

template <class T> void CircularBuffer<T>::cleanup(void *arg)
{
	CircularBuffer<T> *This = (CircularBuffer<T> *)arg;
    pthread_mutex_unlock(&(This->op_lock));
}

template <class T> void CircularBuffer<T>::push(const T&obj)
{
	pthread_mutex_lock(&op_lock);
	
	if(count >= buffer_size)
	{
		//printf("buffer overwrite count %d\n", ++overflow_count);
		tail++;
		if(tail == buffer_size)
			tail = 0;
		overflow_count++;
	
		//TODO: For Android debug purpose. To be removed.
		#if defined(ANDROID)
        VCIL_LOG_E("buffer overwrite!   count %d\n", overflow_count);
		#endif
	}
	else
		count++;

	memcpy ( &buffer[ front ], &obj, sizeof (T) );
	
	front++;
	if(front == buffer_size)
		front = 0;

	pthread_cond_signal(&push_event);
	pthread_mutex_unlock(&op_lock);
}

template <class T> int CircularBuffer<T>::pop(T* obj)
{
	int ret = 0;
	pthread_cleanup_push(&(CircularBuffer<T>::cleanup), this);
	
	pthread_mutex_lock ( &op_lock );
	do
	{
		while ( count == 0 )
		{
			pthread_cond_wait(&push_event, &op_lock);
		}
	
		memcpy ( obj, &buffer[tail], sizeof (T) );

		tail++;	
		if ( tail ==  buffer_size )
			tail = 0;
		count--;
	
	} while(false) ;	
	pthread_mutex_unlock ( &op_lock );
	
	pthread_cleanup_pop(0);
	return ret;
}

template <class T> int CircularBuffer<T>::pop_timed_wait(T* obj, unsigned int ms)
{
	int res = 0;
	int ret = 0;
	
	pthread_cleanup_push(&(CircularBuffer<T>::cleanup), this);
	pthread_mutex_lock ( &op_lock );
	do
	{	
		if ( count == 0 )
		{
			struct timespec abs_time;
	
			clock_gettime(CLOCK_REALTIME , &abs_time);    
			abs_time.tv_sec += ms/1000;
			abs_time.tv_nsec += (ms%1000)*1000000;

			if(abs_time.tv_nsec >= 1000000000)
			{
				abs_time.tv_sec++;
				abs_time.tv_nsec-=1000000000;
			}
		
			res= pthread_cond_timedwait(&push_event, &op_lock, &abs_time);
			// check timeout or inter error
			if( res != 0)
			{
				ret = -1;
				break;
			}
		}
		memcpy ( obj, &buffer[tail], sizeof (T) );

		tail++;	
		if ( tail ==  buffer_size )
			tail = 0;
		count--;
	} while (false) ;
	pthread_mutex_unlock ( &op_lock );
	pthread_cleanup_pop(0);

	return ret;
}
