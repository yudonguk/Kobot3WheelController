#ifndef __SIMPLE_MUTEX_H__
#define __SIMPLE_MUTEX_H__

#if defined(_WIN32)
#include <windows.h>
#else
#include <pthread.h>
#endif

class ScopedLock;

class SimpleMutex
{
public:
	typedef ScopedLock ScopedLock;

public:
	SimpleMutex()
	{
#if defined(_WIN32)
		InitializeCriticalSection(&criticalSection);
#else
		pthread_mutex_init(&mutex, NULL);
#endif
	}

	~SimpleMutex()
	{
#if defined(_WIN32)
		DeleteCriticalSection(&criticalSection);
#else
		pthread_mutex_destroy(&mutex);
#endif
	}

protected:
	SimpleMutex(const SimpleMutex& simpleMutex);
	SimpleMutex& operator=(const SimpleMutex& simpleMutex);

public:
	void Lock()
	{
#if defined(_WIN32)
		EnterCriticalSection(&criticalSection);
#else
		pthread_mutex_lock(&mutex);
#endif
	}

	void Unlock()
	{
#if defined(_WIN32)
		LeaveCriticalSection(&criticalSection);
#else
		pthread_mutex_unlock(&mutex);
#endif
	}

private:
#if defined(_WIN32)
	CRITICAL_SECTION criticalSection;
#else
	pthread_mutex_t mutex;
#endif
};

class ScopedLock
{
public:
	ScopedLock(SimpleMutex& mutex)
		: mpMutex(&mutex)
	{
		Lock();
	}

	ScopedLock(SimpleMutex* pMutex)
		: mpMutex(pMutex)
	{
		Lock();
	}

	~ScopedLock()
	{
		Unlock();
	}

private:
	ScopedLock(const ScopedLock&);
	ScopedLock& operator=(const ScopedLock&);

public:
	void Lock()
	{
		if(mpMutex != NULL)
			mpMutex->Lock();
	}

	void Unlock()
	{
		if (mpMutex != NULL)
			mpMutex->Unlock();
	}

private:
	SimpleMutex* mpMutex;
};

class ScopedReaderLock;
class ScopedWriterLock;

class SimpleReaderWriterLock
{
public:
	typedef ScopedReaderLock ScopedReaderLock;
	typedef ScopedWriterLock ScopedWriterLock;

public:
	class ScopedLock
	{
	public:
		ScopedLock(SimpleMutex& mutex)
			: mpMutex(&mutex)
		{
			Lock();
		}

		ScopedLock(SimpleMutex* pMutex)
			: mpMutex(pMutex)
		{
			Lock();
		}

		~ScopedLock()
		{
			Unlock();
		}

	private:
		ScopedLock(const ScopedLock&);
		ScopedLock& operator=(const ScopedLock&);

	public:
		void Lock()
		{
			if(mpMutex != NULL)
				mpMutex->Lock();
		}

		void Unlock()
		{
			if (mpMutex != NULL)
				mpMutex->Unlock();
		}

	private:
		SimpleMutex* mpMutex;
	};
public:
	SimpleReaderWriterLock()
	{
#if defined(_WIN32)
		readerCount = 0;
		InitializeCriticalSection(&readerCountCriticalSection);
		InitializeCriticalSection(&writerCriticalSection);
		readerCompleteEventHandler = CreateEvent(NULL, TRUE, TRUE, NULL);
#else
		pthread_rwlock_init(&rwLock, NULL);
#endif
	}

	~SimpleReaderWriterLock()
	{
#if defined(_WIN32)	
		DeleteCriticalSection(&readerCountCriticalSection);
		DeleteCriticalSection(&writerCriticalSection);
		CloseHandle(readerCompleteEventHandler);
#else
		pthread_rwlock_destroy(&rwLock);
#endif
	}

protected:
	SimpleReaderWriterLock(const SimpleReaderWriterLock& simpleSemaphore);
	SimpleReaderWriterLock& operator=(const SimpleReaderWriterLock& simpleSemaphore);

public:
	void ReaderLock()
	{
#if defined(_WIN32)
		EnterCriticalSection(&writerCriticalSection);
		EnterCriticalSection(&readerCountCriticalSection);
		if (++readerCount == 1)
		{
			ResetEvent(readerCompleteEventHandler);
		}
		LeaveCriticalSection(&readerCountCriticalSection);
		LeaveCriticalSection(&writerCriticalSection);
#else
		pthread_rwlock_rdlock(&rwLock);
#endif
	}

	void WriterLock()
	{
#if defined(_WIN32)
		EnterCriticalSection(&writerCriticalSection);
		if (readerCount > 0)
		{
			WaitForSingleObject(readerCompleteEventHandler, INFINITE);
		}
#else
		pthread_rwlock_wrlock(&rwLock);
#endif
	}

	void ReaderUnlock()
	{
#if defined(_WIN32)
		EnterCriticalSection(&readerCountCriticalSection);
		if (--readerCount == 0)
		{
			SetEvent(readerCompleteEventHandler);
		}
		LeaveCriticalSection(&readerCountCriticalSection);
#else
		pthread_rwlock_unlock(&rwLock);
#endif
	}

	void WriterUnlock()
	{
#if defined(_WIN32)
		LeaveCriticalSection(&writerCriticalSection);
#else
		pthread_rwlock_unlock(&rwLock);
#endif
	}

private:
#if defined(_WIN32)	
	HANDLE readerCompleteEventHandler;
	CRITICAL_SECTION readerCountCriticalSection;
	CRITICAL_SECTION writerCriticalSection;
	volatile unsigned int readerCount;
#else
	pthread_rwlock_t rwLock;
#endif
};

class ScopedReaderLock
{
public:
	ScopedReaderLock(SimpleReaderWriterLock& lock)
		: mpLock(&lock)
	{
		Lock();
	}

	ScopedReaderLock(SimpleReaderWriterLock* pLock)
		: mpLock(pLock)
	{
		Lock();
	}

	~ScopedReaderLock()
	{
		Unlock();
	}

private:
	ScopedReaderLock(const ScopedReaderLock&);
	ScopedReaderLock& operator=(const ScopedReaderLock&);

public:
	void Lock()
	{
		if(mpLock != NULL)
			mpLock->ReaderLock();
	}

	void Unlock()
	{
		if (mpLock != NULL)
			mpLock->ReaderUnlock();
	}

private:
	SimpleReaderWriterLock* mpLock;
};

class ScopedWriterLock
{
public:
	ScopedWriterLock(SimpleReaderWriterLock& lock)
		: mpLock(&lock)
	{
		Lock();
	}

	ScopedWriterLock(SimpleReaderWriterLock* pLock)
		: mpLock(pLock)
	{
		Lock();
	}

	~ScopedWriterLock()
	{
		Unlock();
	}

private:
	ScopedWriterLock(const ScopedWriterLock&);
	ScopedWriterLock& operator=(const ScopedWriterLock&);

public:
	void Lock()
	{
		if(mpLock != NULL)
			mpLock->WriterLock();
	}

	void Unlock()
	{
		if (mpLock != NULL)
			mpLock->WriterUnlock();
	}

private:
	SimpleReaderWriterLock* mpLock;
};

#endif

