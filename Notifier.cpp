#include "Notifier.h"

std::unique_ptr<std::deque<std::shared_ptr<Notifier>>, std::function<void(std::deque<std::shared_ptr<Notifier>>*)>> Notifier::mpPool;
boost::shared_mutex Notifier::mPoolMutex;

Notifier::Notifier()
	: mStatus(WAIT)
{
}

Notifier::~Notifier()
{
	Reset();
}

std::shared_ptr<Notifier> Notifier::Allocate()
{
	boost::upgrade_lock<boost::shared_mutex> upgradeLock(mPoolMutex);
	if (mpPool.get() == nullptr)
	{
		boost::upgrade_to_unique_lock<boost::shared_mutex> uniqueLock(upgradeLock);
		mpPool = std::unique_ptr<std::deque<std::shared_ptr<Notifier>>
			, std::function<void(std::deque<std::shared_ptr<Notifier>>*)>>(
			new std::deque<std::shared_ptr<Notifier>>, [](std::deque<std::shared_ptr<Notifier>>* pPool)
			{
				mPoolMutex.lock();
				auto holder = std::move(*pPool);
				delete mpPool.release();	
				mPoolMutex.unlock();	
			}
		);
	}

	std::shared_ptr<Notifier> result;

	if (mpPool->empty())
	{
		result.reset(new Notifier, &Release);
	}
	else
	{
		boost::upgrade_to_unique_lock<boost::shared_mutex> uniqueLock(upgradeLock);
		result.swap(mpPool->front());
		mpPool->pop_front();
	}

	return result;
}

void Notifier::Release( Notifier* pNotifier )
{
	boost::upgrade_lock<boost::shared_mutex> upgradeLock(mPoolMutex);
	if (mpPool.get() == nullptr || mpPool->size() >= 10)
	{
		delete pNotifier;
	}
	else
	{
		pNotifier->Reset();
	
		boost::upgrade_to_unique_lock<boost::shared_mutex> uniqueLock(upgradeLock);
		mpPool->push_back(std::shared_ptr<Notifier>(pNotifier, &Release));
	}
}

bool Notifier::IsCanceled()
{
	boost::mutex::scoped_lock lock(mMutex);
	return mStatus != CANCEL;
}

void Notifier::Notify()
{
	boost::mutex::scoped_lock lock(mMutex);
	if (mStatus != WAIT)
		return;

	mStatus = COMPLETE;
	mConditionVariable.notify_all();
}

void Notifier::Cancel()
{
	boost::mutex::scoped_lock lock(mMutex);
	if (mStatus != WAIT)
		return;

	mStatus = CANCEL;
	mConditionVariable.notify_all();
}

void Notifier::Reset()
{
	boost::mutex::scoped_lock lock(mMutex);
	mStatus = WAIT;
	mConditionVariable.notify_all();
}

bool Notifier::Wait( uint32_t timeout /*= 0xFFFFFFFF*/ )
{
	boost::mutex::scoped_lock lock(mMutex);
	if (mStatus == COMPLETE)
		return true;
	else if(mStatus == CANCEL)
		return false;

	if (mConditionVariable.wait_for(lock, boost::chrono::milliseconds(timeout))
		== boost::cv_status::timeout || mStatus != COMPLETE)
		return false;
	
	return true;
}