#ifndef __NOTIFIER_H__
#define __NOTIFIER_H__

#include <memory>
#include <cstdint>
#include <deque>
#include <functional>

#include <boost/thread/shared_mutex.hpp>
#include <boost/thread/condition_variable.hpp>

class Notifier
{
private:
	enum Status	{ WAIT, CANCEL, COMPLETE };

private:
	Notifier();

public:
	~Notifier();

private:
	// 복사 생성자 및 연산자 제거
	Notifier(const Notifier&);
	const Notifier& operator=(const Notifier&);

public:
	static std::shared_ptr<Notifier> Allocate();

public:
	bool IsCanceled();
	void Notify();
	void Cancel();
	void Reset();
	bool Wait(uint32_t timeout = 0xFFFFFFFF);

private:
	static void Release(Notifier* pNotifier);

private:
	static std::unique_ptr<std::deque<std::shared_ptr<Notifier>>, std::function<void(std::deque<std::shared_ptr<Notifier>>*)>> mpPool;
	static boost::shared_mutex mPoolMutex;

private:
	boost::mutex mMutex;
	boost::condition_variable mConditionVariable;
	Status mStatus;
};

#endif // !__NOTIFIER_H__
