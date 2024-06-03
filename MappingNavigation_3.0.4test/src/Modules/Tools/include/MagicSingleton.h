//
//   The interface of class "MagicSingleton".
//

#pragma once

#include <mutex>
#include <memory>

//using namespace std;

//////////////////////////////////////////////////////////////////////////////
// C++11 thread safe
template <typename T>
class MagicSingleton
{
public:
    static T* GetInstance()
    {
        static T t;
        return &t;
    }

private:
    MagicSingleton() {}
    ~MagicSingleton() {}
};


template <typename T>
class MagicSingleton2
{
public:
    static T* GetInstance() {
        if (pSingle == nullptr) {
            std::lock_guard<std::mutex> lock(mtx);
            if (pSingle == nullptr) {
                pSingle = new T();
            }
        }
        return pSingle;
    }

    static void DesInstance() {
        if(pSingle) {
            delete pSingle;
            pSingle = nullptr;
        }
    }

private:
    MagicSingleton2() {}
    static T* pSingle;
    static std::mutex mtx;
};

template <typename T>
std::mutex MagicSingleton2<T>::mtx;
template <typename T>
T* MagicSingleton2<T>::pSingle = nullptr;


template<typename T>
class MagicSingleton3
{
public:
    //获取全局单例对象
    template<typename ...Args>
    static std::shared_ptr<T> GetInstance(Args&&... args) {
        if (!m_pSington) {
            std::lock_guard<std::mutex> gLock(m_Mutex);
            if (nullptr == m_pSington) {
                m_pSington = std::make_shared<T>(std::forward<Args>(args)...);
            }
        }
        return m_pSington;
    }

    //主动析构单例对象（一般不需要主动析构，除非特殊需求）
    static void DesInstance() {
        if (m_pSington) {
            m_pSington.reset();
            m_pSington = nullptr;
        }
    }

private:
    explicit MagicSingleton3();
    MagicSingleton3(const MagicSingleton3&) = delete;
    MagicSingleton3& operator=(const MagicSingleton3&) = delete;
    ~MagicSingleton3();

private:
    static std::shared_ptr<T> m_pSington;
    static std::mutex m_Mutex;
};

template<typename T>
std::shared_ptr<T> MagicSingleton3<T>::m_pSington = nullptr;

template<typename T>
std::mutex MagicSingleton3<T>::m_Mutex;

//auto pClass = MagicSingleton3<MyClass>::GetInstance("create");
//MagicSingleton3<MyClass>::DesInstance();

