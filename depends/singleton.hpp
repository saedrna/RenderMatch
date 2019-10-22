/*!
 * \file singleton.hpp
 *
 * \author Han
 * \date 2017/03/25
 *
 *
 */
#pragma once

#include <string>

using namespace std;

template<typename T>
class Singleton {
  public:
    static bool isBuilt();
    static T *instance();
    static void reset();
  protected:
    static T *_instance;
    static void buildInstance();

    Singleton() {}
    ~Singleton() {}
};

template<typename T>
T *Singleton<T>::_instance = 0;

template<typename T>
void Singleton<T>::buildInstance() {
    if ( _instance )
        throw "Singleton already constructed";
    _instance = new T();
}

template<typename T>
T *Singleton<T>::instance() {
    if ( ! isBuilt() )
        buildInstance();
    return _instance;
}

template<typename T>
inline void Singleton<T>::reset() {
    if (_instance) {
        delete _instance;
        _instance = NULL;
    }
}

template<typename T>
bool Singleton<T>::isBuilt() {
    return (_instance != 0);
}

