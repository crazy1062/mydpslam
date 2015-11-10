#ifndef _SINGLETON_H_
#define _SINGLETON_H_

#include <cstddef>

template<class T>
class singleton{
public:
	static T* instance();
	static void destroy();
private:
	singleton();
	~singleton();
	singleton(singleton const&);
	singleton& operator=(singleton const&);
	static T* _pInstance;
};

template<class T>
T* singleton<T>::instance(){
	return _pInstance ? _pInstance : _pInstance = new T;
}

template<class T>
void singleton<T>::destroy(){
	delete _pInstance;
	_pInstance = NULL;
}

template<class T> T* singleton<T>::_pInstance = NULL;

#endif
