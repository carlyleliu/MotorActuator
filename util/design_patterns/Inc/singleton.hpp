#ifndef __DESIGN_PATTERNS_SINGLETON_HPP__
#define __DESIGN_PATTERNS_SINGLETON_HPP__

namespace DesignedPatterns {

/* Safe after c++11 */
template <typename T>
class Singleton
{
  private:
	Singleton() {}
	~Singleton() {}
	Singleton(const Singleton&);
	Singleton& operator=(const Singleton&);
  public:
	static T& GetInstance() {
		static T instance;
		return instance;
	}
};

}

#endif // ! __DESIGN_PATTERNS_SINGLETON_HPP__
