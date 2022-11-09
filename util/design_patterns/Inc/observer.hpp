#ifndef __DESIGN_PATTERNS_OBSERVER_HPP__
#define __DESIGN_PATTERNS_OBSERVER_HPP__

#include <string>
#include <vector>

namespace DesignedPatterns {

class Subject;

class Observer
{
  public:
    Observer() {}
    virtual ~Observer() {}
    void SetSubjecter(Subject* subjecter)
    {
        subjecter_ = subjecter;
    }

    virtual int Update() = 0;

  protected:
    Subject* subjecter_;
};

class Subject
{
  public:
    Subject() {}
    virtual ~Subject() {}
    void Attach(Observer* observer)
    {
        observers_.push_back(observer);
    }

    void Detach(Observer* observer)
    {
        for (std::vector <Observer*> ::iterator it = observers_.begin(); it != observers_.end(); ++it) {
            if (*it == observer) {
                observers_.erase(it);
                return;
            }
        }
    }

    void Notify()
    {
        for(std::vector <Observer*> ::iterator it = observers_.begin(); it != observers_.end(); ++it) {
            (*it)->Update();
        }
    }

  protected:
    std::vector <Observer*> observers_;
};

}

#endif // ! __DESIGN_PATTERNS_OBSERVER_HPP__
