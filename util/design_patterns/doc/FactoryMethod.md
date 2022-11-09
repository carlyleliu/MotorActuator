
## 简单工厂
有一家生产处理器核的厂家，它只有一个工厂，能够生产两种型号的处理器核。客户需要什么样的处理器核，一定要显示地告诉生产工厂。下面给出一种实现方案。

```c++
enum CTYPE {COREA, COREB};    

class SingleCore    
{    
  public:    
    virtual void Show() = 0;  
};    

//单核A    
class SingleCoreA: public SingleCore    
{    
  public:    
    void Show() { cout<<"SingleCore A"<<endl; }    
};    

//单核B    
class SingleCoreB: public SingleCore    
{    
  public:    
    void Show() { cout<<"SingleCore B"<<endl; }    
};  

//唯一的工厂，可以生产两种型号的处理器核，在内部判断    
class Factory    
{    
public:     
    SingleCore* CreateSingleCore(enum CTYPE ctype)    
    {    
        if(ctype == COREA)              //工厂内部判断    
            return new SingleCoreA();   //生产核A    
        else if(ctype == COREB)    
            return new SingleCoreB();   //生产核B    
        else    
            return NULL;    
    }    
};   
```

## 工厂方法
简单工厂的缺点：当要增加新的核类型时，就需要修改工厂类。这就违反了开放封闭原则：软件实体（类、模块、函数）可以扩展，但是不可修改。于是，工厂方法模式出现了。所谓工厂方法模式，是指定义一个用于创建对象的接口，让子类决定实例化哪一个类。Factory Method使一个类的实例化延迟到其子类。

```c++
class SingleCore    
{    
  public:    
    virtual void Show() = 0;  
};   

//单核A    
class SingleCoreA: public SingleCore    
{    
  public:    
    void Show() { cout<<"SingleCore A"<<endl; }    
};    

//单核B    
class SingleCoreB: public SingleCore    
{    
  public:    
    void Show() { cout<<"SingleCore B"<<endl; }    
};    

class Factory    
{    
  public:    
    virtual SingleCore* CreateSingleCore() = 0;  
};   

//生产A核的工厂    
class FactoryA: public Factory    
{    
  public:    
    SingleCoreA* CreateSingleCore() { return new SingleCoreA; }    
};  

//生产B核的工厂    
class FactoryB: public Factory    
{    
  public:    
    SingleCoreB* CreateSingleCore() { return new SingleCoreB; }    
};    
```

工厂方法模式也有缺点，每增加一种产品，就需要增加一个对象的工厂。如果这家公司发展迅速，推出了很多新的处理器核，那么就要开设相应的新工厂。在C++实现中，就是要定义一个个的工厂类。显然，相比简单工厂模式，工厂方法模式需要更多的类定义。

## 抽象工厂
还是举这个例子，这家公司的技术不断进步，不仅可以生产单核处理器，也能生产多核处理器。它的定义为提供一个创建一系列相关或相互依赖对象的接口，而无需指定它们具体的类。具体这样应用，这家公司还是开设两个工厂，一个专门用来生产A型号的单核多核处理器，而另一个工厂专门用来生产B型号的单核多核处理器，下面给出实现的代码。

```c++

//单核    
class SingleCore     
{    
  public:    
    virtual void Show() = 0;  
};  

class SingleCoreA: public SingleCore      
{    
  public:    
    void Show() { cout<<"Single Core A"<<endl; }    
};  

class SingleCoreB :public SingleCore    
{    
  public:    
    void Show() { cout<<"Single Core B"<<endl; }    
};    

//多核    
class MultiCore      
{    
  public:    
    virtual void Show() = 0;  
};   

class MultiCoreA : public MultiCore      
{    
  public:    
    void Show() { cout<<"Multi Core A"<<endl; }    
    
};   

class MultiCoreB : public MultiCore      
{    
  public:    
    void Show() { cout<<"Multi Core B"<<endl; }    
};    

//工厂    
class CoreFactory      
{    
  public:    
    virtual SingleCore* CreateSingleCore() = 0;  
    virtual MultiCore* CreateMultiCore() = 0;  
};

//工厂A，专门用来生产A型号的处理器    
class FactoryA :public CoreFactory    
{    
public:    
    SingleCore* CreateSingleCore() { return new SingleCoreA(); }    
    MultiCore* CreateMultiCore() { return new MultiCoreA(); }    
};  

//工厂B，专门用来生产B型号的处理器    
class FactoryB : public CoreFactory    
{    
  public:    
    SingleCore* CreateSingleCore() { return new SingleCoreB(); }    
    MultiCore* CreateMultiCore() { return new MultiCoreB(); }    
}; 
```
