
## 策略模式
策略模式是指定义一系列的算法，把它们一个个封装起来，并且使它们可相互替换。本模式使得算法可独立于使用它的客户而变化。也就是说这些算法所完成的功能一样，对外的接口一样，只是各自实现上存在差异。用策略模式来封装算法，效果比较好。下面以高速缓存（Cache）的替换算法为例，实现策略模式。

ReplaceAlgorithm是一个抽象类，定义了算法的接口，有三个类继承自这个抽象类，也就是具体的算法实现。Cache类中需要使用替换算法，因此维护了一个 ReplaceAlgorithm的对象。

```c++
//抽象接口  
class ReplaceAlgorithm  
{  
  public:  
    virtual void Replace() = 0;  
};  

//三种具体的替换算法  
class LRU_ReplaceAlgorithm : public ReplaceAlgorithm  
{  
  public:  
    void Replace() { cout<<"Least Recently Used replace algorithm"<<endl; }  
};  
  
class FIFO_ReplaceAlgorithm : public ReplaceAlgorithm  
{  
  public:  
    void Replace() { cout<<"First in First out replace algorithm"<<endl; }  
};  

class Random_ReplaceAlgorithm: public ReplaceAlgorithm  
{  
  public:  
    void Replace() { cout<<"Random replace algorithm"<<endl; }  
};  
```

接着给出Cache的定义，这里很关键，Cache的实现方式直接影响了客户的使用方式，其关键在于如何指定替换算法。


### 1、直接通过参数指定，传入一个特定算法的指针

```c++
//Cache需要用到替换算法  
class Cache  
{  
  private:  
    ReplaceAlgorithm *m_ra;  
  public:  
    Cache(ReplaceAlgorithm *ra) { m_ra = ra; }  
    ~Cache() { delete m_ra; }  
    void Replace() { m_ra->Replace(); }  
};  
```

如果用这种方式，客户就需要知道这些算法的具体定义。只能以下面这种方式使用，可以看到暴露了太多的细节。

```c++
int main()  
{  
    Cache cache(new LRU_ReplaceAlgorithm()); //暴露了算法的定义  
    cache.Replace();  
    return 0;  
}  
```

### 2、直接通过参数指定，只不过不是传入指针，而是一个标签

```c++
//Cache需要用到替换算法  
enum RA {LRU, FIFO, RANDOM}; //标签  

class Cache  
{  
  private:  
    ReplaceAlgorithm *m_ra;  
  public:  
    Cache(enum RA ra)   
    {   
        if(ra == LRU)  
            m_ra = new LRU_ReplaceAlgorithm();  
        else if(ra == FIFO)  
            m_ra = new FIFO_ReplaceAlgorithm();  
        else if(ra == RANDOM)  
            m_ra = new Random_ReplaceAlgorithm();  
        else   
            m_ra = NULL;  
    }  
    ~Cache() { delete m_ra; }  
    void Replace() { m_ra->Replace(); }  
};  
```

相比方式一，这种方式用起来方便多了。其实这种方式将简单工厂模式与策略模式结合在一起，算法的定义使用了策略模式，而Cache的定义其实使用了简单工厂模式。

```c++
int main()  
{  
    Cache cache(LRU); //指定标签即可  
    cache.Replace();  
    return 0;  
}  
```

上面两种方式，构造函数都需要形参。构造函数是否可以不用参数呢？下面给出第三种实现方式。

### 3、利用模板实现
算法通过模板的实参指定。当然了，还是使用了参数，只不过不是构造函数的参数。在策略模式中，参数的传递难以避免，客户必须指定某种算法。

```c++
//Cache需要用到替换算法  
template <class RA>  
class Cache  
{  
  private:  
    RA m_ra;  
  public:  
    Cache() { }  
    ~Cache() { }  
    void Replace() { m_ra.Replace(); }  
};  
```

使用方式如下：

```c++
int main()  
{  
    Cache<Random_ReplaceAlgorithm> cache; //模板实参  
    cache.Replace();  
    return 0;  
} 
```

