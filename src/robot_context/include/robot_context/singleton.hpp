#ifndef SINGLETON_HPP
#define SINGLETON_HPP

template<typename T> class Singleton {
  public:
    static T& instance(int val)
    {
        static T the_single_instance;
        return the_single_instance;
    }

    virtual ~Singleton () {}
};

#endif