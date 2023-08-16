#include "singleton.hpp"

class TestSingleton : public Singleton<TestSingleton> {

	friend class Singleton<TestSingleton>;

    public:
        int value() { return data; }

    protected:
        TestSingleton() : data(123) {}

    private:
        int data;
};