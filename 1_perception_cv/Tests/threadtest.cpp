#include "Workplace.hpp"
#include <thread>
#include <iostream>

using namespace std;

static int counter = 0;

//input type: int, output type: nothing
class toolA : public WP_Tool
{
    toolA();

    vector<Work> *process(void *dataPtr)
    {
        cout << *(int *)dataPtr;
        return NULL;
    };
};

class toolP : public WP_Tool
{
    toolP();

    //input type: nothing, this is a producer
    vector<Work> *process(void *dataPtr)
    {
        cout << *(int *)dataPtr;
    };
};

int main()
{
    while (1)
        ;
}