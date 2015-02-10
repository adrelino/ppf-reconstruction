#include "Params.h"
#include "math.h"

using namespace std;

Params* Params::instance = 0;


Params* Params::getInstance(){
    if(instance == 0){
        instance = new Params();
    }
    return instance;
}
