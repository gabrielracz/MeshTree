#include <iostream>
#include <exception>
#include <vector>

#include "application.h"


template <typename T>
std::vector<T> concatenate(std::vector<T>& A, std::vector<T>& B) {
    std::vector<T> AB;
    AB.reserve( A.size() + B.size() ); // preallocate memory
    AB.insert( AB.end(), A.begin(), A.end() );
    AB.insert( AB.end(), B.begin(), B.end() );
    return AB;
}

int main(void){
    Application app;
    app.Init();
    while(!app.Closed()) {
        app.Update();
    }
    return 0;
}