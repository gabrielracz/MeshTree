#include <iostream>
#include <exception>
#include <vector>

#include "path_config.h"
#include "application.h"


template <typename T>
std::vector<T> concatenate(std::vector<T>& A, std::vector<T>& B) {
    std::vector<T> AB;
    AB.reserve( A.size() + B.size() ); // preallocate memory
    AB.insert( AB.end(), A.begin(), A.end() );
    AB.insert( AB.end(), B.begin(), B.end() );
    return AB;
}

int main(int argc, char** argv){
    Application app;
    std::string objfile1 = argc > 1 ? std::string(argv[1]) : RESOURCES_DIRECTORY"/bunny_full.obj";
    std::string objfile2 = argc > 2 ? std::string(argv[2]) : RESOURCES_DIRECTORY"/dragon.obj";
    std::string objfile3 = argc > 3 ? std::string(argv[3]) : RESOURCES_DIRECTORY"/lucy.obj";
    app.Init(objfile1, objfile2, objfile3);
    while(!app.Closed()) {
        app.Update();
    }
    return 0;
}