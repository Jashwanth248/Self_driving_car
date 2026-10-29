#include "autodrive/perception.hpp"
#include <iostream>
#include <exception>
int main(int argc,char** argv) {
    if(argc!=2) {std::cerr<<"Usage: autodrive_train output_weights.txt\n";return 2;}
    try {
        auto model=autodrive::RiskModel::train();model.save(argv[1]);
        std::cout<<"Trained synthetic logistic classifier; saved 4 weights to "<<argv[1]<<'\n';
        return 0;
    }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
