#include "neuronbot2_bringup/data_holder.h"
#if 0
#include "board.h"

void Data_holder::load_parameter(){
    Board::get()->get_config((unsigned char*)&parameter, sizeof(parameter));
}

void Data_holder::save_parameter(){
    Board::get()->set_config((unsigned char*)&parameter, sizeof(parameter));
}
#endif
    

