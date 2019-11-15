#ifndef PIBOT_VARIABLE_QUEUE_H_
#define PIBOT_VARIABLE_QUEUE_H_

#include<string.h>
#include <stdio.h>
template <unsigned short MAX_SIZE=256>
class VQueue : public Queue{
  public:
    VQueue();
    bool put(unsigned char ch);
    bool get(unsigned char& ch);

    unsigned short size();
    unsigned short max_size();
  private:
    unsigned char _buffer[MAX_SIZE];
    unsigned short _max_size;
    unsigned short _head;
    unsigned short _tail;
};

template<unsigned short MAX_SIZE>
VQueue<MAX_SIZE>::VQueue(){
    _max_size = MAX_SIZE;
    memset(_buffer, 0, MAX_SIZE);
}

template<unsigned short MAX_SIZE>
unsigned short VQueue<MAX_SIZE>::size(){
    return (_tail+_max_size-_head)%_max_size;
}

template<unsigned short MAX_SIZE>
unsigned short VQueue<MAX_SIZE>::max_size(){
    return _max_size;
}

template<unsigned short MAX_SIZE>
bool VQueue<MAX_SIZE>::put(unsigned char ch){
    if (_tail+1 == _head)
        return false;

    _buffer[_tail++] = ch;

    if (_tail >= MAX_SIZE)
        _tail = 0;

    return true;
}

template<unsigned short MAX_SIZE>
bool VQueue<MAX_SIZE>::get(unsigned char& ch){
    if (_head == _tail)
        return false;

    ch = _buffer[_head++];

    if (_head >= MAX_SIZE)
        _head = 0;

    return true;
}


#endif