//
// Created by lsg on 2024/3/22.
//

#include "../include/can_pkg/move.h"

class Move{
public:
    VCI_CAN_OBJ MOVE;

    virtual void move(VCI_CAN_OBJ MOVE){
        VCI_CAN_OBJ send[1];
        send[0] = MOVE;
        VCI_Transmit(VCI_USBCAN2, 0, 1, send, 1);
    }
};

////////Stop/////////////

void Move_Stop(VCI_CAN_OBJ stop){
    Move move_stop;
    move_stop.move(stop);
}


///////////////////////



///////Forward/////////////////
void Forward(VCI_CAN_OBJ Forward){
    Move move_forward;
    move_forward.move(Forward);
}


///////Back/////////////////
void Back(VCI_CAN_OBJ Back){
    Move move_back;
    move_back.move(Back);
}

///////Left/////////////////
void Left(VCI_CAN_OBJ Left){
    Move move_left;
    move_left.move(Left);
}

///////Right/////////////////

void Right(VCI_CAN_OBJ Right){
    Move move_right;
    move_right.move(Right);
}
