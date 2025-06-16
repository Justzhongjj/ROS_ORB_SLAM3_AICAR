//
// Created by lsg on 2024/3/22.
//


#ifndef AICAR_MOVE_H
#define AICAR_MOVE_H

#include "can.h"


////////Stop/////////////

void Move_Stop(VCI_CAN_OBJ stop);


///////Forward/////////////////

void Forward(VCI_CAN_OBJ Forward);

///////Back/////////////////

void Back(VCI_CAN_OBJ Back);


///////Left/////////////////

void Left(VCI_CAN_OBJ Left);

///////Right/////////////////

void Right(VCI_CAN_OBJ Right);


#endif //AICAR_MOVE_H
