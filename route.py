_to_first_can = [f,f,f,f,l,r,f,f,r,f,r,f]
_move_first_can1 = [f,f,f,f,d,r,f,l,f,l,f,f,d,]
_move_first_can2= [r,f,l,f,f,l,f,f,l,f,l,f,f,d]
_back_to_cans = [r,f,r,f,l,f,r,f,f,f,f]
#double turn might be a problem
_make_space = [r,f,d,l,l,f,l,f,f,l,f,l,f,d]
#again double turn
_move2_into_posistion = [r,f,f,l,f,f,l,f,f,d,r,r]

test_route = _to_first_can+_move_first_can1 + _move_first_can2 + _back_to_cans + _make_space + _move2_into_posistion