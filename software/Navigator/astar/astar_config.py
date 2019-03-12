#encoding=utf-8
def manhattan(p,p_end):
    return 10*(abs(p[0]-p_end[0])+abs(p[1]-p_end[1]+abs(p[2]-p_end[2])))

astar_config={
    'movement_list': [(1,0,0),(-1,0,0),(0,1,0),(0,-1,0),(0,0,1),(0,0,-1)],
    'func_h': manhattan,
    'z_move_cost': 2
}

