#encoding=utf-8

class DiscreteGridUtils:
    def __init__(self,grid_size = 0.3):
        self.grid_size = grid_size
    def continuous_to_discrete(self,pos):  # pos:x,y,z
        #print (((pos[0] - (self.grid_size * 0.5)) / self.grid_size),
        # ((pos[1] - (self.grid_size * 0.5)) / self.grid_size),
        # ((pos[2] - (self.grid_size * 0.5)) / self.grid_size))
        return (  int((pos[0] + (self.grid_size*0.5))/self.grid_size)-1,
                  int((pos[1] + (self.grid_size * 0.5)) / self.grid_size)-1,
                  int((pos[2] + (self.grid_size * 0.5)) / self.grid_size)-1)
    def discrete_to_continuous_target(self,grid_pos):  # x,y,z the center of input grid
        return ( (grid_pos[0]+0.5)*self.grid_size,
                 (grid_pos[1]+0.5)*self.grid_size,
                 (grid_pos[2]+0.5)*self.grid_size)


if __name__ == '__main__':
    dg = DiscreteGridUtils(grid_size=0.5)
    print ('res1:',dg.continuous_to_discrete((0.4,0.4,0.4) )) # 0,0,0
    print('res2:',dg.continuous_to_discrete((-0.4, -0.4, -0.4)))  # 0,0,0
    print (dg.discrete_to_continuous_target((1,1,1))  )# 0.5,0.5,0.5
    print(dg.discrete_to_continuous_target((0, 0, 0)))  # 0.5,0.5,0.5