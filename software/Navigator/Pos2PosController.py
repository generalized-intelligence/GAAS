#encoding=utf-8


def bangbang(error): # on 1 axis:
    if error>THRES:
        move(-error)
    else:
        pass
    if error<-THRES:
        move(error)
    else:
        pass

def pid():
    pass

def

class Pos2PosController:
    def __init__(self):
        pass
    def main_thread(self):
        self.init_mavros()
        self.main_loop()


    def init_mavros(self):
        while self.mavros_state == "OFFBOARD":

    def set_mavros_mode(self, mode):
        self.mavros_mode

    def get


    def main_loop(self):
        while True:
            #PID controller or bang-bang controller
            get_target() # continuous x,y,z,yaw
            get_current_pos()
            err  = xxx
            do_action()
            sleep(0.1)

    def mav_move(self, position_x, position_y, position_z, relative_yaw=0):

        self.set_status(status.GOING_TO_TARGET)
        new_command = Command()
        new_command.header.stamp = rospy.Time.now()

        # use body frame
        new_command.sub_mode = 0

        # use command = MOVE according to AMO lab
        new_command.command = 6

        new_command.pos_sp[0] = position_x
        new_command.pos_sp[1] = position_y
        new_command.pos_sp[2] = position_z
        new_command.vel_sp[0] = 0.0
        new_command.vel_sp[1] = 0.0
        new_command.yaw_sp = relative_yaw  # TODO:fix this with 2step: 1:move;2.rotate(in absolute mode)
        new_command.comid = self.cur_command_id

        # self.task_id = new_command.comid

        self.prev_command_id = self.cur_command_id
        self.cur_command_id = self.cur_command_id + 1

        self.mavros_control_pub.publish(new_command)

        if self.reachTargetPosition(new_command):
            return True
        else:
            return False