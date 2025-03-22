import commands2
import elevate
import depo
import const

P_L1 = 1
P_L2 = 2
P_L3 = 3
P_L4 = 4
P_AL = 5
P_AH = 6

class Position:
    def __init__(self, elevator: elevate.Elevator, wrist: depo.DepositorWrist):
        self.elevator = elevator
        self.wrist = wrist
    
    def goto(self, ep: float, wp: float, ele_speed: float, wri_speed: float) -> commands2.Command:
        return commands2.ParallelCommandGroup(
            commands2.RepeatCommand(self.elevator.goto(ep, ele_speed)),
            commands2.RepeatCommand(self.wrist.goto(wp, wri_speed))
        )
    def ready(self) -> bool:
        er = self.elevator.get_distance() < 0.03
        wr = self.wrist.at_setpoint() < 2
        return er and wr
    
    def outofway(self, ele_speed=0.4, wri_speed=0.8) -> commands2.Command:
        return self.goto(0.2, 180, ele_speed=ele_speed, wri_speed=wri_speed)
    
    def stow(self, ele_speed=0.4, wri_speed=0.8) -> commands2.Command:
        return self.goto(0, 90, ele_speed=ele_speed, wri_speed=wri_speed)

    def acquire(self, ele_speed=0.4, wri_speed=0.8) -> commands2.Command:
        return self.goto(0, -5, ele_speed=ele_speed, wri_speed=wri_speed)
    
    # L4ities
    def scan(self, ele_speed=0.4, wri_speed=0.8) -> commands2.Command:
        return self.goto(0.32, 138, ele_speed=ele_speed, wri_speed=wri_speed)
    def tall(self, ele_speed=0.4, wri_speed=0.8) -> commands2.Command:
        return self.goto(const.l4_ext, 120, ele_speed=ele_speed, wri_speed=wri_speed)
    

    def reef_position(self, pos: int, ele_speed=0.4, wri_speed=0.8) -> commands2.Command:
        if pos == P_AL: return self.goto(const.al_ext, const.l_wrist_algae_angle, ele_speed=ele_speed, wri_speed=wri_speed)
        if pos == P_AH: return self.goto(const.ah_ext, const.l_wrist_algae_angle, ele_speed=ele_speed, wri_speed=wri_speed)
        if pos == P_L1: return self.goto(const.l1_ext, const.l_wrist_angle, ele_speed=ele_speed, wri_speed=wri_speed)
        if pos == P_L2: return self.goto(const.l2_ext, const.l_wrist_angle, ele_speed=ele_speed, wri_speed=wri_speed)
        if pos == P_L3: return self.goto(const.l3_ext, const.l_wrist_angle, ele_speed=ele_speed, wri_speed=wri_speed)
        if pos == P_L4: return self.goto(const.l4_ext, const.l_wrist_angle, ele_speed=ele_speed, wri_speed=wri_speed)