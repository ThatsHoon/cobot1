from rclpy.node import Node

class RoboChef(Node):
    GRIPPER_ON = 1
    GRIPPER_OFF = 0
    
    def __init__(self, vel, acc, namespace):
        super().__init__("RoboChef", namespace=namespace)
               
        self._vel = vel
        self._acc = acc
        
        # 모드별 실행 함수를 딕셔너리로 관리
        self._modes = {
            'reset': self.reset,
            'horizontal': self._move_horizontal,
            'vertical': self._move_vertical,
            'periodic': self._move_periodic,
            'gripper': self._gripper
        }
    
    # Getter: 멤버변수 읽을 때 호출
    @property
    def vel(self):
        return self._vel
    
    @property
    def acc(self):
        return self._acc
    
    # Setter: 멤버변수 수정할 때 호출
    @vel.setter
    def vel(self, value):
        self._vel = value
    
    @acc.setter
    def acc(self, value):
        self._acc = value
    
    def _flatten_locations(self, locations_dict):
        """중첩된 locations 구조를 단일 레벨 딕셔너리로 변환"""
        flat = {}
        for category in locations_dict.values():
            for name, data in category.items():
                flat[name] = data['coord']
        return flat
            
    # 통합 동작 함수(모드에 따라 다른 함수 실행)    
    def move(self, mode, **kwargs):
        """
        모든 동작을 하나의 함수로 통합 실행.
        사용 예:
        1. move(mode='reset')
        2. move(pos=posx_val, mode='linear')
        3. move(mode='periodic', amp=[10,10,10,0,0,0], period=2.0, atime=0.5, repeat=1)
        """
        
        func = self._modes.get(mode.lower())
        if not func:
            raise ValueError(f"잘못된 동작 모드: {mode}")
    
        try:
            # 가변인자를 넘겨서 해당 모드의 함수 실행
            return func(**kwargs)
        except TypeError as e:
            raise ValueError(f"{mode} 인자 에러")   
    
    #==================== 동작 함수들 =========================#
    def reset(self):
        from DSR_ROBOT2 import posj
        from DSR_ROBOT2 import movej
        
        movej(posj(0, 0, 90, 0, 90, 0), vel=self.vel, acc=self.acc)
        self._gripper(gripper_mode='open')
        
    # 수평 이동(미완성) 
    def _move_horizontal(self, pos):
        from DSR_ROBOT2 import posx,posj
        from DSR_ROBOT2 import movej,movel
        
        if isinstance(pos, posx):
            movel(pos, vel=self.vel, acc=self.acc)
            print("movel: ", pos)
        elif isinstance(pos, posj): 
            movej(pos, vel=self.vel, acc=self.acc)
            print("movej")
            
        else:
            raise TypeError("Unknown Pos Type!")
    
    # 수직 이동
    def _move_vertical(self, z):
        from DSR_ROBOT2 import posx, movel, DR_MV_MOD_REL
                
        rel_pos = posx(0, 0, z, 0, 0, 0)
        movel(rel_pos, vel=self.vel, acc=self.acc, mod=DR_MV_MOD_REL)
        
    # 주기 운동
    def _move_periodic(self, amp, period, atime, repeat):
        from DSR_ROBOT2 import move_periodic
        move_periodic(amp=amp, period=period, atime=atime, repeat=repeat)

    # 그리퍼 동작
    def _gripper(self, gripper_mode):
        if gripper_mode not in ['open', 'close']:
            raise ValueError(f"잘못된 그리퍼 모드: {gripper_mode}")
        
        from DSR_ROBOT2 import set_digital_output, wait
        if gripper_mode == 'open':
            print("Gripper Open")
            set_digital_output(1, self.GRIPPER_OFF)
            set_digital_output(2, self.GRIPPER_ON)
            #wait(2)
        
        if gripper_mode == 'close':
            print("Gripper Close")
            set_digital_output(1, self.GRIPPER_ON)
            set_digital_output(2, self.GRIPPER_OFF)
            #wait(2)
