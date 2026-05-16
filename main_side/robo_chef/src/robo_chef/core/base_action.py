# 최상단에 DSR_ROBOT2를 임포트하지 말 것

class BaseAction:
    action_name = None 

    GRIPPER_ON = 1
    GRIPPER_OFF = 0
    
    def __init__(self, manager, vel_linear=200, acc_linear=50, vel_angular=70, acc_angular=70):
        self.vel_linear = vel_linear
        self.acc_linear = acc_linear
        self.vel_angular = vel_angular
        self.acc_angular= acc_angular
        
        self.manager = manager  # ActionManager 참조

    def execute(self, **kwargs):
        raise NotImplementedError
    
    # ── 저수준 동작 래핑 (기본 기능) ──
    def movel(self, pos, vel=None, acc=None, time=0, radius=0, mode='abs', ref='base'):
        v = vel if vel is not None else self.vel_linear
        a = acc if acc is not None else self.acc_linear
        
        from DSR_ROBOT2 import movel
        from DSR_ROBOT2 import DR_MV_MOD_ABS, DR_MV_MOD_REL, DR_BASE, DR_TOOL
        from DSR_ROBOT2 import posx
        
        if mode == 'abs':
            mode = DR_MV_MOD_ABS
        elif mode == 'rel':
            mode = DR_MV_MOD_REL
        else:
            print("❌ 잘못된 move 모드!")
        
        if ref == 'base':
            ref = DR_BASE
        elif ref == 'tool':
            ref = DR_TOOL
        else:
            print("❌ 잘못된 move 모드!")
            
        pos = posx(pos)
        res = movel(pos, vel=v, acc=a, time=time, radius=radius, mod=mode, ref=ref)
        
        if res != 0:
            print(f"⚠️ 예외사항 발생!: {res}")
            return False
        
        return True

    def movej(self, joint, vel=None, acc=None, time=0, mode='rel'):
        v = vel if vel is not None else self.vel_angular
        a = acc if acc is not None else self.acc_angular
        
        from DSR_ROBOT2 import movej
        from DSR_ROBOT2 import DR_MV_MOD_ABS, DR_MV_MOD_REL
        from DSR_ROBOT2 import posj
        
        if mode == 'abs':
            mode = DR_MV_MOD_ABS
        elif mode == 'rel':
            mode = DR_MV_MOD_REL
        else:
            print("❌ 잘못된 move 모드!")
            
        joint = posj(joint)
        res = movej(joint, vel=v, acc=a, time=time, mod=mode)
        
        if res != 0:
            print(f"⚠️ 예외사항 발생!: {res}")
            return False
        
        return True

    def periodic(self, amp, period, repeat):
        from DSR_ROBOT2 import move_periodic, DR_BASE
        res = move_periodic(amp=amp, period=period, repeat=repeat, ref=DR_BASE)
        
        if res != 0:
            print(f"⚠️ 예외사항 발생!: {res}")
            return False
        
        return True
    
    def gripper_open(self):
        from DSR_ROBOT2 import set_digital_output
        
        set_digital_output(1, self.GRIPPER_OFF)
        set_digital_output(2, self.GRIPPER_ON)
        set_digital_output(3, self.GRIPPER_OFF)
        
        # 그리퍼의 예외상황에 대한 set_digital_output 반환값 처리도 필요
        self.wait(2)
    
    def gripper_open_little(self):
        from DSR_ROBOT2 import set_digital_output
        
        set_digital_output(1, self.GRIPPER_OFF)
        set_digital_output(2, self.GRIPPER_OFF)
        set_digital_output(3, self.GRIPPER_ON)
        
        # 그리퍼의 예외상황에 대한 set_digital_output 반환값 처리도 필요
        self.wait(2)
        
    def gripper_close(self):
        from DSR_ROBOT2 import set_digital_output
        
        set_digital_output(1, self.GRIPPER_ON)
        set_digital_output(2, self.GRIPPER_OFF)
        set_digital_output(3, self.GRIPPER_OFF)
        
        self.wait(2)

    def compliance_on(self, stx=[500, 500, 500, 100, 100, 100], ref='tool'):
        """
        순응 제어(Compliance Control)를 시작합니다.
        :param stx: 각 축(x, y, z, rx, ry, rz)에 대한 강성(Stiffness) 값의 리스트.
                    값이 작을수록 부드럽게(스프링처럼) 움직입니다.
        """
        from DSR_ROBOT2 import task_compliance_ctrl, set_ref_coord
        from DSR_ROBOT2 import DR_BASE, DR_TOOL
        
        # 순응 제어 활성화 '전'에만 기준 좌표계를 설정해야함
        ref_val = DR_TOOL if ref == 'tool' else DR_BASE
        set_ref_coord(ref_val)
            
        res = task_compliance_ctrl(stx=stx)
            
        self.wait(1)
        
        if res != 0:
            print(f"⚠️ 컴플라이언스 제어 활성화 실패: {res}")
            return False
            
        return True
    
    def compliance_off(self):
        """순응 제어를 해제하고 원래의 강성(Rigid) 제어 상태로 복귀합니다."""
        from DSR_ROBOT2 import release_compliance_ctrl, set_ref_coord, DR_BASE
        
        res = release_compliance_ctrl()
        set_ref_coord(DR_BASE)
        
        if res != 0:
            print(f"⚠️ 컴플라이언스 제어 해제 실패: {res}")
            return False
        
        return True
    
    def set_desired_force(self, fd=[0, 0, 0, 0, 0, 0], dir=[0, 0, 0, 0, 0, 0], ref='tool', mode='rel'):
        """
        로봇이 특정 방향으로 가할 목표 힘을 설정합니다.
        :param fd: 목표 힘/토크 리스트 (N 또는 Nm)
        :param dir: 힘을 가할 방향 (1: 활성화, 0: 비활성화)
        :param mode: 힘 제어 모드 ('abs' 또는 'rel')
        """
        from DSR_ROBOT2 import set_desired_force
        from DSR_ROBOT2 import DR_FC_MOD_ABS, DR_FC_MOD_REL
        
        fc_mode = DR_FC_MOD_ABS if mode == 'abs' else DR_FC_MOD_REL
        
        # DSR 파이썬 API에 ref 키워드가 없으므로 함수 호출 시 넘기지 않습니다.
        # (compliance_on에서 설정된 ref를 시스템이 자동으로 따라갑니다)
        res = set_desired_force(fd=fd, dir=dir, mod=fc_mode)
        
        if res != 0:
            print(f"⚠️ 목표 힘 설정 실패: {res}")
            return False
            
        return True
    
    def reset(self):
        from DSR_ROBOT2 import movej
        from DSR_ROBOT2 import posj
        
        res = movej(posj(0, 0, 90, 0, 90, 0), vel=self.vel_angular, acc=self.acc_angular)
        
        if res != 0:
            print(f"⚠️ 예외사항 발생!: {res}")
            return False
        
        self.gripper_open()        
        return True
    
    def clear_alarm(self):
        """공식 매뉴얼(SetRobotControl)에 기반한 하드웨어 에러/서보 복구 로직"""
        import DR_init
        from DSR_ROBOT2 import get_robot_state, drl_script_stop, DR_QSTOP_STO
        from dsr_msgs2.srv import SetRobotControl
        import time

        state_code = get_robot_state()
        print(f"🔄 현재 로봇 제어기 상태 코드: {state_code}")

        if state_code == 1:
            return True  # 이미 대기 중(정상)

        # 1. 안전을 위해 현재 실행 중인 스크립트 강제 정지
        try:
            drl_script_stop(DR_QSTOP_STO)
            time.sleep(1.0)
        except Exception as e:
            print(f"스크립트 정지 중 오류 (무시됨): {e}")

        # 2. 제어 상태 강제 변환 서비스 클라이언트 생성
        node = getattr(DR_init, '__dsr__node')
        ns = node.get_namespace()
        if ns == "/": ns = "/dsr01"  # 네임스페이스 기본값 처리
        
        cli = node.create_client(SetRobotControl, f'{ns}/system/set_robot_control')
        
        if not cli.wait_for_service(timeout_sec=2.0):
            print(f"⚠️ {ns}/system/set_robot_control 서비스를 찾을 수 없습니다.")
            return False

        req = SetRobotControl.Request()

        # 3. 상태 코드별 맞춤형 복구 명령 세팅
        if state_code == 5:
            print("🛠️ 보호 정지(Safe Stop) 감지됨. [리셋(2)] 명령을 전송합니다.")
            req.robot_control = 2
        elif state_code == 3:
            print("🔌 서보 꺼짐(Safe Off) 감지됨. [서보 ON(3)] 명령을 전송합니다.")
            req.robot_control = 3
        elif state_code == 6:
            print("🚨 비상 정지(E-Stop) 상태입니다! 물리적 버튼을 직접 해제해야 합니다.")
            return False
        else:
            print(f"⚠️ 기타 예외 상태({state_code}). 안전 리셋(2)을 시도합니다.")
            req.robot_control = 2

        # 4. 비동기 서비스 호출 (결과값 기다리지 않음)
        cli.call_async(req)
        
        # 5. 로봇의 실제 상태가 1(STANDBY)로 돌아올 때까지 직접 모니터링 (최대 10초)
        print("⏳ 복구 명령 전송됨. 로봇 하드웨어 안정화 대기 중...")
        start_time = time.time()
        
        while True:
            current_state = get_robot_state()
            
            if current_state == 1:
                print("🎉 로봇이 정상(STANDBY) 상태로 완벽히 복구되었습니다!")
                time.sleep(1.0) # 상태 1이 된 직후의 물리적 안정화를 위한 짧은 꿀잠
                return True
                
            if time.time() - start_time > 10.0:
                print(f"❌ 복구 시간 초과! (현재 상태 코드가 계속 {current_state}에 머물러 있습니다.)")
                return False
                
            time.sleep(0.5) # 0.5초마다 상태 재확인
    
    def stop(self):
        from DSR_ROBOT2 import dr_stop, STOP_TYPE_QUICK
        dr_stop(STOP_TYPE_QUICK)
        
    def wait(self, time=0):
        """wait(0)을 자주 쓰므로 편의를 위해 래핑"""
        from DSR_ROBOT2 import wait
        return wait(time)
    