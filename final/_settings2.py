from dynamixel_sdk import *
from table import *
import sys

print("✅ settings.py : table address 불러오는 중") 

# 포트 및 핸들러 초기화
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION)
groupSyncRead = GroupSyncRead(portHandler, packetHandler, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)

print("✅ settings.py : table address 불러오기 성공") 


def start():
    """Dynamixel 포트 오픈 및 보드레이트 설정"""
    
    
    # if not portHandler.openPort():
    #     print(f"❌ 포트를 열 수 없습니다: {DEVICENAME}")
    #     return False

    # if not portHandler.setBaudRate(BAUDRATE):
    #     print(f"❌ 보드레이트 설정 실패: {BAUDRATE}")
    #     portHandler.closePort()
    #     return False

    print(f"✅ settings.py : 포트 {DEVICENAME} 열림, 보드레이트 {BAUDRATE} 설정 완료")
    return True


def mode(ids, mode_number):
    """여러 Dynamixel에 대해 모드를 설정합니다.
       입력: ids - 단일 id(int) 또는 id 리스트, mode_number - 설정할 모드 번호
    """
    # 만약 단일 id가 들어왔다면 리스트로 변환
    if not isinstance(ids, list):
        ids = [ids]
    
    # 포트가 열려있는지 확인
    if not portHandler.openPort():
        print(f"🚨 settings: 포트가 닫혀 있어 다시 엽니다: {DEVICENAME}")
        portHandler.openPort()

    for motor_id in ids:
        # Torque Disable
        result, error = packetHandler.write1ByteTxRx(portHandler, motor_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        if result != COMM_SUCCESS:
            print(f"❌ Torque Disable 실패 (ID {motor_id}): {packetHandler.getTxRxResult(result)}")
            continue
        if error != 0:
            print(f"❌ Torque Disable 에러 (ID {motor_id}): {packetHandler.getRxPacketError(error)}")
            continue

        # Set Operating Mode
        result, error = packetHandler.write2ByteTxRx(portHandler, motor_id, ADDR_OPERATE_MODE, mode_number)
        if result != COMM_SUCCESS:
            print(f"❌ 모터 모드 설정 오류 (ID {motor_id}): {packetHandler.getTxRxResult(result)}")
            continue
        if error != 0:
            print(f"❌ 모터 모드 설정 에러 (ID {motor_id}): {packetHandler.getRxPacketError(error)}")
            continue
        
        sys.stdout.write(f"✅ settings: Operating mode set successfully for ID {motor_id}")
        #sys.stdout.flush()
        # print(f"✅ settings: Operating mode set successfully for ID {motor_id}")

        # Torque Enable
        result, error = packetHandler.write1ByteTxRx(portHandler, motor_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        if result != COMM_SUCCESS:
            print(f"❌ Torque Enable 실패 (ID {motor_id}): {packetHandler.getTxRxResult(result)}")
            continue
        if error != 0:
            print(f"❌ Torque Enable 에러 (ID {motor_id}): {packetHandler.getRxPacketError(error)}")
            continue

    return True


def set_velocity(ids, velocity):
    """여러 Dynamixel에 대해 동일 속도를 설정합니다.
       입력: ids - 단일 id 또는 id 리스트, velocity - 목표 속도 값
    """
    if not isinstance(ids, list):
        ids = [ids]
    
    if not portHandler.openPort():
        print(f"🚨 settings: 포트가 닫혀 있어 다시 엽니다: {DEVICENAME}")
        portHandler.openPort()
    
    # velocity_control 모드로 전환
    mode(ids, velocity_control)
    
    for motor_id in ids:
        result, error = packetHandler.write4ByteTxRx(portHandler, motor_id, ADDR_GOAL_VELOCITY, velocity)
        if result != COMM_SUCCESS:
            print(f"❌ 목표 속도 설정 실패 (ID {motor_id}): {packetHandler.getTxRxResult(result)}")
        if error != 0:
            print(f"❌ 목표 속도 설정 에러 (ID {motor_id}): {packetHandler.getRxPacketError(error)}")


def set_position(ids, position):
    """여러 Dynamixel에 대해 동일 위치를 설정합니다.
       입력: ids - 단일 id 또는 id 리스트, position - 목표 위치 값
    """
    if not isinstance(ids, list):
        ids = [ids]
    
    if not portHandler.openPort():
        print(f"🚨 settings: 포트가 닫혀 있어 다시 엽니다: {DEVICENAME}")
        portHandler.openPort()
    
    # position_control 모드로 전환
    mode(ids, position_control)
    
    for motor_id in ids:
        result, error = packetHandler.write4ByteTxRx(portHandler, motor_id, ADDR_GOAL_POSITION, position)
        if result != COMM_SUCCESS:
            print(f"❌ 목표 위치 설정 실패 (ID {motor_id}): {packetHandler.getTxRxResult(result)}")
            continue
        if error != 0:
            print(f"❌ 목표 위치 설정 에러 (ID {motor_id}): {packetHandler.getRxPacketError(error)}")
            continue
