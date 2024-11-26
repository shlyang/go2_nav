import time
import sys
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.sport.sport_client import SportClient
from unitree_sdk2py.go2.obstacles_avoid.obstacles_avoid_client import ObstaclesAvoidClient

class AtomAction:
    def __init__(self) -> None:
        # 初始化运动客户端
        self.sport_client = SportClient()
        self.sport_client.SetTimeout(10.0)
        self.sport_client.Init()
        
        # 初始化避障客户端
        self.avoid_client = ObstaclesAvoidClient()
        self.avoid_client.SetTimeout(3.0)
        self.avoid_client.Init()
        while not self.avoid_client.SwitchGet()[1]:
            self.avoid_client.SwitchSet(True)
            time.sleep(0.1)
        self.avoid_client.UseRemoteCommandFromApi(True)
        time.sleep(0.5)
        print("Obstacle avoidance activated")
        
    def stand_up(self):
        """机器人站立"""
        self.sport_client.StandUp()
        print("Standing up...")
        time.sleep(1.0)
        
    def stand_down(self):
        """机器人趴下"""
        self.sport_client.StandDown()
        print("Standing down...")
        time.sleep(1.0)
        
    def balance_stand(self):
        """平衡站立"""
        self.sport_client.BalanceStand()
        print("Balance standing...")
        # time.sleep(1.0)
        
    def recovery_stand(self):
        """恢复站立姿态"""
        self.sport_client.RecoveryStand()
        print("Recovery standing...")
        # time.sleep(1.0)
        
    def damp(self):
        """阻尼模式"""
        self.sport_client.Damp()
        print("Damp...")
        
    def move_with_avoid(self, vx, vy, vyaw, duration=1.0):
        """带避障的移动
        
        Args:
            vx (float): x方向速度 (前后)
            vy (float): y方向速度 (左右)
            vyaw (float): 旋转角速度
            duration (float): 移动持续时间(秒)
        """
        self.avoid_client.Move(vx, vy, vyaw)
        # time.sleep(duration)
        print("Obstacle avoidance moving with {}, {}, {}".format(vx, vy, vyaw))
            
    def move(self, vx, vy, vyaw, duration=1.0):
        """简单移动(不带避障)
        
        Args:
            vx (float): x方向速度 (前后)
            vy (float): y方向速度 (左右)
            vyaw (float): 旋转角速度
            duration (float): 移动持续时间(秒)
        """
        self.sport_client.Move(vx, vy, vyaw)
        print("Moving with {}, {}, {}".format(vx, vy, vyaw))
        # time.sleep(duration)

    def stop(self):
        """停止移动"""
        self.sport_client.StopMove()
        self.avoid_client.Move(0.0, 0.0, 0.0)
        print("Stopping...")
        # self.avoid_client.UseRemoteCommandFromApi(False)
        
# 示例使用
if __name__ == "__main__":
    if len(sys.argv)>1:
        ChannelFactoryInitialize(0, sys.argv[1])
    else:
        ChannelFactoryInitialize(0)
        
    robot = AtomAction()
    
    try:
        print("Starting basic movement test...")
        
        # 基础动作测试
        robot.stand_up()
        time.sleep(1)
        
        # 简单移动测试
        print("Testing simple move...")
        robot.simple_move(0.3, 0, 0, 2.0)  # 向前移动2秒
        time.sleep(1)
        
        # 避障移动测试
        print("Testing obstacle avoidance move...")
        robot.move_with_avoid(0.3, 0, 0, 2.0)  # 带避障向前移动2秒
        time.sleep(1)
        
        # 结束动作
        robot.stand_down()
        robot.damp()
        
    except KeyboardInterrupt:
        print("Test interrupted!")
        robot.stand_down()
        robot.damp()
