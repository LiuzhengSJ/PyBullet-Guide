import time
import numpy as np
import math
import time
useNullSpace = 1
ikSolver = 0
pandaEndEffectorIndex = 11 #8
pandaNumDofs = 7

ll = [-7]*pandaNumDofs
#upper limits for null space (todo: set them to proper range)
ul = [7]*pandaNumDofs
#joint ranges for null space (todo: set them to proper range)
jr = [7]*pandaNumDofs
#restposes for null space
# jointPosition 前7个是关节角度rad,后两个是尖爪位置
jointPositions=[0.98, 0.458, 0.31, -2.24, -0.30, 2.66, 3.32, 0.02, 0.02]
rp = jointPositions

rdyP = [[0,0.0,0.8],[1,0,0,0]]

class PandaSim(object):
  def __init__(self, bullet_client, offset):
    
    self.timeUnreach = 0
    self.stateReached  = False
    self.timeReached = 0
    self.tasks = 10
    self.task_done = 0
    self.targetPQ = np.array([[-0.2,0,0.7],[-0.6,0.2,0.7],[-0.2,0.4,0.7]])
    self.EndPQ = 0
    self.grispClose = 0
    # self.grispCloseDis = 0.02
    # self.grispOpenDis = 0.05
    self.rdyP = rdyP
    
    self.targetPosition = np.array([0,0,0])
    self.targetQuat = np.array([0,0,0,0])
    self.tiimeStable = 0.2
    self.errorP = 0.02
    self.errorGrisp = 0.02
    self.bullet_client = bullet_client
    self.bullet_client.setPhysicsEngineParameter(solverResidualThreshold=0)
    self.offset = np.array(offset)
    
    print("offset=",offset)
    flags = self.bullet_client.URDF_ENABLE_CACHED_GRAPHICS_SHAPES
    self.legos=[]

    #   -------------   水平基准面 -----------------------------------
    objects = [
      self.bullet_client.loadURDF("plane.urdf", [0 , 0 , 0 ],[-0.0, -0.0, -0.0, 1])
    ]

    #   --------------  背景场景    -------------------------------
    objects = [
      self.bullet_client.loadURDF("samurai.urdf", [0 , 0 , 0 ],[-0.0, -0.0, -0.0, 1])
    ]

    #   ---------------   桌子  -------------------------------
    objects = [
      self.bullet_client.loadURDF("table/table.urdf", [0 , 0 , 0 ],[-0.0, 0.0, -0.0, 1])
    ]
    

    
    #   --------------  黑色盘子  -----------------------------
    # self.bullet_client.loadURDF("tray/traybox.urdf", [0.4+offset[0], 0.2+offset[1], 0.61+offset[2]], [-0.0, 0.0, -0.0, 1], flags=flags)
    self.bullet_client.loadURDF("tray/traybox.urdf", [-0.4 + offset[0], 0.2 + offset[1], 0.61 + offset[2]],
                                [-0.0, 0.0, -0.0, 1], flags=flags)
    #   乐高  第一个变成红色
    for i in range(self.tasks):
      euler0 = np.random.rand(3,1)
      qua = self.bullet_client.getQuaternionFromEuler(euler0)
      P = np.array([0.2+0.3*np.random.rand(),-0.1+0.4*np.random.rand(),0.7,0.1,0.2,0.3,0.4])  #rand(1,3)+self.offset
      Q0 = np.random.rand(1,3).tolist()[0]
      Q = self.bullet_client.getQuaternionFromEuler([1,2,3])
      # self.legos.append(self.bullet_client.loadURDF("lego/lego.urdf", P))
      self.legos.append(self.bullet_client.loadURDF("lego/lego.urdf", 0.2+0.3*np.random.rand(),-0.1+0.4*np.random.rand(),0.7,
                                                    qua[0],qua[1],qua[2],qua[3]))

    #   panda 位姿
    orn=[-0.0, 0.0, -0.0, 1]#p.getQuaternionFromEuler([-math.pi/2,math.pi/2,0])
    # eul = self.bullet_client.getEulerFromQuaternion([-0.5, -0.5, -0.5, 0.5])
    self.panda = self.bullet_client.loadURDF("franka_panda/panda.urdf", np.array([0,-0.3,0.6])+self.offset, orn, useFixedBase=True, flags=flags)
    index = 0
    self.state = 0
    self.total_time = 0
    self.control_dt = 1./240.
    self.finger_target = np.array([0.03,0])
    self.gripper_height = 0.2
    #create a constraint to keep the fingers centered
    # 9 denote left finger, 10 denote right finger, here use a gear with ratio -1
    c = self.bullet_client.createConstraint(self.panda,
                       9,
                       self.panda,
                       10,
                       jointType=self.bullet_client.JOINT_GEAR,
                       jointAxis=[1, 0, 0],
                       parentFramePosition=[0, 0, 0],
                       childFramePosition=[0, 0, 0])
    self.bullet_client.changeConstraint(c, gearRatio=-1, erp=0.1, maxForce=50)
 
    for j in range(self.bullet_client.getNumJoints(self.panda)):
      self.bullet_client.changeDynamics(self.panda, j, linearDamping=0, angularDamping=0)
      info = self.bullet_client.getJointInfo(self.panda, j)
      print("Joint info=",info)
      jointName = info[1]
      jointType = info[2]
      if (jointType == self.bullet_client.JOINT_PRISMATIC):
        self.bullet_client.resetJointState(self.panda, j, jointPositions[index]) 
        index=index+1
      if (jointType == self.bullet_client.JOINT_REVOLUTE):
        self.bullet_client.resetJointState(self.panda, j, jointPositions[index]) 
        index=index+1
    self.t = 0.
  def reset(self):
    pass

  def step(self):
    # if self.state==6:
    #   self.finger_target = 0.01
    # if self.state==5:
    #   self.finger_target = 0.04
    # self.bullet_client.submitProfileTiming("step")
    self.update_state()
    #print("self.state=",self.state)
    #print("self.finger_target=",self.finger_target)
    alpha = 0.8 #0.99
    if self.state==1 or self.state==2 or self.state==3 or self.state==5 or self.state==6:
      if self.state == 6:
        alpha = 0.9
      # #gripper_height = 0.034
      # self.gripper_height = alpha * self.gripper_height + (1.-alpha)*0.03
      # if self.state == 2 or self.state == 3 or self.state == 7:
      #   self.gripper_height = alpha * self.gripper_height + (1.-alpha)*0.2
      #
      # t = self.t
      # self.t += self.control_dt
      # pos = [self.offset[0]+0.2 * math.sin(1.5 * t), self.offset[1]+self.gripper_height, self.offset[2]+-0.6 + 0.1 * math.cos(1.5 * t)]
      # if self.state == 3 or self.state== 4:
      #   pos, o = self.bullet_client.getBasePositionAndOrientation(self.legos[0])
      #   pos = [pos[0], self.gripper_height, pos[2]]
      #   self.prev_pos = pos
      # if self.state == 7:
      #   pos = self.prev_pos
      #   diffX = pos[0] - self.offset[0]
      #   diffZ = pos[2] - (self.offset[2]-0.6)
      #   self.prev_pos = [self.prev_pos[0] - diffX*0.1, self.prev_pos[1], self.prev_pos[2]-diffZ*0.1]

      # eu=self.bullet_client.getEulerFromQuaternion(o)
      # orn = self.bullet_client.getQuaternionFromEuler([0,0,1*math.pi/2 ])
      # self.bullet_client.submitProfileTiming("IK")

      # EndPQ = self.bullet_client.getLinkState(self.panda, pandaEndEffectorIndex)
      # jointPosesRd = self.bullet_client.getJointStates(self.panda,[0,1,2,3,4,5,6])
      # linkstate = self.bullet_client.getLinkState(self.panda,11,computeForwardKinematics=1)
      #
      # pos, o = self.bullet_client.getBasePositionAndOrientation(self.legos[0])
      # pos = [pos[0], pos[1], 0.7]
      # poso = np.array(self.bullet_client.getEulerFromQuaternion(o))
      # eu0 = np.array(self.bullet_client.getQuaternionFromEuler([3.14159,0,-poso[2]]))
      
      posInput = alpha * np.array(self.EndPQ[0]) + (1-alpha) * self.targetPosition
      
      # ornInputE = alpha * np.array(self.bullet_client.getEulerFromQuaternion(self.EndPQ[1])) + (1-alpha) * self.targetEular
      # ornInput  = self.bullet_client.getQuaternionFromEuler(ornInputE)
      # ornInput = alpha * np.array(self.EndPQ[1]) + (1-alpha) * eu0  #np.array([1,0,0,0])

#     末端执行器 实时 当前 姿态-欧拉角
#       targetEuler = self.bullet_client.getEulerFromQuaternion(self.targetQuat)
      # 末端执行器 调整姿态 ， 转换成 四元数
      targetEulerTurn = np.array(self.bullet_client.getQuaternionFromEuler([3.14159,0,self.targetEuler[2]]))
      # 末端执行器 实时修正 执行 姿态
      ornInput = alpha * np.array(self.EndPQ[1]) + ( 1 - alpha ) * targetEulerTurn
      
      
      jointPoses = self.bullet_client.calculateInverseKinematics(self.panda,11, posInput, ornInput, ll, ul,
        jr, rp, maxNumIterations=30)
      # jointPosesRd = self.bullet_client.getLinkState(self.panda,pandaEndEffectorIndex)
      # print(jointPoses)
      
      # self.bullet_client.submitProfileTiming()
      for i in range(pandaNumDofs):
        self.bullet_client.setJointMotorControl2(self.panda, i, self.bullet_client.POSITION_CONTROL, jointPoses[i],force=5 * 240.)
      #target for fingers
    elif self.state == 4 or self.state == 7:
      if self.grispClose == 1:
        for i in [9,10]:
          self.bullet_client.setJointMotorControl2(self.panda, i, self.bullet_client.POSITION_CONTROL,self.finger_target[1] ,force= 10)
        # self.bullet_client.submitProfileTiming()
      else:
        for i in [9,10]:
          self.bullet_client.setJointMotorControl2(self.panda, i, self.bullet_client.POSITION_CONTROL,self.finger_target[0] ,force= 10)

class PandaSimAuto(PandaSim):
  def __init__(self, bullet_client, offset):
    PandaSim.__init__(self, bullet_client, offset)
    self.state_t = 0
    self.cur_state = 0
    self.states=[0,3,5,4,6,3,7]
    self.state_durations = 10
  
  def update_state(self):
    self.total_time += self.control_dt
    self.EndPQ = self.bullet_client.getLinkState(self.panda, pandaEndEffectorIndex)
    
    
    
    if self.state == 0:
      self.timeReached += self.control_dt
      if self.timeReached >= self.tiimeStable:
        self.state = 1
        self.timeReached = 0
        # self.rdyP = [self.rdyP[0],self.rdyP[1]]
        self.targetPosition = np.array(self.rdyP[0])
        self.targetQuat = np.array(self.rdyP[1])
        self.targetEuler = np.array(self.bullet_client.getEulerFromQuaternion(self.targetQuat))
        print('from state 0 to 1')
          
          #   --------  运动到 ready 位置，首先判断是否到达目标位置，然后判断是否稳定，最后计算下个位置 ---------------
    elif self.state == 1:
      disP = np.array(self.EndPQ[0]) - self.targetPosition
      disE = np.array((self.bullet_client.getEulerFromQuaternion(self.EndPQ[1]))) - self.targetEuler
      disPnorm = np.linalg.norm(disP)
      disEnorm = np.linalg.norm(disE)

      self.stateJudge(disEnorm,disPnorm,self.errorP)
      
      if (self.timeReached > self.tiimeStable and self.task_done < self.tasks) or (self.timeUnreach > self.state_durations):
        self.state = 2
        self.timeReached = 0
        self.timeUnreach = 0
        #   TODO  设计 grasp 与 lego 角度匹配
        self.legoPQ = self.bullet_client.getBasePositionAndOrientation(self.legos[self.task_done])
        self.targetPosition = np.array(self.legoPQ[0])
        self.targetPosition[2] = 0.8
        self.targetQuat = np.array(self.legoPQ[1])
        self.targetEuler = np.array(self.bullet_client.getEulerFromQuaternion(self.targetQuat))
        print('from state 1 to 2')
        
        #   运动到 lego 上方位置
    elif self.state ==2:
      disP = np.array(self.EndPQ[0]) - self.targetPosition
      disE0 = np.array(self.bullet_client.getEulerFromQuaternion(self.EndPQ[1])) - np.array([-3.14159, 0 , self.targetEuler[2]])
      disE = np.array(self.bullet_client.getEulerFromQuaternion(self.bullet_client.getQuaternionFromEuler(disE0)))
      disPnorm = np.linalg.norm(disP)
      disEnorm = np.linalg.norm(disE)

      self.stateJudge(disEnorm, disPnorm, self.errorP)
      if (self.timeReached >= self.tiimeStable ) or (self.timeUnreach > self.state_durations):
        self.state = 3
        self.timeReached = 0
        self.timeUnreach = 0
        #   匹配 lego 高度
        self.legoPQ = self.bullet_client.getBasePositionAndOrientation(self.legos[self.task_done])
        self.targetPosition[2] = self.legoPQ[0][2]
        print('from state 2 to 3')
    
    #   降高度，准备夹取的位置
    elif self.state ==3:
      disP = np.array(self.EndPQ[0]) - self.targetPosition
      # disE = self.bullet_client.getEulerFromQuaternion(self.EndPQ[1]) - self.legoPQ[1]
      disPnorm = np.linalg.norm(disP)

      self.stateJudge(disPnorm, 0, self.errorP)
      if (self.timeReached >= self.tiimeStable ) or (self.timeUnreach > self.state_durations):
        self.state = 4
        self.timeReached = 0
        self.timeUnreach = 0
        #   TODO 夹爪闭合指令
        self.grispClose = 1
        print('from state 3 to 4')
        
    #     夹爪闭合操作
    elif self.state ==4:
      grispDis = self.bullet_client.getJointStates(self.panda,[9,10])
      disPnorm1 = np.linalg.norm(grispDis[0][0])
      disPnorm2 = np.linalg.norm(grispDis[1][0])

      self.stateJudge(disPnorm1, disPnorm2, 0.05)
      if (self.timeReached >= 2 * self.tiimeStable ) or (self.timeUnreach > self.state_durations):
        self.state = 5
        self.timeReached = 0
        self.timeUnreach = 0
        #   TODO 夹爪闭合指令
        self.grispClose = 1
        self.targetPosition[2] = 0.8
        print('from state 4 to 5')

      #   ----  恢复 lego 位置上方高度
    elif self.state == 5:
      disP = np.array(self.EndPQ[0]) - self.targetPosition
      disE0 = np.array(self.bullet_client.getEulerFromQuaternion(self.EndPQ[1])) - np.array([-3.14159, 0 , self.targetEuler[2]])#self.bullet_client.getEulerFromQuaternion(self.legoPQ[1])
      disE = np.array(self.bullet_client.getEulerFromQuaternion(self.bullet_client.getQuaternionFromEuler(disE0)))
      disPnorm = np.linalg.norm(disP)
      disEnorm = np.linalg.norm(disE)

      self.stateJudge(disEnorm, disPnorm, self.errorP)
      if (self.timeReached >= self.tiimeStable ) or (self.timeUnreach > self.state_durations):
        self.state = 6
        self.timeReached = 0
        self.timeUnreach = 0
        #   TODO 随机选择目标位置
        self.targetPosition = self.targetPQ[1]
        self.targetPosition[2] = 0.8
        print('from state 5 to 6')
        
    # -----   移动到  随机 目标 位置上方，欧拉角不变
    elif self.state == 6:
      disP = np.array(self.EndPQ[0]) - self.targetPosition
      # disE = np.array(self.bullet_client.getEulerFromQuaternion(self.EndPQ[1])) - self.targetPQ[1]
      disPnorm = np.linalg.norm(disP)

      self.stateJudge(disPnorm, 0, self.errorP)
      if (self.timeReached >= self.tiimeStable ) or (self.timeUnreach > self.state_durations):
        self.state = 7
        self.timeReached = 0
        self.timeUnreach = 0
        #   匹配 lego 高度
        self.grispClose = 0
        print('from state 6 to 7')

    #   打开夹爪，释放lego
    elif self.state == 7:
      grispDis = self.bullet_client.getJointStates(self.panda, [7, 8])
      disPnorm1 = np.linalg.norm(grispDis[0][0]) - jointPositions[7]
      disPnorm2 = np.linalg.norm(grispDis[1][0]) - jointPositions[8]

      self.stateJudge(disPnorm1, disPnorm2, self.errorGrisp)
      if (self.timeReached >= self.tiimeStable ) or (self.timeUnreach > self.state_durations):
        self.task_done += 1
        print('have finished', self.task_done,' tasks')
        if self.task_done < self.tasks:
          self.state = 2
          self.timeReached = 0
          self.timeUnreach = 0
          print('from state 7 to 2')
          self.legoPQ = self.bullet_client.getBasePositionAndOrientation(self.legos[self.task_done])
          self.targetPosition = np.array(self.legoPQ[0])
          self.targetPosition[2] = 0.8
          self.targetQuat = np.array(self.legoPQ[1])
          self.targetEuler = np.array(self.bullet_client.getEulerFromQuaternion(self.targetQuat))
        else:
          self.state = 1
          self.timeReached = 0
          # self.rdyP = [self.rdyP[0],self.rdyP[1]]
          self.targetPosition = np.array(self.rdyP[0])
          self.targetQuat = np.array(self.rdyP[1])
          self.targetEuler = np.array(self.bullet_client.getEulerFromQuaternion(self.targetQuat))
          print('from state 7 to 1')

  def stateJudge(self,er1,er2,tgt):
    if er1 + er2 < tgt:
      self.timeReached += self.control_dt
    else:
      self.timeUnreach += self.control_dt


