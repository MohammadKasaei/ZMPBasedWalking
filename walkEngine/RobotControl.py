import pybullet as p
import math
import pybullet_data as pd

deg2rad = lambda x: math.radians(x)
SAT = lambda inp, UB, LB: max(min(inp, UB), LB)


class RobotControl():
    def __init__(self):

        self.physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
       
        self.init_robot()

        
    def init_robot(self):
        
        p.setGravity(0,0,-9.81)
        p.setAdditionalSearchPath(pd.getDataPath())
        self.sampling_time = 1/240.0
        p.setTimeStep(self.sampling_time)
        
        self.FloorId = p.loadURDF("plane.urdf",[0,0,0])
        p.setAdditionalSearchPath("")
        
        self.Coman_ID = p.loadURDF("models/coman/model_org.urdf",[0,0,0.5])
        
        num_joints = p.getNumJoints(self.Coman_ID)
        link_names = {}

        for i in range(num_joints):
            joint_info = p.getJointInfo(self.Coman_ID, i)
            link_name = joint_info[12].decode('utf-8')
            link_names[i] = link_name
            print(f"Link Index: {i}, Link Name: {link_name}")

        target_link_index = [40, 54]  # change this to the link you want
        for link_id in target_link_index:
            p.changeDynamics(self.Coman_ID, link_id, lateralFriction=10)
            p.changeVisualShape(self.Coman_ID, link_id,rgbaColor=(0.3,0.5,0.5,1))
        


        
        
        WaistX_DOF = 3
        WaistY_DOF = 2
        WaistZ_DOF = 4

        RShX_DOF = 7
        RShY_DOF = 8
        RShZ_DOF = 9
        REB_DOF =10

        LShX_DOF = 21
        LShY_DOF = 22
        LShZ_DOF = 23
        LEB_DOF =24

        RHipX_DOF =35
        RHipY_DOF =36
        RHipZ_DOF =37
        RKneeX_DOF =38
        RAnkleX_DOF =40
        RAnkleY_DOF =39

        LHipX_DOF =49
        LHipY_DOF =50
        LHipZ_DOF =51
        LKneeX_DOF =52
        LAnkleX_DOF =54
        LAnkleY_DOF =53


        self.joint_ID_order = [WaistX_DOF,WaistY_DOF,WaistZ_DOF,
                        RShX_DOF,RShY_DOF,RShZ_DOF,REB_DOF,
                        LShX_DOF,LShY_DOF,LShZ_DOF,LEB_DOF,
                        RHipX_DOF,RHipY_DOF,RHipZ_DOF,RKneeX_DOF,RAnkleX_DOF,RAnkleY_DOF,
                        LHipX_DOF,LHipY_DOF,LHipZ_DOF,LKneeX_DOF,LAnkleX_DOF,LAnkleY_DOF
                        ]


        self.numJoints = p.getNumJoints(self.Coman_ID)
        
        for j in range(self.numJoints):
            p.setJointMotorControl2(bodyIndex=self.Coman_ID, jointIndex=j, controlMode=p.POSITION_CONTROL,targetPosition=0)
        
        
        WaistX = deg2rad(0)
        WaistY = deg2rad(0)
        WaistZ = deg2rad(0)

        RShX = deg2rad(20)
        RShY = deg2rad(-20)
        RShZ = deg2rad(0)
        REB = deg2rad(-70)

        LShX = deg2rad(20)
        LShY = deg2rad(20)
        LShZ = deg2rad(0)
        LEB = deg2rad(-70)
        
        self.z_leg = -0.375

        JPR = self.IK([0,0,self.z_leg,0])
        RHipX = -JPR[0]
        RHipY = JPR[1]
        RHipZ = JPR[2]
        RKneeX = JPR[3]
        RAnkleX = JPR[4]
        RAnkleY = JPR[5]


        JPL = self.IK([0,0,self.z_leg,0])
        LHipX = -JPL[0]
        LHipY = JPL[1]
        LHipZ = JPL[2]
        LKneeX = JPL[3]
        LAnkleX = JPL[4]
        LAnkleY = JPL[5]

        
        RfootPosition = [0,0,self.z_leg,0]
        LfootPosition = [0,0,self.z_leg,0]
        self.JointPos = self.update_joint_pos(RfootPosition,LfootPosition,(0,0),(0,0))
        
        self.JointPosions = [WaistX,WaistY,WaistZ,
                        RShX,RShY,RShZ,REB,
                        LShX,LShY,LShZ,LEB,
                        RHipX,RHipY,RHipZ,RKneeX,RAnkleX,RAnkleY,
                        LHipX,LHipY,LHipZ,LKneeX,LAnkleX,LAnkleY]
        
        p.enableJointForceTorqueSensor(self.Coman_ID,42,1)
        p.enableJointForceTorqueSensor(self.Coman_ID,56,1)

        self.JointStateLeft = p.getLinkState(self.Coman_ID,42,1)
        self.JointStateRight = p.getLinkState(self.Coman_ID,56,1)
        self.reset_upper_body()
        self.CoP_L = []
        self.CoP_R = []
        posAndori = p.getBasePositionAndOrientation(self.Coman_ID)
        self.ori = p.getEulerFromQuaternion(posAndori[1])
        self.orip = p.getEulerFromQuaternion(posAndori[1])
        

        
        
    def reset_upper_body(self):
        
        self.WaistX = deg2rad(0)
        self.WaistY = deg2rad(0)
        self.WaistZ = deg2rad(0)
        
        self.RShX = deg2rad(20)
        self.RShY = deg2rad(-20)
        self.RShZ = deg2rad(0)
        self.REB = deg2rad(-70)
        
        self.LShX = deg2rad(20)
        self.LShY = deg2rad(20)
        self.LShZ = deg2rad(0)
        self.LEB = deg2rad(-70)
        
        JointPosions = [self.WaistX,self.WaistY,self.WaistZ,
                        self.RShX,self.RShY,self.RShZ,self.REB,
                        self.LShX,self.LShY,self.LShZ,self.LEB]
        
        self.update_upper_body_motors(JointPosions,(0,0))
        
    def generate_arm_motions(self,first_step_is_right,arm_amp,arm_offset,StepTheta, twalk,StepTime):
        
        if (first_step_is_right):
            LShX = math.radians(arm_offset ) + arm_amp*math.sin(twalk*2*math.pi/(2*StepTime)) #- EN_ArmX * 2 * PDArmX
            RShX= math.radians(arm_offset ) - arm_amp*math.sin(twalk*2*math.pi/(2*StepTime)) #- EN_ArmX * 2 * PDArmX
            Ltheta = math.radians(StepTheta)*SAT(twalk/StepTime,1,0)
            Rtheta = math.radians(StepTheta)*SAT((twalk-StepTime)/StepTime,1,0)
            
        else:
            LShX = math.radians(arm_offset ) - arm_amp*math.sin(twalk*2*math.pi/(2*StepTime)) #- EN_ArmX * 2 * PDArmX
            RShX = math.radians(arm_offset ) + arm_amp*math.sin(twalk*2*math.pi/(2*StepTime)) #- EN_ArmX * 2 * PDArmX
            Ltheta = math.radians(StepTheta)*SAT((twalk-StepTime)/StepTime,1,0)
            Rtheta = math.radians(StepTheta)*SAT(twalk/StepTime,1,0)

        arms_joint_pos = [self.WaistX,self.WaistY,self.WaistZ,
                          RShX,self.RShY,self.RShZ,self.REB,
                          LShX,self.LShY,self.LShZ,self.LEB]
        
        return arms_joint_pos, Ltheta, Rtheta
    
    
    
    def update_legs_motors(self,joint_pos):
        JointIDOrder = self.joint_ID_order[-len(joint_pos):]
        for j in range(len(JointIDOrder)):
            p.setJointMotorControl2(bodyIndex=self.Coman_ID, jointIndex=JointIDOrder[j], controlMode=p.POSITION_CONTROL,
                                                targetPosition=joint_pos[j])

    def update_upper_body_motors(self,joint_pos,waste_balance_ctrl_cmd=(0,0)):
        joint_ID_order = self.joint_ID_order[:len(joint_pos)]
        joint_pos[0] += waste_balance_ctrl_cmd[0]
        joint_pos[1] += waste_balance_ctrl_cmd[1]
        
        
        for j in range(len(joint_ID_order)):
            p.setJointMotorControl2(bodyIndex=self.Coman_ID, jointIndex=joint_ID_order[j], controlMode=p.POSITION_CONTROL,
                                                targetPosition=joint_pos[j])
        
    def IK(self,targetPosition):

        Lu = 0.225
        Ld = 0.200

        sqrLx =  (targetPosition[0]* targetPosition[0])+(targetPosition[2]* targetPosition[2])
        sqrLy =  (targetPosition[1]* targetPosition[1])+(targetPosition[2]* targetPosition[2])
        Lx = math.sqrt(sqrLx)
        Ly = math.sqrt(sqrLy)

        sqrLu = Lu * Lu
        sqrLd = Ld * Ld

        alpha = math.acos((sqrLx+sqrLu-sqrLd) / (2*Lx*Lu))
        hipX = alpha + math.atan(targetPosition[0] / Lx)
        hipY = math.atan(targetPosition[1] / Ly)
        hipTheta = targetPosition[3]

        cosKnee = (sqrLu+sqrLd-sqrLx) / (2.0*Lu*Ld)
        kneeX  = math.pi-(math.acos(cosKnee))
        ankleX = hipX-(kneeX)
        ankleY = -hipY
        
        JointPoses = [hipX,hipY,hipTheta,kneeX,ankleX,ankleY]
        return JointPoses 

    def update_joint_pos(self, RfootPosition,LfootPosition,ankle_x_balance_ctrl_cmd= (0,0),ankle_y_balance_ctrl_cmd=(0,0)):
        JP_right = self.IK(RfootPosition)
        RHipX = -JP_right[0]
        RHipY = JP_right[1]
        RHipZ = JP_right[2]
        RKneeX = JP_right[3]
        RAnkleX = JP_right[4]
        RAnkleY = JP_right[5]


        JP_left = self.IK(LfootPosition)
        LHipX = -JP_left[0]
        LHipY = JP_left[1]
        LHipZ = JP_left[2]
        LKneeX = JP_left[3]
        LAnkleX = JP_left[4]
        LAnkleY = JP_left[5]

        out = ([RHipX,RHipY,RHipZ,RKneeX,RAnkleX + ankle_x_balance_ctrl_cmd[0], RAnkleY+ankle_y_balance_ctrl_cmd[0],
                 LHipX,LHipY,LHipZ,LKneeX,LAnkleX+ ankle_x_balance_ctrl_cmd[1], LAnkleY+ankle_y_balance_ctrl_cmd[1]])
        
        return out


    def PD_control(self, error, d_err, P_gain, D_gain, saturation):
        return SAT((P_gain*(error)) + (D_gain*d_err),saturation[0],saturation[1])

    def balance_control(self,ori,CoP_L,CoP_R):
        
        PD_Wx = 1*self.PD_control(-ori[1], (ori[1]-self.orip[1])/self.sampling_time, 2, -0.0, [0.3,-0.3])
        PD_Wy = 1*self.PD_control(ori[0], (ori[0]-self.orip[0])/self.sampling_time, 1., 0.02, [0.08,-0.08])

        PD_ALx = 1*self.PD_control(-CoP_L[1], 0, 0.35, 0 , [0.15,-0.15])
        PD_ARx = 1*self.PD_control(-CoP_R[1], 0, 0.35, 0 , [0.15,-0.15])
        
        PD_ALy = 1*self.PD_control(-CoP_L[0], 0, 0.35, 0 , [0.15,-0.15])
        PD_ARy = 1*self.PD_control(-CoP_R[0], 0, 0.35, 0 , [0.15,-0.15])
        
        self.orip = ori
        return (PD_Wx,PD_Wy),(PD_ALx,PD_ARx),(PD_ALy,PD_ARy)
        
    def update_sensors(self):
        
        # pose :

        self.orip = self.ori
        
        posAndori = p.getBasePositionAndOrientation(self.Coman_ID)
        self.pos = posAndori[0]
        self.ori = p.getEulerFromQuaternion(posAndori[1])
        posp = self.pos
        orip = self.ori
        ########################## Update ZMP
        joint_state_left = p.getJointState(self.Coman_ID,42)
        joint_state_right = p.getJointState(self.Coman_ID,56)

        force_torque_left = joint_state_left[2]
        force_torque_right = joint_state_right[2]

        H = 0.05
        p1 = 50
        if (force_torque_right[2] < 50):
            xCopR = -( force_torque_right[4] + force_torque_right[0] ) * H / p1
            yCopR = -( force_torque_right[3] + force_torque_right[1] ) * H / p1
        else:
            xCopR = -(  force_torque_right[4] + force_torque_right[0] ) * H / force_torque_right[2]
            yCopR = -(  force_torque_right[3] + force_torque_right[1] ) * H / force_torque_right[2]
        
        if (force_torque_left[2] < 50):
            xCopL = -( force_torque_left[4] + force_torque_left[0] ) * H / p1
            yCopL = -( force_torque_left[3] + force_torque_left[1] ) * H / p1
        else:
            xCopL = -(  force_torque_left[4] + force_torque_left[0] ) * H / force_torque_left[2]
            yCopL = -(  force_torque_left[3] + force_torque_left[1] ) * H / force_torque_left[2]

        if (self.CoP_L == []):
            self.CoP_L = [xCopL,yCopL]
            self.CoP_R = [xCopR,yCopR]
        else:
            a = 0.8
            b = 0.2
            #CoP_L = [(CoP_L[0]+xCopL)/2,(CoP_L[1]+yCopL)/2]
            #CoP_R = [(CoP_R[0]+xCopR)/2,(CoP_R[1]+yCopR)/2]
            self.CoP_L = [(a*self.CoP_L[0]+b*xCopL),(a*self.CoP_L[1]+b*yCopL)]
            self.CoP_R = [(a*self.CoP_R[0]+b*xCopR),(a*self.CoP_R[1]+b*yCopR)]
            
        
        #print (str(xCopL),",",str(yCopL),",",str(xCopR),",",str(yCopR))
        return self.pos,self.ori,self.CoP_L,self.CoP_R
            

    def SAT(self, inp, UB,LB):
        if inp > UB:
            return UB
        elif inp < LB:
            return LB
        else:
            return inp

