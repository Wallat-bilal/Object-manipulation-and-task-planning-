import time
from threading import Thread, Lock
import mujoco
import mujoco.viewer
import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3
import matplotlib.pyplot as plt
from dmp import dmp_joint.py  #class implemented you can find all of them in dmp folder to be used

class Test:

    def __init__(self):
        self.m = mujoco.MjModel.from_xml_path('Ur5_robot/Robot_scene.xml')
        self.d = mujoco.MjData(self.m)
        self.jointLock = Lock()
        self.GripperLock = Lock()
        self.sendPositions = False
        self.moveGripper = False
        self.joints = [0, 0, 0, 0, 0, 0, 0]
        self.dt = 1 / 100
        self.robot = rtb.models.UR5()  # Use the UR5 model from Robotics Toolbox

    def launch_mujoco(self):
        with mujoco.viewer.launch_passive(self.m, self.d) as viewer:
            start = time.time()
            while viewer.is_running():
                mujoco.mj_step(self.m, self.d)
                viewer.sync()
                time.sleep(self.dt)

    def sendJoint(self, joint_values):
        with self.jointLock:
            self.joints = joint_values
            self.sendPositions = True

    def controlGr(self, Gstate):
        with self.GripperLock:
            self.GrSt = Gstate
            self.moveGripper = True

    def plan_dmp(self, start, goal, timesteps):
        # Initialize and train DMP
        dmp = DMP(start=start, goal=goal, n_bfs=100, dt=self.dt, timesteps=timesteps)
        dmp.train()
        return dmp.generate()

    def send2sim(self, trj):
        for q in trj:
            self.sendJoint(q)
            time.sleep(self.dt)
        return trj[-1]

    def start(self):
        mujoco_thrd = Thread(target=self.launch_mujoco, daemon=True)
        mujoco_thrd.start()
        self.sendJoint(self.robot.qz)  # Initial joint configuration
        input('Press enter to start')

        # Example of moving to a new position using DMP
        goal_pose = SE3(0.5, 0, 0.5) * SE3.RPY([0, 0, np.pi/2], order='xyz')
        q_goal = self.robot.ikine_LMS(goal_pose)  # Inverse kinematics to find the goal joint configuration
        trj = self.plan_dmp(start=self.robot.qz, goal=q_goal.q, timesteps=100)
        self.send2sim(trj)

        print("Operation complete. Press any key to exit.")
        input()

if __name__ == "__main__":
    Test().start()

