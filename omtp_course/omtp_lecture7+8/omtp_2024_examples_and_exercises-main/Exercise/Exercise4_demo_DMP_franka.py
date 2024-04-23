import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import time
from threading import Thread

import glfw
import mujoco
import numpy as np
import matplotlib.pyplot as plt
from dmp import dmp_joint
import roboticstoolbox as rp
import spatialmath as sm
import pandas as pd

class Demo:
    qpos0 = [0, -0.785, 0, -2.356, 0, 1.571, 0.785]
    print(qpos0)
    height, width = 600, 800  # Rendering window resolution.
    fps = 30  # Rendering framerate.

    def __init__(self) -> None:
        demo_filename = "Jtrajectory.csv"
        self.demo = pd.read_csv(demo_filename)
        self.qtrj = self.demo.to_numpy()
        self.dt = 1/100

        self.model = mujoco.MjModel.from_xml_path('franka_emika_panda/scene.xml')
        self.data = mujoco.MjData(self.model)
        self.cam = mujoco.MjvCamera()
        self.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
        self.cam.fixedcamid = 0
        self.scene = mujoco.MjvScene(self.model, maxgeom=10000)
        for i in range(1, 8):
            self.data.joint(f"joint{i}").qpos = self.qpos0[i - 1]
        mujoco.mj_forward(self.model, self.data)

        # Initialize ROS node and publisher
        rospy.init_node('demo_dmp_franka', anonymous=True)
        self.pub = rospy.Publisher('/franka_joint_states', JointState, queue_size=10)

    def getState(self):
        qState = []
        for i in range(1, 8):
            qState.append(float(self.data.joint(f"joint{i}").qpos))
        return qState

    def step(self) -> None:
        panda = rp.models.Panda()
        q_cur = self.getState()
        panda.q = q_cur

        Tep = sm.SE3.Trans(0, 0, -0.3) * panda.fkine(panda.q)
        Trj = rp.ctraj(panda.fkine(panda.q), Tep, 500)
        qik = panda.ikine_LM(Trj, q0=panda.q)

        for qs in qik.q:
            joint_state_msg = JointState()
            joint_state_msg.header = Header()
            joint_state_msg.header.stamp = rospy.Time.now()
            joint_state_msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7']
            joint_state_msg.position = qs

            self.pub.publish(joint_state_msg)
            for i in range(1, 8):
                self.data.actuator(f"actuator{i}").ctrl = qs[i - 1]
            mujoco.mj_step(self.model, self.data)
            time.sleep(self.dt)

        print('Done')

    def render(self) -> None:
        glfw.init()
        glfw.window_hint(glfw.SAMPLES, 8)
        window = glfw.create_window(self.width, self.height, "Demo", None, None)
        glfw.make_context_current(window)
        self.context = mujoco.MjrContext(
            self.model, mujoco.mjtFontScale.mjFONTSCALE_100
        )
        opt = mujoco.MjvOption()
        pert = mujoco.MjvPerturb()
        viewport = mujoco.MjrRect(0, 0, self.width, self.height)
        while not glfw.window_should_close(window):
            w, h = glfw.get_framebuffer_size(window)
            viewport.width = w
            viewport.height = h
            mujoco.mjv_updateScene(
                self.model,
                self.data,
                opt,
                pert,
                self.cam,
                mujoco.mjtCatBit.mjCAT_ALL,
                self.scene,
            )
            mujoco.mjr_render(viewport, self.scene, self.context)
            time.sleep(1.0 / self.fps)
            glfw.swap_buffers(window)
            glfw.poll_events()
        glfw.terminate()

    def start(self) -> None:
        step_thread = Thread(target=self.step)
        step_thread.start()
        self.render()

if __name__ == "__main__":
    Demo().start()

