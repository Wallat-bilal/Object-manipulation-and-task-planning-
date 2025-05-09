from dmp import dmp_joint
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt



if __name__ == '__main__':
    demo_filename = "Jtrajectory.csv"
    demo = pd.read_csv(demo_filename)
    qtrj = demo.to_numpy()
    dt = 1/100

    # calculate time (tau) = to length of the trajectory
    tau = len(demo) * dt
    ts = np.arange(0, tau, dt)
    cs_alpha = -np.log(0.0001)

    ## encode DMP 

    dmp_q = dmp_joint.JointDMP(NDOF=7,n_bfs=100, alpha=48, beta=12, cs_alpha=cs_alpha)

    ## Encode the dmp e.g., train 
    dmp_q.train(qtrj, ts, tau)

    ## integrate DMP
    q_out, dq_out, ddq_out = dmp_q.rollout(ts, tau, FX=True)
    
    # plot position response 
    plt.figure(1)
    plt.plot(q_out, 'b')
    plt.plot(qtrj, 'r')
    plt.legend(['DMP traj', 'Orig traj'])

    # plot velocity response 

    # plot acceleration response 

    ## ------  change duration of the DMP vafiables tau and ts
   
    #--------  Integrate DMP ----------    

    # plot position response 

    # plot velocity response 

    # plot acceleration response 

    ##change duration and goal (dmp_q.gp)of the DMP without the forcing term f(x) = False

    dmp_q.gp = [np.pi/2, -2.99868213e-01,5.56952098e-05,-2.19981486e+00, 8.28152916e-06, 1.99994665e+00, 7.85484458e-01]

    #--------  Integrate DMP ----------    

    # plot position response 

    # plot velocity response 

    # plot acceleration response 

    plt.show()