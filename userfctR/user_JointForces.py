# -*- coding: utf-8 -*-
"""Module for the definition of joint forces."""
# Author: Robotran Team
# (c) Universite catholique de Louvain, 2020


def user_JointForces(mbs_data, tsim):
    """Compute the force and torques in the joint.

    It fills the MBsysPy.MbsData.Qq array.

    Parameters
    ----------
    mbs_data : MBsysPy.MbsData
        The multibody system associated to this computation.
    tsim : float
        The current time of the simulation.

    Notes
    -----
    The numpy.ndarray MBsysPy.MbsData.Qq is 1D array with index starting at 1.
    The first index (array[0]) must not be modified. The first index to be
    filled is array[1].

    Returns
    -------
    None
    """
    # cleaning previous forces value
    mbs_data.Qq[1:] = 0.
    K = 940000  #Nm/rad
    
    id1 = mbs_data.joint_id["R1_caisse1"]
    id2 = mbs_data.joint_id["R1_caisse2"]
    
    mbs_data.Qq[id1] = -K * mbs_data.q[id1]
    mbs_data.Qq[id2] = -K * mbs_data.q[id2]
    
    id_essieu1 = mbs_data.joint_id["R2_Essieu1"]
    id_essieu2 = mbs_data.joint_id["R2_Essieu2"]
    id_essieu3 = mbs_data.joint_id["R2_Essieu3"]
    id_essieu4 = mbs_data.joint_id["R2_Essieu4"]
    
    m = 1813*4 + 2615*2 + 32000
    g = 1/9
    a = g*9.81
    C = m*a*0.46/4
    
    mbs_data.Qq[id_essieu1] = C
    mbs_data.Qq[id_essieu2] = C
    mbs_data.Qq[id_essieu3] = C
    mbs_data.Qq[id_essieu4] = C
    
    # lateral bumpstop
    id3 = mbs_data.joint_id['T2_ball1']
    id4 = mbs_data.joint_id['T2_ball2']
    T2_1 = mbs_data.q[id3]
    T2_2 = mbs_data.q[id4]
    
    x = [0.025, 0.03, 0.035, 0.04, 0.045, 0.05, 0.055, 0.06, 0.065]  # déplacemnt en m
    y = [0, 600, 1760, 3730, 6870, 11580, 17170, 29200, 230000]      # force en N
    F1 = 0
    F2 = 0
    
    if T2_1 > 0:
        
        for k in range(8):
            if x[k] < T2_1 <= x[k+1]:
                i = k
                F1 = y[i]+(y[i+1]-y[i])*((T2_1-x[i])/(x[i+1]-x[i]))
                break
        mbs_data.Qq[id3] = -F1
    
    if T2_2 > 0:   
        
        for k in range(8):
            if x[k] < T2_2 <= x[k+1]:
                j = k
                F2 = y[j]+(y[j+1]-y[j])*((T2_2-x[j])/(x[j+1]-x[j]))
                break
        mbs_data.Qq[id4] = -F2
    
    if T2_1 < 0:
        
        for k in range(8):
            if -x[k] > T2_1 >= -x[k+1]:
                i = k
                F1 = y[i]+(y[i+1]-y[i])*((-T2_1-x[i])/(x[i+1]-x[i]))
                break
        mbs_data.Qq[id3] = F1
    
    if T2_2 < 0:
        
        for k in range(8):
            if -x[k] > T2_2 >= -x[k+1]:
                j = k
                F2 = y[j]+(y[j+1]-y[j])*((-T2_2-x[j])/(x[j+1]-x[j]))
                break
        mbs_data.Qq[id4] = F2

    return
