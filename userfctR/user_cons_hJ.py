# -*- coding: utf-8 -*-
"""
Module for the definition of user constraints.

Summary
-------
The user constraints enable to impose constraints that can not be resolved using
classical Robotran cuts.
"""
# Author: Robotran Team
# (c) Universite catholique de Louvain, 2020
from mbs_rwt import RwtTrackGeometry
from mbs_rwc import RwcMain

def user_cons_hJ(h, Jac, mbs_data, tsim):
    """Compute the Jacobian and the constraints vector for the user constraints.

    Parameters
    ----------
    h : numpy.ndarray
        The constraint vector to be filled. The first index (h[0]) must not be
        modified. The first index to be filled is h[1].
    Jac : numpy.ndarray
        The jacobian matrix to be filled. The first row (Jac[0,:]) and line (Jac[:,0])
        must not be modified. The subarray to be filled is Jac[1:, 1:].
    mbs_data : MBsysPy.MbsData
        The instance containing the multibody project.
    tsim : float
        The current time of the simulation.

    Returns
    -------
    None
    """

    # Example: Compute the expression of h and Jac then assign the values.
    # h[1] = mbs_data.q[1]-mbs_data.q[2]*mbs_data.q[2]
    # Jac[1,1] =  1.
    # Jac[1,2] = -2*mbs_data.q[2].
    # IMPORTANT: NEVER REASSIGN h => h = np.array([0,mbs_data.q[1]-mbs_data.q[2]*mbs_data.q[2],0])
    #            NEVER REASSIGN Jac => Jac = np.array([[0,0,0,0],[0,1,-2*mbs_data.q[2],0])
    #            Both command will change the values of h, Jac in this function
    #            but they will not be modified outside the scope of this function.
    rwt = RwtTrackGeometry(mbs_data, pointer = mbs_data.user_model['addons']['rwt'])
    rwc = RwcMain(pointer = mbs_data.user_model['addons']['rwc'])
    
    rwt.cons_hJ(mbs_data, h, Jac)
    rwc.compute_constraints(mbs_data, h, Jac)
    
    """id1 = mbs_data.joint_id["R1_caisse1"]
    id2 = mbs_data.joint_id["R1_caisse2"]
    id3 = mbs_data.joint_id["R1_chassis1"]
    id4 = mbs_data.joint_id["R1_chassis2"]

   # define the value of the constraint
    h[1] = (mbs_data.q[id1] + mbs_data.q[id3]) - (mbs_data.q[id2] + mbs_data.q[id4])

   # define the value of the jacobian matrix
    Jac[1,id1] = 1
    Jac[1,id2] = -1
    Jac[1,id3] = 1
    Jac[1,id4] = -1"""
   
    return
