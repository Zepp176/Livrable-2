# -*- coding: utf-8 -*-
"""Module for the definition of user links forces."""
# Author: Robotran Team
# (c) Universite catholique de Louvain, 2020


def user_LinkForces(Z, Zd, mbs_data, tsim, identity):
    """Compute the force in the given link.

    Parameters
    ----------
    Z : float
        The distance between the two anchor points of the link.
    Zd : float
        The relative velocity between the two anchor points of the link.
    mbs_data : MBsysPy.MbsData
        The multibody system associated to this computation.
    tsim : float
        The current time of the simulation.
    identity : int
        The identity of the computed link.

    Returns
    -------
    Flink : float
        The force in the current link.

    """

    m_caisse = 32000.0
    m_bogie = 2615.0
    m_essieu = 1813.0
    g = 9.81

    Flink = 0.0
    
    L1 = mbs_data.link_id['KXL1'] # ressorts longitudinaux primaires
    L2 = mbs_data.link_id['KXL2']
    L3 = mbs_data.link_id['KXL3']
    L4 = mbs_data.link_id['KXL4']
    L5 = mbs_data.link_id['KXR1']
    L6 = mbs_data.link_id['KXR2']
    L7 = mbs_data.link_id['KXR3']
    L8 = mbs_data.link_id['KXR4']
    L9 = mbs_data.link_id['KYL1'] # ressorts latéraux primaires
    L10 = mbs_data.link_id['KYL2']
    L11 = mbs_data.link_id['KYL3']
    L12 = mbs_data.link_id['KYL4']
    L13 = mbs_data.link_id['KYR1']
    L14 = mbs_data.link_id['KYR2']
    L15 = mbs_data.link_id['KYR3']
    L16 = mbs_data.link_id['KYR4']
    L17 = mbs_data.link_id['KDZL1'] # ressorts + amortisseurs verticaux primaires
    L18 = mbs_data.link_id['KDZL2']
    L19 = mbs_data.link_id['KDZL3']
    L20 = mbs_data.link_id['KDZL4']
    L21 = mbs_data.link_id['KDZR1']
    L22 = mbs_data.link_id['KDZR2']
    L23 = mbs_data.link_id['KDZR3']
    L24 = mbs_data.link_id['KDZR4']
    L25 = mbs_data.link_id['CKZL1'] # ressorts verticaux secondaires
    L26 = mbs_data.link_id['CKZL2']
    L27 = mbs_data.link_id['CKZR1']
    L28 = mbs_data.link_id['CKZR2']
    L29 = mbs_data.link_id['DZL1'] # amortisseurs verticaux secondaires
    L30 = mbs_data.link_id['DZL2']
    L31 = mbs_data.link_id['DZR1']
    L32 = mbs_data.link_id['DZR2']
    L33 = mbs_data.link_id['DYL1'] # amortisseur latéraux secondaires
    L34 = mbs_data.link_id['DYL2']
    L35 = mbs_data.link_id['DYR1']
    L36 = mbs_data.link_id['DYR2']
    L37 = mbs_data.link_id['CKXL1'] # ressorts longitudinaux secondaires
    L38 = mbs_data.link_id['CKXR1']
    L39 = mbs_data.link_id['CKXL2']
    L40 = mbs_data.link_id['CKXR2']
    L41 = mbs_data.link_id['CKYL1'] # ressort latéraux seconadaires
    L42 = mbs_data.link_id['CKYR1']
    L43 = mbs_data.link_id['CKYL2']
    L44 = mbs_data.link_id['CKYR2']
    
    if identity in [L1, L2, L3, L4, L5, L6, L7, L8]: # ressorts longitudinaux primaires
        K  = mbs_data.user_model['kx1']['k'] *1000 # 31391 kN/m
        C  = mbs_data.user_model['kx1']['c'] *1000 # 15 kNs/m
        Z0 = mbs_data.user_model['kx1']['l'] /1000 # 450 mm
        Flink = K*(Z-Z0)+C*Zd
    
    if identity in [L9, L10, L11, L12, L13, L14, L15, L16]: # ressorts latéraux primaires
        K  = mbs_data.user_model['ky1']['k'] *1000 # 3884 kN/m
        C  = mbs_data.user_model['ky1']['c'] *1000 # 2 kNs/m
        Z0 = mbs_data.user_model['ky1']['l'] /1000 # 400 mm
        Flink = K*(Z-Z0)+C*Zd
       
    if identity in [L17, L18, L19, L20, L21, L22, L23, L24]: # ressorts + amortisseur verticaux primaires
        K  = (mbs_data.user_model['kz1']['k'] + mbs_data.user_model['dz1']['k']) *1000 # 1220 + 1000 kN/m
        C  = (mbs_data.user_model['kz1']['c'] + mbs_data.user_model['dz1']['c']) *1000 # 0 + 4 kNs/m
        Z0 = mbs_data.user_model['kz1']['l'] /1000 # 420 mm
        Flink = K*(Z-Z0)+C*Zd - (m_caisse + 2*m_bogie)*g/8
    
    if identity in [L25, L26, L27, L28]: # ressorts verticaux secondaires
        K  = mbs_data.user_model['kz2']['k'] *1000 # 430 kN/m
        #C  = mbs_data.user_model['kz2']['c'] *1000 # 0 kNs/m
        Z0 = mbs_data.user_model['kz2']['l'] /1000 # 605 mm
        Flink = K*(Z-Z0) - m_caisse*g/4
       
    if identity in [L29, L30, L31, L32]: # amortisseurs verticaux secondaires
        # pas de ressort ??
        C  = mbs_data.user_model['dz2']['c'] *1000 # 20 kNs/m
        Flink = C*Zd
    
    if identity in [L33, L34, L35, L36]: # amortisseur latéraux secondaires
        # pas de ressort ??
        C  = mbs_data.user_model['dy2']['c'] *1000 # 32 kNs/m
        Flink = C*Zd
       
    if identity in [L37, L38, L39, L40]: # ressorts longitudinaux secondaires
        K  = mbs_data.user_model['kx2']['k'] *1000 # 160 kN/m
        Z0 = mbs_data.user_model['kx2']['l'] /1000 # 400 mm
        Flink = K*(Z-Z0)

    if identity in [L41, L42, L43, L44]: # ressort latéraux seconadaires
        K  = mbs_data.user_model['ky2']['k'] *1000 # 160 kN/m
        Z0 = mbs_data.user_model['ky2']['l'] /1000 # 400 mm
        Flink = K*(Z-Z0)
       
    return Flink
