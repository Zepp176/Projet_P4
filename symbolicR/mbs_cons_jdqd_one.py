#
#	MBsysTran - Release 8.1 (built: August 08, 2015)
#
#	Copyright 
#	Universite catholique de Louvain (UCL) 
#	Center for Research in Energy and Mechatronics (CEREM) 
#	2, Place du Levant
#	1348 Louvain-la-Neuve 
#	Belgium 
#
#	http://www.robotran.be 
#
#	==> Generation Date: Thu Mar 25 19:59:55 2021
#
#	==> Project name: one
#
#	==> Number of joints: 72
#
#	==> Function: F18 - Constraints Quadratic Velocity Terms (Jdqd)
#

from math import sin, cos

def cons_jdqd(Jdqd, s):
    q = s.q
    qd = s.qd
 
# Trigonometric functions

    S64 = sin(q[64])
    C64 = cos(q[64])
    S65 = sin(q[65])
    C65 = cos(q[65])
    S66 = sin(q[66])
    C66 = cos(q[66])
    S70 = sin(q[70])
    C70 = cos(q[70])
    S71 = sin(q[71])
    C71 = cos(q[71])
    S72 = sin(q[72])
    C72 = cos(q[72])
    S52 = sin(q[52])
    C52 = cos(q[52])
    S53 = sin(q[53])
    C53 = cos(q[53])
    S54 = sin(q[54])
    C54 = cos(q[54])
    S58 = sin(q[58])
    C58 = cos(q[58])
    S59 = sin(q[59])
    C59 = cos(q[59])
    S60 = sin(q[60])
    C60 = cos(q[60])
 
# Augmented Joint Position Vectors

    Dz131 = q[13]+s.dpt[1,2]
    Dz141 = q[14]+s.dpt[1,2]
    Dz251 = q[25]+s.dpt[1,3]
    Dz261 = q[26]+s.dpt[1,3]
    Dz371 = q[37]+s.dpt[1,4]
    Dz381 = q[38]+s.dpt[1,4]
    Dz491 = q[49]+s.dpt[1,5]
    Dz611 = q[61]+s.dpt[1,6]
 
# Augmented Joint Position Vectors

 
# Constraints and Constraints Jacobian

 
# Constraints Quadratic Terms

    ROjpqp1_45 = -S64*C65
    ROjpqp1_55 = C64*C65
    ROjpqp1_75 = S64*S65
    ROjpqp1_85 = -C64*S65
    ROjpqp1_16 = -ROjpqp1_75*S66+C64*C66
    ROjpqp1_26 = -ROjpqp1_85*S66+S64*C66
    ROjpqp1_36 = -C65*S66
    ROjpqp1_76 = ROjpqp1_75*C66+C64*S66
    ROjpqp1_86 = ROjpqp1_85*C66+S64*S66
    ROjpqp1_96 = C65*C66
    ROjpqp1_110 = ROjpqp1_16*C70+ROjpqp1_45*S70
    ROjpqp1_210 = ROjpqp1_26*C70+ROjpqp1_55*S70
    ROjpqp1_310 = ROjpqp1_36*C70+S65*S70
    ROjpqp1_410 = -ROjpqp1_16*S70+ROjpqp1_45*C70
    ROjpqp1_510 = -ROjpqp1_26*S70+ROjpqp1_55*C70
    ROjpqp1_610 = -ROjpqp1_36*S70+S65*C70
    ROjpqp1_411 = ROjpqp1_410*C71+ROjpqp1_76*S71
    ROjpqp1_511 = ROjpqp1_510*C71+ROjpqp1_86*S71
    ROjpqp1_611 = ROjpqp1_610*C71+ROjpqp1_96*S71
    OMjpqp1_15 = qd[65]*C64
    OMjpqp1_25 = qd[65]*S64
    Ompqpjpqp1_15 = -qd[64]*qd[65]*S64
    Ompqpjpqp1_25 = qd[64]*qd[65]*C64
    OMjpqp1_16 = OMjpqp1_15+ROjpqp1_45*qd[66]
    OMjpqp1_26 = OMjpqp1_25+ROjpqp1_55*qd[66]
    OMjpqp1_36 = qd[64]+qd[66]*S65
    Ompqpjpqp1_16 = Ompqpjpqp1_15+qd[66]*(OMjpqp1_25*S65-ROjpqp1_55*qd[64])
    Ompqpjpqp1_26 = Ompqpjpqp1_25+qd[66]*(-OMjpqp1_15*S65+ROjpqp1_45*qd[64])
    Ompqpjpqp1_36 = qd[66]*(OMjpqp1_15*ROjpqp1_55-OMjpqp1_25*ROjpqp1_45)
    RLjpqp1_17 = ROjpqp1_16*q[67]
    RLjpqp1_27 = ROjpqp1_26*q[67]
    RLjpqp1_37 = ROjpqp1_36*q[67]
    ORjpqp1_17 = OMjpqp1_26*RLjpqp1_37-OMjpqp1_36*RLjpqp1_27
    ORjpqp1_27 = -OMjpqp1_16*RLjpqp1_37+OMjpqp1_36*RLjpqp1_17
    ORjpqp1_37 = OMjpqp1_16*RLjpqp1_27-OMjpqp1_26*RLjpqp1_17
    Apqpjpqp1_17 = OMjpqp1_26*ORjpqp1_37-OMjpqp1_36*ORjpqp1_27+Ompqpjpqp1_26*RLjpqp1_37-Ompqpjpqp1_36*RLjpqp1_27+(2.0)*qd[67]*(OMjpqp1_26*ROjpqp1_36-OMjpqp1_36*ROjpqp1_26)
    Apqpjpqp1_27 = -OMjpqp1_16*ORjpqp1_37+OMjpqp1_36*ORjpqp1_17-Ompqpjpqp1_16*RLjpqp1_37+Ompqpjpqp1_36*RLjpqp1_17+(2.0)*qd[67]*(-OMjpqp1_16*ROjpqp1_36+OMjpqp1_36*ROjpqp1_16)
    Apqpjpqp1_37 = OMjpqp1_16*ORjpqp1_27-OMjpqp1_26*ORjpqp1_17+Ompqpjpqp1_16*RLjpqp1_27-Ompqpjpqp1_26*RLjpqp1_17+(2.0)*qd[67]*(OMjpqp1_16*ROjpqp1_26-OMjpqp1_26*ROjpqp1_16)
    RLjpqp1_18 = ROjpqp1_45*q[68]
    RLjpqp1_28 = ROjpqp1_55*q[68]
    RLjpqp1_38 = q[68]*S65
    ORjpqp1_18 = OMjpqp1_26*RLjpqp1_38-OMjpqp1_36*RLjpqp1_28
    ORjpqp1_28 = -OMjpqp1_16*RLjpqp1_38+OMjpqp1_36*RLjpqp1_18
    ORjpqp1_38 = OMjpqp1_16*RLjpqp1_28-OMjpqp1_26*RLjpqp1_18
    Apqpjpqp1_18 = Apqpjpqp1_17+OMjpqp1_26*ORjpqp1_38-OMjpqp1_36*ORjpqp1_28+Ompqpjpqp1_26*RLjpqp1_38-Ompqpjpqp1_36*RLjpqp1_28+(2.0)*qd[68]*(OMjpqp1_26*S65-OMjpqp1_36*ROjpqp1_55)
    Apqpjpqp1_28 = Apqpjpqp1_27-OMjpqp1_16*ORjpqp1_38+OMjpqp1_36*ORjpqp1_18-Ompqpjpqp1_16*RLjpqp1_38+Ompqpjpqp1_36*RLjpqp1_18+(2.0)*qd[68]*(-OMjpqp1_16*S65+OMjpqp1_36*ROjpqp1_45)
    Apqpjpqp1_38 = Apqpjpqp1_37+OMjpqp1_16*ORjpqp1_28-OMjpqp1_26*ORjpqp1_18+Ompqpjpqp1_16*RLjpqp1_28-Ompqpjpqp1_26*RLjpqp1_18+(2.0)*qd[68]*(OMjpqp1_16*ROjpqp1_55-OMjpqp1_26*ROjpqp1_45)
    RLjpqp1_19 = ROjpqp1_76*q[69]
    RLjpqp1_29 = ROjpqp1_86*q[69]
    RLjpqp1_39 = ROjpqp1_96*q[69]
    ORjpqp1_19 = OMjpqp1_26*RLjpqp1_39-OMjpqp1_36*RLjpqp1_29
    ORjpqp1_29 = -OMjpqp1_16*RLjpqp1_39+OMjpqp1_36*RLjpqp1_19
    ORjpqp1_39 = OMjpqp1_16*RLjpqp1_29-OMjpqp1_26*RLjpqp1_19
    Apqpjpqp1_19 = Apqpjpqp1_18+OMjpqp1_26*ORjpqp1_39-OMjpqp1_36*ORjpqp1_29+Ompqpjpqp1_26*RLjpqp1_39-Ompqpjpqp1_36*RLjpqp1_29+(2.0)*qd[69]*(OMjpqp1_26*ROjpqp1_96-OMjpqp1_36*ROjpqp1_86)
    Apqpjpqp1_29 = Apqpjpqp1_28-OMjpqp1_16*ORjpqp1_39+OMjpqp1_36*ORjpqp1_19-Ompqpjpqp1_16*RLjpqp1_39+Ompqpjpqp1_36*RLjpqp1_19+(2.0)*qd[69]*(-OMjpqp1_16*ROjpqp1_96+OMjpqp1_36*ROjpqp1_76)
    Apqpjpqp1_39 = Apqpjpqp1_38+OMjpqp1_16*ORjpqp1_29-OMjpqp1_26*ORjpqp1_19+Ompqpjpqp1_16*RLjpqp1_29-Ompqpjpqp1_26*RLjpqp1_19+(2.0)*qd[69]*(OMjpqp1_16*ROjpqp1_86-OMjpqp1_26*ROjpqp1_76)
    OMjpqp1_110 = OMjpqp1_16+ROjpqp1_76*qd[70]
    OMjpqp1_210 = OMjpqp1_26+ROjpqp1_86*qd[70]
    OMjpqp1_310 = OMjpqp1_36+ROjpqp1_96*qd[70]
    Ompqpjpqp1_110 = Ompqpjpqp1_16+qd[70]*(OMjpqp1_26*ROjpqp1_96-OMjpqp1_36*ROjpqp1_86)
    Ompqpjpqp1_210 = Ompqpjpqp1_26+qd[70]*(-OMjpqp1_16*ROjpqp1_96+OMjpqp1_36*ROjpqp1_76)
    Ompqpjpqp1_310 = Ompqpjpqp1_36+qd[70]*(OMjpqp1_16*ROjpqp1_86-OMjpqp1_26*ROjpqp1_76)
    OMjpqp1_111 = OMjpqp1_110+ROjpqp1_110*qd[71]
    OMjpqp1_211 = OMjpqp1_210+ROjpqp1_210*qd[71]
    OMjpqp1_311 = OMjpqp1_310+ROjpqp1_310*qd[71]
    Ompqpjpqp1_111 = Ompqpjpqp1_110+qd[71]*(OMjpqp1_210*ROjpqp1_310-OMjpqp1_310*ROjpqp1_210)
    Ompqpjpqp1_211 = Ompqpjpqp1_210+qd[71]*(-OMjpqp1_110*ROjpqp1_310+OMjpqp1_310*ROjpqp1_110)
    Ompqpjpqp1_311 = Ompqpjpqp1_310+qd[71]*(OMjpqp1_110*ROjpqp1_210-OMjpqp1_210*ROjpqp1_110)
    Ompqpjpqp1_112 = Ompqpjpqp1_111+qd[72]*(OMjpqp1_211*ROjpqp1_611-OMjpqp1_311*ROjpqp1_511)
    Ompqpjpqp1_212 = Ompqpjpqp1_211+qd[72]*(-OMjpqp1_111*ROjpqp1_611+OMjpqp1_311*ROjpqp1_411)
    Ompqpjpqp1_312 = Ompqpjpqp1_311+qd[72]*(OMjpqp1_111*ROjpqp1_511-OMjpqp1_211*ROjpqp1_411)
    ROjpqp2_45 = -S52*C53
    ROjpqp2_55 = C52*C53
    ROjpqp2_75 = S52*S53
    ROjpqp2_85 = -C52*S53
    ROjpqp2_16 = -ROjpqp2_75*S54+C52*C54
    ROjpqp2_26 = -ROjpqp2_85*S54+S52*C54
    ROjpqp2_36 = -C53*S54
    ROjpqp2_76 = ROjpqp2_75*C54+C52*S54
    ROjpqp2_86 = ROjpqp2_85*C54+S52*S54
    ROjpqp2_96 = C53*C54
    ROjpqp2_110 = ROjpqp2_16*C58+ROjpqp2_45*S58
    ROjpqp2_210 = ROjpqp2_26*C58+ROjpqp2_55*S58
    ROjpqp2_310 = ROjpqp2_36*C58+S53*S58
    ROjpqp2_410 = -ROjpqp2_16*S58+ROjpqp2_45*C58
    ROjpqp2_510 = -ROjpqp2_26*S58+ROjpqp2_55*C58
    ROjpqp2_610 = -ROjpqp2_36*S58+S53*C58
    ROjpqp2_411 = ROjpqp2_410*C59+ROjpqp2_76*S59
    ROjpqp2_511 = ROjpqp2_510*C59+ROjpqp2_86*S59
    ROjpqp2_611 = ROjpqp2_610*C59+ROjpqp2_96*S59
    OMjpqp2_15 = qd[53]*C52
    OMjpqp2_25 = qd[53]*S52
    Ompqpjpqp2_15 = -qd[52]*qd[53]*S52
    Ompqpjpqp2_25 = qd[52]*qd[53]*C52
    OMjpqp2_16 = OMjpqp2_15+ROjpqp2_45*qd[54]
    OMjpqp2_26 = OMjpqp2_25+ROjpqp2_55*qd[54]
    OMjpqp2_36 = qd[52]+qd[54]*S53
    Ompqpjpqp2_16 = Ompqpjpqp2_15+qd[54]*(OMjpqp2_25*S53-ROjpqp2_55*qd[52])
    Ompqpjpqp2_26 = Ompqpjpqp2_25+qd[54]*(-OMjpqp2_15*S53+ROjpqp2_45*qd[52])
    Ompqpjpqp2_36 = qd[54]*(OMjpqp2_15*ROjpqp2_55-OMjpqp2_25*ROjpqp2_45)
    RLjpqp2_17 = ROjpqp2_16*q[55]
    RLjpqp2_27 = ROjpqp2_26*q[55]
    RLjpqp2_37 = ROjpqp2_36*q[55]
    ORjpqp2_17 = OMjpqp2_26*RLjpqp2_37-OMjpqp2_36*RLjpqp2_27
    ORjpqp2_27 = -OMjpqp2_16*RLjpqp2_37+OMjpqp2_36*RLjpqp2_17
    ORjpqp2_37 = OMjpqp2_16*RLjpqp2_27-OMjpqp2_26*RLjpqp2_17
    Apqpjpqp2_17 = OMjpqp2_26*ORjpqp2_37-OMjpqp2_36*ORjpqp2_27+Ompqpjpqp2_26*RLjpqp2_37-Ompqpjpqp2_36*RLjpqp2_27+(2.0)*qd[55]*(OMjpqp2_26*ROjpqp2_36-OMjpqp2_36*ROjpqp2_26)
    Apqpjpqp2_27 = -OMjpqp2_16*ORjpqp2_37+OMjpqp2_36*ORjpqp2_17-Ompqpjpqp2_16*RLjpqp2_37+Ompqpjpqp2_36*RLjpqp2_17+(2.0)*qd[55]*(-OMjpqp2_16*ROjpqp2_36+OMjpqp2_36*ROjpqp2_16)
    Apqpjpqp2_37 = OMjpqp2_16*ORjpqp2_27-OMjpqp2_26*ORjpqp2_17+Ompqpjpqp2_16*RLjpqp2_27-Ompqpjpqp2_26*RLjpqp2_17+(2.0)*qd[55]*(OMjpqp2_16*ROjpqp2_26-OMjpqp2_26*ROjpqp2_16)
    RLjpqp2_18 = ROjpqp2_45*q[56]
    RLjpqp2_28 = ROjpqp2_55*q[56]
    RLjpqp2_38 = q[56]*S53
    ORjpqp2_18 = OMjpqp2_26*RLjpqp2_38-OMjpqp2_36*RLjpqp2_28
    ORjpqp2_28 = -OMjpqp2_16*RLjpqp2_38+OMjpqp2_36*RLjpqp2_18
    ORjpqp2_38 = OMjpqp2_16*RLjpqp2_28-OMjpqp2_26*RLjpqp2_18
    Apqpjpqp2_18 = Apqpjpqp2_17+OMjpqp2_26*ORjpqp2_38-OMjpqp2_36*ORjpqp2_28+Ompqpjpqp2_26*RLjpqp2_38-Ompqpjpqp2_36*RLjpqp2_28+(2.0)*qd[56]*(OMjpqp2_26*S53-OMjpqp2_36*ROjpqp2_55)
    Apqpjpqp2_28 = Apqpjpqp2_27-OMjpqp2_16*ORjpqp2_38+OMjpqp2_36*ORjpqp2_18-Ompqpjpqp2_16*RLjpqp2_38+Ompqpjpqp2_36*RLjpqp2_18+(2.0)*qd[56]*(-OMjpqp2_16*S53+OMjpqp2_36*ROjpqp2_45)
    Apqpjpqp2_38 = Apqpjpqp2_37+OMjpqp2_16*ORjpqp2_28-OMjpqp2_26*ORjpqp2_18+Ompqpjpqp2_16*RLjpqp2_28-Ompqpjpqp2_26*RLjpqp2_18+(2.0)*qd[56]*(OMjpqp2_16*ROjpqp2_55-OMjpqp2_26*ROjpqp2_45)
    RLjpqp2_19 = ROjpqp2_76*q[57]
    RLjpqp2_29 = ROjpqp2_86*q[57]
    RLjpqp2_39 = ROjpqp2_96*q[57]
    ORjpqp2_19 = OMjpqp2_26*RLjpqp2_39-OMjpqp2_36*RLjpqp2_29
    ORjpqp2_29 = -OMjpqp2_16*RLjpqp2_39+OMjpqp2_36*RLjpqp2_19
    ORjpqp2_39 = OMjpqp2_16*RLjpqp2_29-OMjpqp2_26*RLjpqp2_19
    Apqpjpqp2_19 = Apqpjpqp2_18+OMjpqp2_26*ORjpqp2_39-OMjpqp2_36*ORjpqp2_29+Ompqpjpqp2_26*RLjpqp2_39-Ompqpjpqp2_36*RLjpqp2_29+(2.0)*qd[57]*(OMjpqp2_26*ROjpqp2_96-OMjpqp2_36*ROjpqp2_86)
    Apqpjpqp2_29 = Apqpjpqp2_28-OMjpqp2_16*ORjpqp2_39+OMjpqp2_36*ORjpqp2_19-Ompqpjpqp2_16*RLjpqp2_39+Ompqpjpqp2_36*RLjpqp2_19+(2.0)*qd[57]*(-OMjpqp2_16*ROjpqp2_96+OMjpqp2_36*ROjpqp2_76)
    Apqpjpqp2_39 = Apqpjpqp2_38+OMjpqp2_16*ORjpqp2_29-OMjpqp2_26*ORjpqp2_19+Ompqpjpqp2_16*RLjpqp2_29-Ompqpjpqp2_26*RLjpqp2_19+(2.0)*qd[57]*(OMjpqp2_16*ROjpqp2_86-OMjpqp2_26*ROjpqp2_76)
    OMjpqp2_110 = OMjpqp2_16+ROjpqp2_76*qd[58]
    OMjpqp2_210 = OMjpqp2_26+ROjpqp2_86*qd[58]
    OMjpqp2_310 = OMjpqp2_36+ROjpqp2_96*qd[58]
    Ompqpjpqp2_110 = Ompqpjpqp2_16+qd[58]*(OMjpqp2_26*ROjpqp2_96-OMjpqp2_36*ROjpqp2_86)
    Ompqpjpqp2_210 = Ompqpjpqp2_26+qd[58]*(-OMjpqp2_16*ROjpqp2_96+OMjpqp2_36*ROjpqp2_76)
    Ompqpjpqp2_310 = Ompqpjpqp2_36+qd[58]*(OMjpqp2_16*ROjpqp2_86-OMjpqp2_26*ROjpqp2_76)
    OMjpqp2_111 = OMjpqp2_110+ROjpqp2_110*qd[59]
    OMjpqp2_211 = OMjpqp2_210+ROjpqp2_210*qd[59]
    OMjpqp2_311 = OMjpqp2_310+ROjpqp2_310*qd[59]
    Ompqpjpqp2_111 = Ompqpjpqp2_110+qd[59]*(OMjpqp2_210*ROjpqp2_310-OMjpqp2_310*ROjpqp2_210)
    Ompqpjpqp2_211 = Ompqpjpqp2_210+qd[59]*(-OMjpqp2_110*ROjpqp2_310+OMjpqp2_310*ROjpqp2_110)
    Ompqpjpqp2_311 = Ompqpjpqp2_310+qd[59]*(OMjpqp2_110*ROjpqp2_210-OMjpqp2_210*ROjpqp2_110)
    Ompqpjpqp2_112 = Ompqpjpqp2_111+qd[60]*(OMjpqp2_211*ROjpqp2_611-OMjpqp2_311*ROjpqp2_511)
    Ompqpjpqp2_212 = Ompqpjpqp2_211+qd[60]*(-OMjpqp2_111*ROjpqp2_611+OMjpqp2_311*ROjpqp2_411)
    Ompqpjpqp2_312 = Ompqpjpqp2_311+qd[60]*(OMjpqp2_111*ROjpqp2_511-OMjpqp2_211*ROjpqp2_411)
    jdqd1 = Apqpjpqp1_19-Apqpjpqp2_19
    jdqd2 = Apqpjpqp1_29-Apqpjpqp2_29
    jdqd3 = Apqpjpqp1_39-Apqpjpqp2_39
    jdqd4 = Ompqpjpqp1_112-Ompqpjpqp2_112
    jdqd5 = Ompqpjpqp1_212-Ompqpjpqp2_212
    jdqd6 = Ompqpjpqp1_312-Ompqpjpqp2_312
    Jdqd[1] = jdqd1
    Jdqd[2] = jdqd2
    Jdqd[3] = jdqd3
    Jdqd[4] = jdqd4
    Jdqd[5] = jdqd5
    Jdqd[6] = jdqd6

# Number of continuation lines = 0


