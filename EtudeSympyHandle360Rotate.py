import sympy as sym
sym.init_printing()
Pi = sym.S.Pi # 円周率
E = sym.S.Exp1 # 自然対数の底
I = sym.S.ImaginaryUnit # 虚数単位

# 使用する変数の定義(小文字1文字は全てシンボルとする)
(a,b,c,d,e,f,g,h,i,j,k,l,m,n,o,p,q,r,s,t,u,v,w,x,y,z) = sym.symbols('a b c d e f g h i j k l m n o p q r s t u v w x y z')

import sys
import math
import numpy as np
import matplotlib.pyplot as plt
#matplotlib inline

from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm

from matplotlib.patches import Circle
import mpl_toolkits.mplot3d.art3d as art3d
import matplotlib.animation as animation
import datetime
from matplotlib.transforms import CompositeGenericTransform
from matplotlib.transforms import Affine2D

#############################################################################################################################
# パラメータ値の入力
#############################################################################################################################
# 回転角の入力
deltaTheta = 0.1

# 車輪データの要素数（＝エレメント数）の定義
Elements = 1000

# 描画関係パラメータ
nfr = 900 # Number of frames 動画の再現枚数（1回のシミュレーションで描画する枚数）
fps = 500 # Frame per sec

# キャスター角 := caster
caster = 70
casterRad = caster * np.pi / 180

# キャンバー角
steering = 50

# トレール長の設定．フロントフォーク（＝ハンドル回転中心）と前輪中心間の距離となる．
Offset = 10

# 車輪径の定義
Diameter = 20

# ホイールベースの定義
WB = 500

# 回転中心の定義
# 回転中心は前輪と後輪の接地点で結ばれたベクトルとなる
Point1 = np.array([0, 0, Diameter - Offset / math.cos(casterRad)])
Point2 = np.array([Diameter / math.tan(casterRad) - Offset / math.sin(casterRad), 0, 0])

V = sym.Matrix([                                                            0, 0, Diameter - Offset / math.cos(casterRad)])
W = sym.Matrix([Diameter / math.tan(casterRad) - Offset / math.sin(casterRad), 0,                                       0])
Shaft = W - V

print("Shaft = ", Shaft)
print("Shaft*math.cos(casterRad) = ", Shaft*math.cos(casterRad))

RotateCenter = Point2 - Point1
print("RotateCenter = ", RotateCenter)
print("cos(casterRad) =", math.cos(casterRad))

# 前輪中心の設定
Wheel_O = np.array([np.zeros(Elements), np.zeros(Elements), Diameter*np.ones(Elements)])
F_wheelCenter = sym.Matrix([sym.Integer(0),   sym.Integer(0), sym.Integer(Diameter)])
R_wheelCenter = sym.Matrix([sym.Integer(0), sym.Integer(-WB), sym.Integer(Diameter)])

#############################################################################################################################
# 回転行列算出関数
#############################################################################################################################
# 右手座標系の回転処理関数（回転角，回転中心ベクトル）
# 回転角はdegree（0〜360°）を入力する．
def rtnArb_Rot(PrTheta, n):
    n_x = n[0]
    n_y = n[1]
    n_z = n[2]
    t = PrTheta*np.pi/180
    Rot_Rod = np.array([[np.cos(t) + n_x**2*(1-np.cos(t))     , n_x*n_y*(1-np.cos(t)) - n_z*np.sin(t), n_x*n_z*(1-np.cos(t)) + n_y*np.sin(t)],
                        [n_y*n_x*(1-np.cos(t)) + n_z*np.sin(t), np.cos(t) + n_y**2*(1-np.cos(t))     , n_y*n_z*(1-np.cos(t)) - n_x*np.sin(t)],
                        [n_z*n_x*(1-np.cos(t)) - n_y*np.sin(t), n_z*n_y*(1-np.cos(t)) + n_x*np.sin(t), np.cos(t) + n_z**2*(1-np.cos(t))     ]])
    #print("Rot_Rod = ", Rot_Rod)
    return Rot_Rod

def rtnSymArb_Rot(PrTheta, n):
    n_x = n[0]
    n_y = n[1]
    n_z = n[2]
    t = PrTheta*np.pi/180
    Rot_Rod = np.array([[np.cos(t) + n_x**2*(1-np.cos(t))     , n_x*n_y*(1-np.cos(t)) - n_z*np.sin(t), n_x*n_z*(1-np.cos(t)) + n_y*np.sin(t)],
                        [n_y*n_x*(1-np.cos(t)) + n_z*np.sin(t), np.cos(t) + n_y**2*(1-np.cos(t))     , n_y*n_z*(1-np.cos(t)) - n_x*np.sin(t)],
                        [n_z*n_x*(1-np.cos(t)) - n_y*np.sin(t), n_z*n_y*(1-np.cos(t)) + n_x*np.sin(t), np.cos(t) + n_z**2*(1-np.cos(t))     ]])
    #print("Rot_Rod = ", Rot_Rod)
    rtn_Rtn = sym.Matrix(Rot_Rod)
    # rtn_Rtn = Rot_Rod
    return rtn_Rtn

#############################################################################################################################
# 3次元描画関数
#############################################################################################################################
def plotp3(func, simbol=[t], rangeL=[(-4, 4, 100)]):
    X, Y, Z = func
    t1 = simbol[0]
    if len(simbol) >= 2:
        u1 = simbol[1]
    t_rangeL = rangeL[0]
    print("t_rangeL = ", t_rangeL)
    t_ndarray = np.linspace(t_rangeL[0], t_rangeL[1], t_rangeL[2]) # ndarray

    if len(rangeL) >= 2:
        u_rangeL = rangeL[1]
        u_ndarray = np.linspace(u_rangeL[0], u_rangeL[1], u_rangeL[2]) # ndarray
    if len(rangeL) == 1:
        # AttributeError: 'numpy.ndarray' object has no attribute 'subs'問題の解決
        # 参考：https://stackoverflow.com/questions/55674193/how-to-change-symbols-to-sympy-subs-in-the-np-array
        Solve_X = np.vectorize(lambda x: x.subs([(t1, T) for T in t_ndarray]))(X)
        Solve_Y = np.vectorize(lambda x: x.subs([(t1, T) for T in t_ndarray]))(Y)
        Solve_Z = np.vectorize(lambda x: x.subs([(t1, T) for T in t_ndarray]))(Z)
        Xp = np.array([float(sym.N(Solve_X))])
        Yp = np.array([float(sym.N(Solve_Y))])
        Zp = np.array([float(sym.N(Solve_Z))])
        # Xp = np.array([float(sym.N(X.subs([(t1, T)]))) for T in t_ndarray])
        # Yp = np.array([float(sym.N(Y.subs([(t1, T)]))) for T in t_ndarray])
        # Zp = np.array([float(sym.N(Z.subs([(t1, T)]))) for T in t_ndarray])
    else:

        # AttributeError: 'numpy.ndarray' object has no attribute 'subs'問題の解決
        # 参考：https://stackoverflow.com/questions/55674193/how-to-change-symbols-to-sympy-subs-in-the-np-array
        Solve_X = np.vectorize(lambda x: x.subs([(t1, T), (u1, U)]))(X)
        Solve_Y = np.vectorize(lambda x: x.subs([(t1, T), (u1, U)]))(Y)
        Solve_Z = np.vectorize(lambda x: x.subs([(t1, T), (u1, U)]))(Z)
        Xp = np.array([float(sym.N(Solve_X)) for T in t_ndarray for U in u_ndarray])
        Yp = np.array([float(sym.N(Solve_Y)) for T in t_ndarray for U in u_ndarray])
        Zp = np.array([float(sym.N(Solve_Z)) for T in t_ndarray for U in u_ndarray])
        # Xp = np.array([float(sym.N(X.subs([(t1, T), (u1, U)]))) for T in t_ndarray for U in u_ndarray])
        # Yp = np.array([float(sym.N(Y.subs([(t1, T), (u1, U)]))) for T in t_ndarray for U in u_ndarray])
        # Zp = np.array([float(sym.N(Z.subs([(t1, T), (u1, U)]))) for T in t_ndarray for U in u_ndarray])

    fig = plt.figure()
    ax = Axes3D(fig)

    # グラフのプロット幅を均一にする
    max_range = np.array([Xp.max()-Xp.min(), Yp.max()-Yp.min(), Zp.max()-Zp.min()]).max() * 0.5
    mid_x = (Xp.max() + Xp.min()) * 0.5
    mid_y = (Yp.max() + Yp.min()) * 0.5
    mid_z = (Zp.max() + Zp.min()) * 0.5
    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)

    ax.plot(Xp, Yp, Zp)
    plt.show()
    plt.close()
    
#############################################################################################################################
# メイン処理
#############################################################################################################################

# 車輪の定義：　tを媒介変数としている
X = Diameter*sym.cos(t)
Y = sym.Integer(0)      # sympyでintをシンボル扱いにするにはこの関数を用いる
Z = Diameter*sym.sin(t)

# 前輪
F_Wheel = sym.Matrix([X, Y, Z]) + F_wheelCenter
print("F_Wheel = ", F_Wheel)

# 後輪
R_Wheel = sym.Matrix([X, Y, Z]) + R_wheelCenter
print("F_Wheel = ", R_Wheel)

# print('n = ', n)
norm_n = np.linalg.norm(RotateCenter)
# print('norm of n = ', norm_n)
input_n = RotateCenter / norm_n

# 回転行列の導出
R = rtnSymArb_Rot(deltaTheta, input_n)
rtn_wheel = np.dot(R, F_Wheel)
print("rtn_wheel = ", rtn_wheel)
print(" ")

# デバッグ用print文
print("type(F_Wheel) = ", type(F_Wheel))
print("type(rtn_wheel) = ", type(rtn_wheel))
print(" ")

#############################################################################################################################
# 描画処理
#############################################################################################################################
# 前輪の描画（テスト用）
# plotp3([X, Y, Z], [t], [(-5 * np.pi, 5 * np.pi, 200)])

# sym.Marix表現でのPlot3D
#plotp3(F_Wheel, [t], [(-5 * np.pi, 5 * np.pi, 200)])
plotp3(rtn_wheel, [t], [(-5 * np.pi, 5 * np.pi, 200)])



'''
X = (5+2*sym.cos(t))*sym.cos(u)
Y = (5+2*sym.sin(u))*sym.sin(u)
Z = 2*sym.sin(t)-u/sym.oo
plotp3([X, Y, Z], [t, u], [(-np.pi, np.pi, 40), (-np.pi, np.pi, 40)])


# 一般的なsympyによるグラフ表示
X = 2*sym.cos(t)
Y = 5*sym.sin(t)
Z = t
sym.plotting.plot3d_parametric_line(X, Y, Z, (t, -5*np.pi, 5*np.pi))

X = (5+2*sym.cos(t))*sym.cos(u)
Y = (5+2*sym.sin(u))*sym.sin(u)
Z = 2*sym.sin(t)-u/sym.oo
sym.plotting.plot3d_parametric_surface(X, Y, Z, (t, -np.pi, np.pi), (u, -np.pi, np.pi))
'''