# -*- coding: utf-8 -*-
import sys
import math
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from mpl_toolkits.mplot3d import Axes3D
import mpl_toolkits.mplot3d.art3d as art3d
import matplotlib.animation as animation
import numpy as np
import datetime
from matplotlib.transforms import CompositeGenericTransform
from matplotlib.transforms import Affine2D

# ##################################################################
# 入力パラメータの設定
# ##################################################################

# 回転角の入力
deltaomega = 0.1

# 車輪データの要素数（＝エレメント数）の定義
Elements = 1000

# 描画関係パラメータ
nfr = 900 # Number of frames 動画の再現枚数（1回のシミュレーションで描画する枚数）
fps = 500 # Frame per sec

# キャスター角 := theta
theta = 70
thetaRad = theta * np.pi / 180

# 車輪回転角の限界値（実際にハンドルが曲がる限界）
steeringLimit = 50

# トレール長の設定．フロントフォーク（＝ハンドル回転中心）と前輪中心間の距離となる．
Offset = 10

# 車輪径の定義
Diameter = 20

# 回転中心の定義
# 回転中心は前輪と後輪の接地点で結ばれたベクトルとなる
Point1 = np.array([0, 0, Diameter - Offset / math.cos(thetaRad)])
Point2 = np.array([Diameter / math.tan(thetaRad) - Offset / math.sin(thetaRad), 0, 0])
RotateCenter = Point2 - Point1
print("Point1 = ", Point1)
print("Point2 = ", Point2)

# 前輪中心の設定
Wheel_O = np.array([np.zeros(Elements), np.zeros(Elements), Diameter*np.ones(Elements)])

# 微小区間εの定義
epsilon = sys.float_info.epsilon
# print('epsilon = ', epsilon)

# ##################################################################
# 回転元の点の指定
# ##################################################################
O = np.array([0, 0, 0])             # 原点位置

# 回転前の座標を設定
B = np.array([-2, -2, 1])

# ##################################################################
# サブ関数の定義
# ##################################################################
# 回転中心ベクトル算出関数
def calcRotCentVector(param1, param2):
    # 回転中心ベクトルの算出
    n = [param1[0] - param2[0], param1[1] - param2[1], param1[2] - param2[2]]
    print("n = ", n)
    nn = param1 - param2
    print("nn = ", nn)

    # 回転中心ベクトルのノルムを導出
    norm_n = np.linalg.norm(n)

    # 回転中心基本ベクトルの算出
    input_n = n / norm_n

    # 回転中心基底ベクトルの出力
    return input_n

# 点の描画処理（点，色）指定
def plotDot(PrBB, PrColor):

    for x, y, z in [PrBB]:
        ax.scatter(x, y, z, color = PrColor, marker = 's')

# 矢印の描画
def arrow(v, sp, c):
    # 関数名(始点位置，ベクトル，色)
    # 始点位置を基準としてでベクトルで示した方向にプロットする
    # v:始点位置、sp:ベクトル、c:色
    #ax.quiver(v[0]+sp[0], v[1]+sp[1], v[2]+sp[2],
    #          v[0], v[1], v[2],
    #          length=np.linalg.norm(v),
    #          color=c, linewidth=3)
    ax.quiver(v[0], v[1], v[2],
              sp[0], sp[1], sp[2],
              #length=np.linalg.norm(sp),
              np.linalg.norm(sp),
              color=c, linewidth=3)
    # print("ベクトル sp の大きさ", np.linalg.norm(sp))

# 車輪の描画
def plotWheel(data):
    for angle in range(0, 360):
        # plt.cla()                      # 現在描写されているグラフを消去．使用してもアニメーション描画できなかったので削除した
        wheelData = DotDot_Itr[angle]
        for x, y, z in wheelData:
            ax.scatter(x, y, z, color = 'r', marker = 's')

# 右手座標系の回転処理関数（回転角，回転中心ベクトル）
# 回転角はdegree（0〜360°）を入力する．
def rtnArb_Rot(Promega, n):
    n_x = n[0]
    n_y = n[1]
    n_z = n[2]
    t = Promega*np.pi/180
    Rot_Rod = np.array([[np.cos(t) + n_x**2*(1-np.cos(t))     , n_x*n_y*(1-np.cos(t)) - n_z*np.sin(t), n_x*n_z*(1-np.cos(t)) + n_y*np.sin(t)],
                        [n_y*n_x*(1-np.cos(t)) + n_z*np.sin(t), np.cos(t) + n_y**2*(1-np.cos(t))     , n_y*n_z*(1-np.cos(t)) - n_x*np.sin(t)],
                        [n_z*n_x*(1-np.cos(t)) - n_y*np.sin(t), n_z*n_y*(1-np.cos(t)) + n_x*np.sin(t), np.cos(t) + n_z**2*(1-np.cos(t))     ]])
    #print("Rot_Rod = ", Rot_Rod)
    return Rot_Rod


# 愛点行列算出関数
def rotM(p):
    # 回転行列を計算する
    px = p[0]
    py = p[1]
    pz = p[2]

    # 物体座標系の 1->2->3 軸で回転させる
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(px), np.sin(px)],
                   [0, -np.sin(px), np.cos(px)]])
    Ry = np.array([[np.cos(py), 0, -np.sin(py)],
                   [0, 1, 0],
                   [np.sin(py), 0, np.cos(py)]])
    Rz = np.array([[np.cos(pz), np.sin(pz), 0],
                   [-np.sin(pz), np.cos(pz), 0],
                   [0, 0, 1]])
    R = Rz.dot(Ry).dot(Rx)

    return R

# 例
# p = np.array([np.pi, np.pi/2, np.pi/3])
# R = rotM(p)
# A = R.T

# ##################################################################
# 設定値の内部パラメータへの変換
# ##################################################################
# print('n = ', n)
norm_n = np.linalg.norm(RotateCenter)
# print('norm of n = ', norm_n)
input_n = RotateCenter / norm_n
# print("input_n = ", input_n)

# 設定した回転角と回転中心に応じた回転行列を導出
R = rtnArb_Rot(deltaomega, input_n) 

# ##################################################################
# 初期位置の車輪を設定，描画
# 1回目の開店後の前輪点群データを算出
# ##################################################################

# プロットサイズの指定
# fig = plt.figure(figsize=(10,10),dpi=200)

# 車輪の描画（前輪）
omega = np.linspace(0, 2*np.pi, Elements)
# print("type of omega = ", type(omega))
# print("shape of omega = ", np.shape(omega))
# print("omega = ", omega)
WheelInit = [Diameter * np.cos(omega), np.zeros(Elements), Diameter * np.sin(omega)] + Wheel_O
# print("type of WheelInit = ", type(WheelInit))
# print("type of Wheel_O", type(Wheel_O))
# print("WheelInit = ", WheelInit)

#for omega in range(0, 360):
#    DotDot_Tmp = np.array([Diameter * np.cos(omega*np.pi/180), 0, Diameter * np.sin(omega*np.pi/180)] + Front_O)
#    plotDot(DotDot_Tmp, 'y')
#    DotDot_F = np.append(DotDot_F, [DotDot_Tmp], axis = 0)
# print("DotDot_F = ", DotDot_F)
# print('len(DotDot_F) = ', len(DotDot_F))
# print('type(DotDot_F) = ', type(DotDot_F)) = <class 'numpy.ndarray'>
# print('DotDot_F.ndim = ', DotDot_F.ndim)
# print('DotDot_F.shape = ', DotDot_F.shape)

# ##################################################################
# 回転処理
# ##################################################################

#　バンク後の車輪データ（バンク中の暫定車輪データ）の定義
DotDot_Tmp = np.empty((0,3), int)
DotDot_Itr = np.empty((0, 360, 3), int)

# 接地点の初期化
# 前輪描画データ：PtGnd_F
# 後輪描画データ：PtGnd_R
PtGnd_F = np.empty((0,3), int)
PtGnd_R = np.empty((0,3), int)

# 描画データの初期化（前輪）車輪の360°回転分のデータ（x，y，z軸）要素数＝3600個
xfG = []
yfG = []
zfG = []

# キャンバー角でのセルフステアによる軌跡
xfGC = []
yfGC = []
zfGC = []
xfGD = []
yfGD = []
zfGD = []

# 回転中心ベクトルの設定
input_n = calcRotCentVector(Point1, Point2)
print("input_n", input_n)

# ハンドル回転処理：　フロントフォークを中心に車輪を回転させる．
# 0°から359°まで前輪，後輪を倒した時のそれぞれのデータ（前輪，後輪）の算出及び描画処理を実行する
# 前輪データ： PtGnd_F
# 後輪データ： PtGnd_R
# leanAngle: 傾き角（リーン角）→0°〜360°まで傾ける
# range：　反復回数を示す．1回当たりの変化量が0.1なので，反復回数＝3600で傾き角＝360°となる．
for omega in range(0, 3600):    #1回の遷移は0.01°．ハンドルを360°回転させる．range()の引数に指定できるのは整数intのみ．
    
    # 設定した回転角と回転中心に応じた回転行列を導出
    R = rtnArb_Rot(omega/10, input_n)

    iCounter = 0

    # ハンドル回転角度＝0°の初期車輪位置：　WheelInit
    # WheelInitを回転行列Rを経てomega分だけ回転させた時の車輪はDotDot_Tmpになる．
    # print("WheelInit = ", WheelInit)
    DotDot_Tmp = np.dot(R, WheelInit)
    row, col = DotDot_Tmp.shape

    min_index = np.argmin(DotDot_Tmp[2,:])
    # print("min_index = ", min_index)
    # print("minimum of DotDot_Tmp = ", np.min(DotDot_Tmp[2,:]))

    xfG.append(DotDot_Tmp[0,min_index])
    yfG.append(DotDot_Tmp[1,min_index])
    zfG.append(DotDot_Tmp[2,min_index])

    if omega <= steeringLimit*10:
        xfGC.append(DotDot_Tmp[0,min_index])
        yfGC.append(DotDot_Tmp[1,min_index])
        zfGC.append(DotDot_Tmp[2,min_index])
    elif 3600 - steeringLimit*10 <= omega <= 3600:
        xfGD.append(DotDot_Tmp[0,min_index])
        yfGD.append(DotDot_Tmp[1,min_index])
        zfGD.append(DotDot_Tmp[2,min_index])

##########################################################
# 2次元グラフ表示処理
##########################################################
FFig = plt.figure(figsize=(Diameter/0.9, Diameter/1.6), dpi=80)
dAx = FFig.add_subplot(111)
dAx.set_title("Trajectory of Turning Handle at 360 degrees")
dAx.set_xlim(-(2*Diameter+2), 2*np.pi*10)
dAx.set_ylim(-(2*Diameter+2), 2*Diameter+1)
dAx.spines['right'].set_color('none')
dAx.spines['top'].set_color('none')
dAx.xaxis.set_ticks_position('bottom')
dAx.spines['bottom'].set_position(('data',0))
#plt.xticks([-40, -20, 0, 10*np.pi/2, 20*np.pi], [r'$40$', r'$20$', r'$0$', r'$+\np.pi/2$', r'$+2\np.pi$'])
dAx.yaxis.set_ticks_position('left')
dAx.spines['left'].set_position(('data',0))
dAx.set_xlabel("Turning Direction", horizontalalignment='right', x=1.0)
dAx.set_ylabel("Traveling  Direction", horizontalalignment='right', y=1.0)
# グラフのグリッドを正方形にする
dAx.set_aspect('equal')

# 90度回転
t = CompositeGenericTransform(Affine2D.identity().rotate_deg(90), dAx.transData)
# 回転しない（0°回転）
# t = CompositeGenericTransform(Affine2D.identity().rotate_deg(0), dAx.transData)
dAx.plot(xfG, yfG, color="red",linestyle="-", transform=t)
dAx.plot(xfGC, yfGC, color="red",linestyle="solid", linewidth = 3, transform=t)
dAx.plot(xfGD, yfGD, color="red",linestyle="solid", linewidth = 3, transform=t)

dAx.text(Diameter, Diameter, r'Chaster angle = {0} degree,  Diameter = {1} '.format(theta, Diameter))
dAx.text(Diameter, Diameter - 2, r'Offset between center of the wheel and caster = {0} '.format(Offset))
dAx.text(Diameter, Diameter - 4, r'Steering angle is {0} degree.'.format(steeringLimit))
dAx.text(Diameter, Diameter - 6, r'Thick red line shows a trajectory which touches a ground from -{0} degree to {1} degree in steering angle'.format(steeringLimit, steeringLimit), color = 'red')

# ##################################################################
# 3次元プロット表示処理    
# ##################################################################
# プロット
fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
ax = fig.gca(projection='3d')
ax.set_aspect('auto')
sct, = ax.plot([], [], [], "o", markersize = 3)

# 描画を最大化し，グラフ範囲を正方形にしたい時はset_aspect('equal', 'datalim')にすると良い．
# ax.set_aspect('equal', 'datalim')

# タイヤのサイズを小さくして形状をリアル（真円）にしたい場合は下記のコードを憂苦にすると良い．
ax.tick_params(axis = 'x', length = 40.0)
ax.tick_params(axis = 'y', length = 40.0)
ax.tick_params(axis = 'z', length = 20.0)
ax.set_xlim(-2 * Diameter, 2 * Diameter)
ax.set_ylim(-2 * Diameter, 2 * Diameter)
ax.set_zlim(0, 4 * Diameter)

ax.set_xlabel("x")
ax.set_ylabel("y")
ax.set_zlabel("z")

ax.set_aspect('auto')

# ##################################################################
# 描画データの定義，描画処理
# ##################################################################
im = [] # フレーム更新の際に前回のプロットを削除するために用意
# imgif = [] # フレーム更新の際に前回のプロットを削除するために用意．gifアニメ用

# ##################################################################
# 回転軸の表示
# ##################################################################
# arrow(Point1, Point2, 'k')
# line= art3d.Line3D(Point1[0], Point1[1], Point1[2], Point2[0], Point2[1], Point2[2], color="k")
Point3 = [-1*(Point1[0]+Diameter * 1.2), Point1[1], Point1[2]+math.tan(thetaRad)*Diameter*1.2]
ax.plot([Point1[0], Point3[0]],[Point1[1], Point3[1]], [Point1[2], Point3[2]], color = "k")
# ax.add_line(line)

# ##################################################################
# 車輪の表示
# ##################################################################
# 車輪中心
ax.plot( [0], [0], [Diameter],marker="o",linestyle='None')
# 車輪
# Draw a circle on the y=0
q=Circle((0, Diameter), Diameter,color='red', fill=False)
ax.add_patch(q)
art3d.pathpatch_2d_to_3d(q, z=0, zdir="y")

# ##################################################################
# 接地点の表示
# ##################################################################
# 車輪データのテスト描画
# 周回後の最終位置にある車輪を描画する．
# 色は自動的に指定される．
ax.scatter( xfG, yfG, zfG, s = 40, alpha = 0.3, marker = ".")

# 描画データ属性の設定
def update(ifrm, xa, ya, za, xb, yb, zb):
    sct.set_data(xa[ifrm], ya[ifrm])
    sct.set_3d_properties(za[ifrm])
    sct.set_data(xb[ifrm], yb[ifrm])
    sct.set_3d_properties(zb[ifrm])

# 更新する内容
def _update_plot(i, fig, im, xa, ya, za):
    # 前回のフレーム内容を一旦削除
    if len(im) > 0:
        im[0].remove()
        im.pop()

    # print('i = ', i)
    # print('xa[i] = ', xa[i])
    # 複数の点を描画する際は，x，y，zを配列表記にしてscatter関数の引数にする．
    # im.append(ax.scatter( [xa[i], xb[i]], [ya[i], yb[i]], [za[i], zb[i]], s = 20, alpha = 0.3, color = 'c', marker = 'o'))
    im.append(ax.scatter(xa[i], ya[i], za[i], s = 20, alpha = 0.3, color = 'c', marker = 'o'))
    print('*')
    # im.append(ax.scatter( xa[i], ya[i], za[i], s = 20, alpha = 0.3, color = 'c', marker = 'o'))

# ##################################################################
# 描画データファイルの保存，描画処理
# ##################################################################
d = datetime.datetime.today()
FFig.savefig("Trail_"+d.strftime("%B%d日%A")+"Caster="+str(theta)+"_Wheel＝"+str(Diameter)+"_Offset="+str(Offset)+".png", dpi=100,transparent = False)
plt.show()

# アニメーション作成
# アニメーション機能は削除する
'''
ani = animation.FuncAnimation(fig, _update_plot, frames = nfr, fargs=(fig, im, xfG, yfG, zfG), interval=1000/fps)
# aniFront = animation.FuncAnimation(fig, _update_plot, nfr, fargs=(fig, im, xfG, yfG, zfG, xrG, yrG, zrG), interval=1000/fps)
#aniRear  = animation.FuncAnimation(fig, _update_plot, nfr, fargs=(fig, im, xrG, yrG, zrG), interval=1000/fps)

d = datetime.datetime.today()
FFig.savefig("Trail"+d.strftime("%B%d日%A")+"Caster="+str(theta)+"_Wheel＝"+str(Diameter)+"_Offset="+str(Offset)+".png", dpi=100,transparent = False)
plt.show()
'''
'''　アニメーション機能は削除する  
directory = '/Users/east/programming/Python/BicycleModeling/MovieGIF/'
fn = 'plot_BicycleTrajectory'+ str(datetime.datetime.now())

# ani.save(directory+fn+'.gif', writer='imagemagick')
ani.save(directory+fn+'.mp4', writer='ffmpeg', fps = fps)
'''