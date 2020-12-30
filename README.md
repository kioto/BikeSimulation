# BikeSimulation
Realize how the bike turns
このPythonソースコードは，オートバイや自転車がどの様に曲がるか，そのメカニズムを明らかにする為のものです．
1．期待している処理内容
1.1　パラメータ
　（1）パラメータ
 ・キャスター角：θ          変数名＝theta
 ・オフセット：　　         変数名＝Offset
 ・車輪径（前輪後輪共通）：　 変数名＝Diameter
 ・回転軸下端              変数名＝Point1
 ・回転軸上端              変数名＝Point2
 ・回転軸                 変数名＝RotateCenter
 ・前輪中心               変数名＝Wheel_O
 
1.2　処理内容（本来実現したい処理）
（1）omega=0
（2）前輪を回転軸を中心にdeltaomegaだけ回転させる
（3）回転後の前輪と後輪の接平面を求める
    条件：
    ・前輪の中心と前輪接点のベクトル⊥接平面
    ・後輪の中心と後輪接点のベクトル⊥接平面
    ・接平面はX軸に平行
（4）回転後の前輪の接点を求める
（5）omega = deltaomega + omega
（6）上記（1）〜（6）をomega=360になるまで繰り返す
※注意点
 ・本来は前輪と後輪の接地面（接平面）が変化することになるので，その接平面上のX座標，Y座標をプロットする必要があります．
 ・但し，接点は前輪と後輪の2点しかない為，接平面は同定されません．
 ・これより，前輪の回転軸（シャフト）は直立していると仮定します．
  

2．ファイル名「Opus_Handle180Rotate.py」
　これはnumpyを用いています．
　シミュレーション内容：
  （1）ハンドルを回した時の前輪の接地点がどの様に移動するのか，XY平面（地面）上の軌跡を算出しています．
  （2）このコードは，単純にハンドルを回転した時の前輪の最下点のX座標，Y座標をプロットしたものになっています．
3．ファイル名「EtudeSympyHandle360Rotate.py」
　「Opus_Handle180Rotate.py」で実現した内容をsympy+numpyを用いることでもっとコードが簡単にできないか試しました．
 しかしエラーメッセージ（Attribute Error）が出てキチンと動作しません．

