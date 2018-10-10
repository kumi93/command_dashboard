README.txt

・概要
上肢ロボットおよび3対6筋2自由度アームの各筋肉の目標圧力値をPublishするGUIです

・準備
Arneさん（https://github.com/arne48?tab=repositories）のarl_commonsとarl_hw_msgsをcatkin_ws/src内に導入して，arl_commons内のpyarlarm_example.pyが正常に動作することを確認して下さい

・導入方法
~/catkin_ws/srcの下でgit clone
catkin_makeなどをして
rosrun command_dashboard command_dashboard.py
で起動します

・使い方
バーまたはEditボタンで各筋肉の目標圧力値を設定
Publishボタンでパブリッシュします
Resetでロボットを初期状態に戻します

csvファイルで目標圧力値の保存・読み込みができます

各ボタンにはショートカットキーが設定されています
GUI内のHelpで確認できます

接続先のロボットの判定は$ROS_MASTER_URIを取得して行っています
接続先が不明な場合は上肢ロボット用のGUIを表示します
