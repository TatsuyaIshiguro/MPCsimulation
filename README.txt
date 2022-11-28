・ソリューションMPCsimulation内の構成
	－プロジェクトLauncher
	－プロジェクトOptimization

	Launcher.cppでコース読み込み、初期状態の設定	→	Optimization.exeを起動して最適化計算
										↓↑
									　　　共有メモリ
										↓↑
									Launcher.cppで状態を更新

　	・Launcher.cpp
	シミュレーションの初期状態とコース用csvのパスの設定はLauncher.cppのエントリーポイント付近
	最適化計算に関する設定はParameter.csv
	Launcher.cppの上部のOptimization.exeのパスの指定
	setting.hでコースの変更可能（OA or Sine or Loadcsv）

	・Optimization.cpp
	Carsimを使っておらず最適化における予測モデルとシミュレータ用のモデルで分けることは行っていない（DBM to DBM）
	NoiseMake.hにおいてノイズに関する設定が可能（デフォルトでは各ノイズの標準偏差が0）
	デフォルトではIPMで最適化を実行している、parameter.csvで解法をSQPに変更可能

	データ保存用のフォルダDataにMPCという名前のフォルダを作成しておくこと
	スタートアッププロジェクトはLauncher側を指定してあるので自動的にLauncher側が実行される