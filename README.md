# robust-control
ロバスト制御勉強用

- sampleの中はこちらのサイトを使用させてもらっています https://qiita.com/Yuya-Shimizu/items/f811317d733ee3f45623


# 注意
- 多入力、多出力をやろうとするとslycotというモジュールが必要で、そのインストールがpipだとできない
- ここを参考にwhlよりインストールする  
https://note.com/tomtom_0301/n/na8fedf6f87ee

が、以前うまくいかない・・・

- あきらめてanaconda 入れて下記を実行したら一発だったので、これをやりましょう
https://github.com/python-control/python-control
conda install -c conda-forge control slycot