# 2車線道路の車線変更を最適化する【交通シミュレータSUMO】
## 概要
数理最適化を用いて、車線変更をスムーズに行えるように自動で車間距離を調整する交通システムを作成した。

加えて、交通シミュレータでそのシステムを検証した。

## 実行環境
- windows 10 64bit

- Anaconda 4.13.0

- SUMO version 1.13.0

## 作成した理由
大学の研究のため

## この研究の意義
近年、自動運転技術や、5Gを始めとした通信技術の発展が著しい。
将来的に、自動車は自分の位置やその周辺情報を収集し、他の車や外部と情報を共有できるようになる。
すなわち、自動運転車は自身周辺の情報だけでなく、より広範囲の情報を活用してより良い運転制御が実現できる。
例えば、

- 500m先で渋滞が発生しているので、別のルートを取る

- 前の車がブレーキを踏む予定なので、前もって減速しておく

- 横の車が割り込みたがっているので、進路を譲る

のような車同士の協調制御が可能になり、これらの決定を全体の交通への影響を考慮しながら可能になる。

大雑把にいうと、従来のように個々の車がそれぞれ運転のやり方を決めるより、ある一つのシステムが車の最適な運転を決めるほうが全体の交通流は効率的になる。

それが自動運転とコネクテッドを備えた車が走行する環境であれば可能になるので、その将来に備えて研究をしておくということ。

## 最適交通調整システムの構想
### 想定する交通環境

- 完全自動運転車のみが走行する

- すべての自動運転車はクラウドに無線接続している

道路上を走行する自動運転車は、自身の位置や速度・目的地やセンサーからの情報をクラウドに送信する。
クラウド上の交通調整システムはこれらの情報を基に、数秒後までの車の動きを最適化し、それを各自動運転車にフィードバックする。
各自動運転車はこの運転計画に従って道路を走行する。

## 最適化の数理モデル

### 最適化したい変数

### 目的関数
<img src="https://latex.codecogs.com/svg.image?minimize\&space;f_{\rm&space;cost}&space;=&space;f_1&space;&plus;&space;f_2&space;&plus;&space;f_3" />

<img src="https://latex.codecogs.com/svg.image?{\rm&space;Minimize}\&space;f_{\rm&space;cost}&space;=&space;f_1&space;&plus;&space;f_2&space;&plus;&space;f_3" />

<img src="https://latex.codecogs.com/svg.image?minimize\&space;f_{\rm&space;cost}&space;=&space;f_1&space;&plus;&space;f_2&space;&plus;&space;f_3" />

$$ f_1=\sum_{h=1}^H \sum_{n\in\mathcal{N}} w_1(v_n(t)-v_{des})^2 $$

<img src="https://latex.codecogs.com/svg.image?f_1&space;=&space;\sum_{h=1}^{H}&space;\sum_{n\in\mathcal{N}}&space;w_1&space;(v_n(t)&space;-&space;v_{\rm&space;des})^2" />

$$ f_2=\sum_{h=1}^H \sum_{n\in\mathcal{N}} w_2a_n^2(t) $$

$$ f_3=\sum_{h=1}^H \sum_{p\in\mathcal{P}} \sum_{q\in\mathcal{Q}} w_3(\theta_p + \theta_q)e^{-\alpha(x_p(t)-x_q(t))^2} $$

### 制約条件

## pythonで最適交通調整システムを構築する
論文にはMATLABによる数値シミュレーションの結果が載っているが、実用化にはより多くのパターンと
現実の車との相互作用を伴う実験が必要である。

現実の車を使って実験することは不可能なので、代わりに交通シミュレータSUMOを用いてシミュレーション上の車で実験する。
SUMOにはpython用のインターフェース(TraCI)が存在し、交通調整システムをpythonで構築することで実験環境を作れる。

## 実験結果
次の動画は交通シミュレータ上の車を最適化させた様子である。比較のため、最適化しない場合も載せた。

緑色の三角形が車線変更する予定のある車、黄色の三角形が車線変更する予定のない車である。

### 最適化をしない場合

https://user-images.githubusercontent.com/108399244/176835689-bb255541-6a08-48b0-8fa3-befeacbd04ce.mp4

車線変更の予定がある車(緑色の三角形)の横に十分なスペースが無く、車線変更が難しい。

### 最適化した場合

https://user-images.githubusercontent.com/108399244/176835032-5f443ed5-7c5c-4cab-94d0-6afe4496b5b6.mp4

車線変更の予定がある車(緑色の三角形)の横にスペースが作られていることが確認できる。
