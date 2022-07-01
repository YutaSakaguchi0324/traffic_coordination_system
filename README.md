# 自動運転と車間通信のための最適交通調整システムの開発
## 概要
2車線道路において、車線変更をスムーズに行えるように自動で車間距離を調整するシステムを作った。
詳しくはdocument.pdf参照

## 研究背景と研究目的
近年、自動車は自動化が進み、GPSやカメラ等のセンサーが搭載されるようになり、人を介さない運転が可能になりつつある。
また、5Gを始めとした、車両間・車両と外部間の通信に利用できる技術も発達してきている。
これにより、自分の現在位置やその周辺の情報を収集し、他の車や外部と情報を共有できるようになる。
すなわち、自動運転車は自身周辺の情報だけでなく、より広範囲の情報を活用してより良い運転制御が実現できる。
例えば、

- 500m先で渋滞が発生しているので、別のルートを取る

- 前の車がブレーキを踏む予定なので、前もって減速しておく

- 横の車が割り込みたがっているので、進路を譲る

のような車同士の協調制御が可能になり、これらの決定を全体の交通への影響を考慮しながら可能になる。

しかし、一般に交通流は複雑であり、車両情報を効果的に活用し、協調的な交通管理システムを開発するのは困難である。
このような課題を克服するために、様々な研究が試みられている。

## 最適交通調整システムの構想
### 想定する環境

- 完全自動運転車のみが走行する

- すべての自動運転車はクラウドに無線接続している

道路上を走行する自動運転車は、自身の位置や速度・目的地やセンサーからの情報をクラウドに送信する。
クラウド上の交通調整システムはこれらの情報を基に、数秒後までの車の動きを最適化し、それを各自動運転車にフィードバックする。
各自動運転車はこの運転計画に従って道路を走行する。

![サイバーフィジカル](https://user-images.githubusercontent.com/108399244/176767030-cae9156e-24bb-438d-bc8b-638bab054acf.png)

最適化の計算の際、道路上すべての車を同時に最適化するのは計算リソースの観点から得策ではないため、
車数台からなるいくつかのグループに分割して、局所的に最適化を繰り返す。

## 最適化の数理モデル
詳しくはdocument.pdf参照

## 最適交通調整システムをpythonで構築する
本研究では車線変更に着目して、車線変更をスムーズに行えるように車間距離を調整するシステムを構築した。

論文にはMATLABによる数値シミュレーションの結果が載っているが、実用化にはより多くのパターンと
現実の車との相互作用を伴う実験が必要である。

現実の車を使って実験することは不可能なので、代わりに交通シミュレータSUMOを用いてシミュレーション上の車で実験する。
SUMOにはpython用のインターフェース(TraCI)が存在し、交通調整システムをpythonで構築することで実験環境を作れる。

## 実験結果
次の動画は交通シミュレータ上の車を最適化させた様子である。比較のため、最適化しない場合の様子も載せた。

緑色の三角形が車線変更する予定のある車、黄色の三角形が車線変更する予定のない車である。

### *最適化をしない*場合のシミュレーション

https://user-images.githubusercontent.com/108399244/176835689-bb255541-6a08-48b0-8fa3-befeacbd04ce.mp4

最適化しない場合、車線変更の予定がある車(緑色の三角形)の横に十分なスペースが無く、車線変更が難しい。

### *最適化した*場合のシミュレーション

https://user-images.githubusercontent.com/108399244/176835032-5f443ed5-7c5c-4cab-94d0-6afe4496b5b6.mp4

最適化した場合、車線変更の予定がある車(緑色の三角形)の横にスペースが作られていることが確認できる。
