# 2車線道路の車線変更を最適化する【交通シミュレータSUMO】
## 概要
数理最適化を用いて、自動運転車が車線変更をスムーズに行えるように、自動で車間距離を調整するシステムを作成した。
（詳しくは論文.pdf参照）

加えて、交通シミュレータSUMOを用いてそのシステムを検証した。（今回の部分）

## 実行環境
- windows 10 64bit

- Anaconda 4.13.0

- SUMO version 1.13.0

## 検証した結果
次の動画は交通シミュレータ上の車を最適化させた様子である。比較のため、最適化しない場合も載せた。

緑色の三角形が車線変更する予定のある車、黄色の三角形が車線変更する予定のない車である。

### 最適化をしない場合

https://user-images.githubusercontent.com/108399244/176835689-bb255541-6a08-48b0-8fa3-befeacbd04ce.mp4

車線変更の予定がある車(緑色の三角形)の横に十分なスペースが無く、車線変更が難しい。

### 最適化した場合

https://user-images.githubusercontent.com/108399244/176835032-5f443ed5-7c5c-4cab-94d0-6afe4496b5b6.mp4

車線変更の予定がある車(緑色の三角形)の横にスペースが作られており、車線変更をスムーズに行える。

## このシステムを作成した理由
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

まとめると、従来のように個々の車がそれぞれ運転のやり方を決めるより、ある一つのシステムが車の最適な運転を決めるほうが全体の交通流は効率的になる。

それが自動運転と車間通信を備えた車が走行する環境であれば可能になるので、その将来に備えて研究をしておくということ。

## 社会実装に向けての構想
想定する交通環境は、

- 自動運転車(レベル2以上)が走行する

- 自動運転車の多くがクラウドに無線接続している

の2点である。

道路上を走行する自動運転車は、自身の位置や速度・目的地やセンサーからの情報をクラウドに送信する。
クラウド上の交通調整システムはこれらの情報を基に、数秒後までの車の動きを最適化し、それを各自動運転車にフィードバックする。
各自動運転車はこの運転計画に従って道路を走行する。

## 最適化の数理モデル
このシステムには数理最適化が使われており、そのモデルの説明を記した。

### 最適化したい変数
最適化したい対象は時間経過に伴う車の動きである。
よって運転は物体の運動としてモデル化でき、時間 $t$ に対する車の位置、速度、加速度の関数で説明できる。

<img src="https://latex.codecogs.com/svg.image?\large&space;x_n(t),\&space;v_n(t),\&space;a_n(t),\&space;n\in\mathcal{N}" />
$\mathcal{N}$ は最適化する車の集合である。

しかし、これらの関数は連続量でありコンピュータで扱えないため、離散時間による時系列データに近似する。

<img src="https://latex.codecogs.com/svg.image?\large&space;\mathbf{x_n}=\begin{pmatrix}&space;x_n(t_0)\\&space;x_n(t_1)\\&space;x_n(t_2)\\&space;\cdot&space;\cdot&space;\cdot\\&space;x_n(t_H)\end{pmatrix},\&space;\mathbf{v_n}=\begin{pmatrix}&space;v_n(t_0)\\&space;v_n(t_1)\\&space;v_n(t_2)\\&space;\cdot&space;\cdot&space;\cdot\\&space;v_n(t_H)\end{pmatrix}" /> <img src="https://latex.codecogs.com/svg.image?\large&space;,\&space;\mathbf{a_n}=\begin{pmatrix}&space;a_n(t_0)\\&space;a_n(t_1)\\&space;a_n(t_2)\\&space;\cdot&space;\cdot&space;\cdot\\&space;a_n(t_H)\end{pmatrix},\&space;n\in\mathcal{N}" />

<img src="https://latex.codecogs.com/svg.image?\large&space;{\rm&space;where\&space;with}\&space;t_h&space;=&space;t_0&space;&plus;&space;h\Delta&space;t" />

ここで、 $\Delta t$ は微小時間、 $H$ は時間ステップ数である。

### 目的関数
最適化の目的は、平均速度を高く維持して燃費を低く抑えながら、車線変更する車に十分な車間距離を空けることである。

従って、平均速度・燃費・車線変更の3つの評価軸の和を目的関数とする。

<img src="https://latex.codecogs.com/svg.image?\large&space;{\rm&space;minimize}\&space;f_{cost}=w_1f_1&space;&plus;&space;w_2f_2&space;&plus;&space;w_3f_3" />
<!-- {\rm minimize}\ f_{cost}=w_1f_1 + w_2f_2 + w_3f_3 -->

ここで、 $f_1$ は速度評価関数、 $f_2$ は燃費評価関数、 $f_3$ は車線変更リスク評価関数であり、 $w1,\ w2,\ w3$ はそれぞれの重み付け(定数)である。

- 速度評価 $f_1$

<img src="https://latex.codecogs.com/svg.image?\large&space;f_1=&space;\sum_{h=1}^H&space;\sum_{n\in\mathcal{N}}(v_n(t_h)&space;-&space;v_{\rm&space;des})^2" />
<!-- f_1= \sum_{h=1}^H \sum_{n\in\mathcal{N}}(v_n(t_h) - v_{\rm des})^2 -->

$f_1$ は速度 $v_n$ が理想速度 $v_{\rm des}$ から離れるほど値が大きくなる。
理想速度から離れるほど渋滞を意味するので、交通の円滑さを評価できる。

<br>

- 燃費評価 $f_2$

<img src="https://latex.codecogs.com/svg.image?\large&space;f_2&space;=&space;\sum_{h=1}^H&space;\sum_{n\in\mathcal{N}}a_n^2(t_h)" />
<!-- f_2 = \sum_{h=1}^H \sum_{n\in\mathcal{N}}a_n^2(t_h) -->

$f_2$ は速度変化が大きく加減速が多いほど値が大きくなる。
車の燃費は加減速を繰り返すほど悪くなるため、加速度の分散で燃費を評価できる。

<br>

- 車線変更リスク評価 $f_3$

<img src="https://latex.codecogs.com/svg.image?\large&space;f_3&space;=&space;\sum_{h=1}^H&space;\sum_{p\in\mathcal{P}}&space;\sum_{q\in\mathcal{Q}}&space;(\theta_p&space;\vee&space;\theta_q)&space;e^{-\alpha(x_p(t_h)&space;-&space;x_q(t_h))^2}" />
<!-- f_3 = \sum_{h=1}^H \sum_{p\in\mathcal{P}} \sum_{q\in\mathcal{Q}} (\theta_p \vee \theta_q) e^{-\alpha(x_p(t_h) - x_q(t_h))^2} -->

ここで、 $\mathcal{P}$ は二車線道路上の右車線の車の集合で、 $\mathcal{Q}$ は左車線の車の集合である。 $\theta_n$ は車線変更する車なら1、しないなら0となるバイナリ変数であり、
 $\theta_p \vee \theta_q$ はその論理和を示す。

$f_3$ は車線変更する車とそれ以外の車の車間距離が開くほど値が小さくなることを意味する。
車線変更の危険度が横の車との車間距離の正規分布に従うと仮定して評価している。

### 制約条件
- 運動条件

運動の微分方程式より、 $x_n,\ v_n,\ a_n$ の関係は以下の式に定式化される。

<img src="https://latex.codecogs.com/svg.image?\large&space;x_n(t&plus;\Delta&space;t)&space;=&space;x_n(t)&space;&plus;&space;v_n(t)\Delta&space;t" />
<!-- x_n(t+\Delta t) = x_n(t) + v_n(t)\Delta t, -->

<img src="https://latex.codecogs.com/svg.image?\large&space;v_n(t&plus;\Delta&space;t)&space;=&space;v_n(t)&space;&plus;&space;a_n(t)\Delta&space;t" />
<!-- v_n(t+\Delta t) = v_n(t) + a_n(t)\Delta t, -->

<br>

- 速度・加速度制限

現実の交通規則による速度制限・加速度制限である。

<img src="https://latex.codecogs.com/svg.image?\large&space;v_{\rm&space;min}&space;\leq&space;v_n(t)&space;\leq&space;v_{\rm&space;max}&space;&space;" />
<!-- v_{\rm min} \leq v_n(t) \leq v_{\rm max}  -->

<img src="https://latex.codecogs.com/svg.image?\large&space;a_{\rm&space;min}&space;\leq&space;a_n(t)&space;\leq&space;&space;a_{\rm&space;max}&space;" />
<!-- a_{\rm min} \leq a_n(t) \leq  a_{\rm max} -->

<br>

- 衝突回避制約

先行車と衝突しないことを示す制約である。

<img src="https://latex.codecogs.com/svg.image?\large&space;a_f&space;\leq&space;&space;a_{\rm&space;cfm}(x_l,\&space;x_f,\&space;v_l,\&space;v_f)" />
<!-- a_f \leq  a_{\rm cfm}(x_l,\ x_f,\ v_l,\ v_f) -->

ここで、 $x_l,\ v_l$ は先行車の位置と速度、 $x_f,\ v_f$ はその追従車の位置と速度である。 $a_{\rm cfm}$ は車追従モデルで計算される加速度である。

交通シミュレーションでは、先行車の速度や車間距離から安全が確保できる加速度を計算するモデルが存在する。そのモデルで計算される加速度 $a_{cfm}$ 以下であれば、前方車と衝突しないことが保証される。車追従モデルには様々なモデルがあるが、今回はIDM(intelligent driver model)というモデルを使った。

<img src="https://latex.codecogs.com/svg.image?\large&space;a_{\rm&space;cfm}&space;=&space;a_{\rm&space;max}\left&space;(&space;1&space;-&space;\left(\frac{v_f}{v_{\rm&space;des}}&space;\right)^4&space;-&space;\left(\frac{s}{g}\right)^2&space;\right&space;)" />
<!-- a_{\rm cfm} = a_{\rm max}\left ( 1 - \left(\frac{v_f}{v_{\rm des}} \right)^4 - \left(\frac{s}{g}\right)^2 \right ), -->

<img src="https://latex.codecogs.com/svg.image?\large&space;g&space;=&space;x_l&space;-&space;x_f&space;-&space;l" />
<!-- g = x_l - x_f - l, -->

<img src="https://latex.codecogs.com/svg.image?\large&space;s&space;=&space;g_{\rm&space;min}&space;&plus;&space;v_f&space;T&space;&plus;&space;\frac{v_f(v_f&space;-&space;v_l)}{2\sqrt{|a_{\rm&space;max}&space;a_{\rm&space;min}}|}" />
<!-- s = g_{\rm min} + v_f T + \frac{v_f(v_f - v_l)}{2\sqrt{|a_{\rm max} a_{\rm min}}|}, -->

