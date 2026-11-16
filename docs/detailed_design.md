# Skin Physics 詳細設計書

- 文書バージョン: 1.0
- 状態: 初版作成・詳細設計レビュー待ち
- 対応要件: requirements.md 1.1
- 対応基本設計: basic_design.md 1.1
- 対象: Skin Physics
- 実装対象API: Direct3D 11 / HLSL Shader Model 5
- 言語・ビルド: C++17 / Visual Studio .sln・.vcxproj

## 1. 本書の目的と設計判断

本書は基本設計で残したアルゴリズム、GPUデータ構造、処理順序、数値処理、エラー処理を実装可能な粒度へ具体化する。実装開始の承認文書ではない。レビューで合格するまで実装を開始しない。

### 1.1 初期版の方式決定

| 領域 | 初期採用方式 | 比較・代替候補 |
|---|---|---|
| Soft Body | 四面体Corotational FEM、陽的内部力 + 半陰的Euler | XPBDひずみ制約、Shape Matching |
| Cloth Stretch | XPBD距離制約、Graph Coloring | Gather Jacobi |
| Cloth Shear | XPBD距離制約、Gather Jacobi | Graph Coloring |
| Cloth Bend | XPBD二面角制約、Gather Jacobi | Isometric Bending |
| 共通接触 | XPBD不等式接触制約、Gather Jacobi | Speculative Contact |
| Broadphase | GPU Spatial Hash + Count/Scan/Fill | Radix Sort Grid、階層グリッド |
| Narrowphase | VT + EE | Proxy SDFは将来追加 |
| CCD | Swept AABB + 選別VT/EE連続判定 | Speculative Contact |
| 描画対応 | Offline Embedding + GPU補間 | GPU最近傍検索は採用しない |
| 四面体生成 | fTetWildをオフライン候補として使用 | 他の四面体化器を比較 |

Soft Bodyを初期版でFEMとXPBDの混成にしない。Corotational FEMは力ベースの弾性モデルとして独立実装し、Clothと接触はXPBDとする。これにより、材料内部の応力計算と接触拘束の役割を分離する。

### 1.2 数値単位

内部単位系はSIを採用する。

- 長さ: meter
- 質量: kilogram
- 時間: second
- 力: newton
- 密度: kg/m³
- 弾性率: pascal
- 重力加速度: m/s²

Importerで単位変換を行い、ランタイム内部でモデルごとの暗黙スケールを持たせない。モデルの単位情報が不明な場合はユーザーに入力を要求する。

## 2. 初期対応アセット仕様

### 2.1 Importer

初期対応形式を以下とする。

- glTF 2.0 / GLB: スケルトン、スキニング、アニメーション、メッシュ、マテリアルを含む主形式。
- OBJ: 静的メッシュの入力・検証用。スケルトン・アニメーションは対象外。

Importerは共通内部表現へ変換する。外部Importerライブラリを利用する場合は、ライセンス、依存関係、C++17 / MSVC対応を確認し、Importer層の内部に閉じ込める。ランタイム物理処理からImporter実装を参照しない。

### 2.2 共通アセット

共通モデルは以下を持つ。

- Mesh単位の頂点・インデックス・サブメッシュ
- 頂点位置、法線、接線、UV、カラー等
- Bone hierarchy、bind pose、bone weights
- Animation clip、keyframe、補間方式
- Material slot
- Meshごとの物理方式とCollision設定
- 単位・座標系・軸変換情報

座標系変換はImporterで一度だけ行い、物理・描画・アニメーションの内部座標系を統一する。三角形の表裏と法線方向も変換後に再検査する。

## 3. CPU / GPU責務

### 3.1 CPU側

CPUは以下を担当する。

- アセット読み込み、入力検査、Physics Mesh生成・キャッシュ
- 初期隣接リスト、制約リスト、Graph Coloringの構築
- アニメーション評価とボーン行列更新
- GPUリソース生成・破棄、定数・制御データ更新
- 固定時間刻みの進行制御
- 非同期診断カウンタ・エラー状態の確認
- JSON入出力、試験環境制御

CPUは通常フレームごとの物理頂点位置・速度を読み戻さない。GPU診断カウンタの読み戻しは遅延可能な非同期経路に限定し、同期Readbackで描画を停止させない。

### 3.2 GPU側

GPUは以下を担当する。

- Skinning済みアニメーション目標位置の生成、またはCPUから渡されたボーン行列によるSkinning
- Soft Bodyの変形勾配・回転・応力・内部力計算
- Cloth制約解決
- 予測位置・速度更新
- Broadphase / Narrowphase / CCD
- 接触制約の解決
- Render Mesh位置・法線・接線更新
- GPUタイムスタンプ計測対象Pass

### 3.3 GPUデータ形式

基本データはStructured Bufferを使用する。頂点位置・速度はfloat3またはfloat4、インデックスはuint、材料値はfloat群を基本とする。アラインメントと帯域を計測し、float4への統一が有利な場合はパディングを許容する。

全バッファについて要素数、stride、最大容量、生成元、更新頻度、読み書きPassを定義する。CPU / GPU双方の構造体はサイズとオフセットをstatic_assert等で検証する。

## 4. 物理頂点・要素データ

### 4.1 共通頂点状態

概念上の共通頂点状態:

```cpp
struct physics_vertex_state
{
    float3 position;
    float3 previous_position;
    float3 velocity;
    float3 predicted_position;
    float inverse_mass;
};
```

実際のCPU構造体とGPU構造体は用途別に分割してよい。固定頂点データと毎ステップ更新データを別バッファに置き、不要な転送・書込みを避ける。

- inverse_mass = 0: 固定頂点
- inverse_mass > 0: 動的頂点
- 負値、NaN、Infinity: アセット構築時または更新時にエラー

### 4.2 Soft Body四面体

四面体ごとに4頂点インデックス、Rest Pose逆行列、Rest Volume、材料ID、品質フラグを保持する。

Rest Pose位置を x0..x3 とし、以下をオフライン生成する。

```text
Dm = matrix(x1 - x0, x2 - x0, x3 - x0)
V0 = abs(determinant(Dm)) / 6
Bm = inverse(Dm)
```

をオフライン生成する。Rest Volumeが閾値以下、または逆行列が不安定な四面体は採用しない。頂点順序と符号付き体積を統一し、反転要素を検出する。

### 4.3 Cloth制約

Clothの制約はオフライン構築し、以下を保持する。

- Stretch: 辺両端、Rest Length、Compliance、λ
- Shear: 三角形内の対角・せん断を表す頂点組、Rest Length、Compliance、λ
- Bend: 隣接三角形の共有辺と対向頂点4点、Rest Dihedral Angle、Compliance、λ
- Constraint type、Material ID、頂点ウェイト

同じ物理的拘束を重複生成しない。非多様体辺、境界辺、退化三角形はBend制約生成対象から除外し、診断数を記録する。

## 5. Soft Body Solver詳細

### 5.1 採用モデル

初期版は四面体Corotational Linear Elasticityを採用する。各四面体の変形勾配から局所回転を取り出し、剛体回転に対する不要な弾性抵抗を抑えた応力を求める。

四面体の現在位置を x0..x3 とすると、


```text
D_s = [x_1-x_0, x_2-x_0, x_3-x_0]
```



```text
F = D_s B_m
```


で変形勾配 F を得る。

極分解:

```text
F = R * S
R_next = 0.5 * (R + transpose(inverse(R)))
```

Rは回転行列、Sは対称変形行列とする。RはNewton polar iterationで近似する。初期値はFとし、固定回数反復する。det(F)が小さい、逆行列が不安定、またはdet(F)が負となる場合は通常更新を行わず、異常要素フラグを記録して反転回復処理へ送る。反転要素に対する回復応力の式は別Passで定義し、NaNを伝播させない。

### 5.2 材料パラメータ

ヤング率 E、ポアソン比 nu、密度 rho、減衰係数を材料単位で保持する。

```text
mu = E / (2 * (1 + nu))
lambda_lame = E * nu / ((1 + nu) * (1 - 2 * nu))
```

νは非圧縮極限に近づけすぎない。初期UIの許容範囲は0.0～0.45とし、0.49等の高ポアソン比を直接指定するモードは初期版に含めない。材料パラメータはSI単位で入力し、メッシュスケール変更時に材料値を暗黙変更しない。

### 5.3 応力・内部力

初期版のCorotational応力を次で定義する。

```text
J = determinant(F)
P = 2 * mu * (F - R) + lambda_lame * (J - 1) * cofactor(F)
```

cof(F)は余因子行列。四面体のRest Shape Function勾配を用い、頂点ごとの内部力を計算する。

```text
f_internal_i = -V0 * P * grad_rest(N_i)
```

4頂点分の力を四面体単位で計算し、頂点単位のGatherで集約する。四面体から頂点への浮動小数点Atomic Addは使わない。

### 5.4 時間積分

初期版は半陰的Eulerを採用する。

```text
velocity_next = velocity + delta_t * inverse_mass * (force_external + force_internal)
position_next = position + delta_t * velocity_next
```

減衰は指数減衰または速度比例減衰のどちらかに統一し、時間刻み依存性を避けるため係数の意味を文書化する。固定頂点は位置・速度をアニメーション目標に拘束する。

陽的内部力は高剛性・大要素で不安定になりやすいため、サブステップを必須とする。初期値は固定時間刻み 1/60 s、Soft Bodyサブステップ4を基準とし、1/120～1/240 s相当まで試験できるようにする。安定性を満たさない材料値は警告し、時間刻み・密度・要素品質を確認させる。無制限に自動サブステップを増やさない。

### 5.5 体積保持と形状復元

体積保持はλ項による体積応答で表現し、E・νと独立した調整が必要な場合は材料パラメータのスケール係数として導入する。Rest Shapeへの復元はCorotational弾性エネルギーが担う。体積係数と形状復元係数を同一パラメータにまとめない。

局所へこみの回復を検証し、単なる全体体積維持だけで形状復元を代用しない。

### 5.6 Soft Body Gather

四面体ごとに4頂点への内部力寄与を固定4スロットで出力する。オフライン生成した頂点→四面体のCSR隣接表を用い、頂点Dispatchで関連四面体寄与を総和する。


```text
f_i = sum_{t in T(i)} f_{t,i}
```


力は総和であり、単純な隣接数平均を行わない。頂点の質量は四面体体積と密度から質量集中法で配分し、頂点逆質量を生成する。共有頂点に重複する四面体質量を二重計上しない。

## 6. Cloth Solver詳細

### 6.1 XPBD基本式

制約関数 C(x) = 0、Compliance alpha、サブステップ delta_t、制約lambda_constraintを用いる。

```text
alpha_tilde = alpha / (delta_t * delta_t)
delta_lambda = (-C(x) - alpha_tilde * lambda_constraint)
               / (sum_i(inverse_mass_i * dot(gradient_i, gradient_i)) + alpha_tilde)
lambda_next = lambda_constraint + delta_lambda
delta_position_i = inverse_mass_i * gradient_i * delta_lambda
```

w_iは逆質量。分母が極小、入力が非有限値、全頂点が固定の制約は補正を出さず診断カウンタを増やす。

制約λはサブステップ開始時にリセットし、同一サブステップ内の反復で保持する。Warm Startを行う場合は前サブステップのλをそのまま使わず、時間刻み比率を考慮した再スケールを検証してから有効化する。初期実装ではWarm Startを無効とする。

### 6.2 Stretch

辺(i,j)の制約:

```text
C_stretch = length(x_i - x_j) - rest_length
gradient_i = normalize(x_i - x_j)
gradient_j = -gradient_i
```

nは辺方向単位ベクトル。Rest Length L0はRest Poseから算出する。StretchはGraph Coloringを基本方式とする。同色内の制約が頂点を共有しないことを構築時に検証し、色単位でDispatchして位置を更新する。

### 6.3 Shear

三角形のせん断は対角距離制約を基本とする。三角形を構成する辺・対角関係からRest Poseの距離を保存し、変形時に距離制約を適用する。ShearはGather Jacobiで解く。三角形分割や対角選択が結果へ偏りを与えるため、対称化方式を試験する。

### 6.4 Bend

隣接三角形の共有辺を挟む二面角 thetaを計算する。

```text
C_bend = wrap_angle(theta - rest_dihedral_angle)
```

勾配は解析式を使用する。共有辺長や三角形面積が極小の場合は制約を無効化する。BendはGather Jacobiで解く。二面角の符号・wrap範囲を一貫させ、法線反転を検出する。

### 6.5 Graph Coloring

制約グラフは頂点共有を競合とする。Greedy Coloringで色を割り当て、同色制約の頂点重複がないことを検証する。色数、最大色サイズ、色別Dispatch数をアセット診断に保存する。色数が設定上限を超えた場合はGather Jacobiへ切り替える。

### 6.6 Gather Jacobiの補正集約

制約ごとにXPBD式でdelta_lambdaを計算し、頂点ごとの補正寄与を固定スロットまたは可変長寄与バッファへ出力する。頂点→制約CSRを用いて頂点が寄与を総和する。

```text
delta_position_sum_i = sum_over_incident_constraints(delta_position_constraint_i)
position_next_i = position_current_i + omega * delta_position_sum_i
```

初期値 ω=0.5を試験開始値とし、制約密度・反復数によって検証する。制約寄与数で無条件に平均化しない。ωは安定化パラメータであり、Complianceや材料硬さとは別に管理する。Graph ColoringとGatherで結果が異なるため、同じ試験条件で収束・伸び誤差・GPU時間を比較する。

## 7. 接触制約

### 7.1 接触の一般式

接触制約は非貫通条件 C(x) >= 0 とし、lambda_constraint >= 0を満たす不等式XPBDとして解く。

```text
delta_lambda = (-C(x) - alpha_tilde * lambda_constraint)
               / (sum_i(inverse_mass_i * dot(gradient_i, gradient_i)) + alpha_tilde)
lambda_next = max(0, lambda_constraint + delta_lambda)
applied_delta_lambda = lambda_next - lambda_constraint
```

実際に加える補正は λ更新差分を用いる。法線は接触点の相対位置から決定し、ゼロ距離では前フレーム法線または安定した幾何法線を使用する。接触厚みは両形状の厚みの和として扱う。

### 7.2 Vertex-Triangle

頂点 p と三角形頂点 a,b,c の最近接点を重心座標 beta0,beta1,beta2で表す。

```text
q = beta0 * a + beta1 * b + beta2 * c
C_vt = dot(normal, p - q) - contact_thickness
denominator = wp + beta0^2 * wa + beta1^2 * wb + beta2^2 * wc + alpha_tilde
```

dは接触厚み。勾配は

- p: +n
- a: -β0 n
- b: -β1 n
- c: -β2 n

とする。分母は


```text
w_p + betaeta_0^2w_a+betaeta_1^2w_b+betaeta_2^2w_c+tilde_ilde{alpha}
```


となる。接触補正は双方の頂点へ逆質量と重心座標に従って配分する。

### 7.3 Edge-Edge

辺Aの最近接点を (1-s)*a0+s*a1、辺Bを (1-t)*b0+t*b1とする。

```text
qA = (1 - s) * a0 + s * a1
qB = (1 - t) * b0 + t * b1
C_ee = dot(normal, qA - qB) - contact_thickness
```

勾配はA側に +(1-s)n、+s n、B側に -(1-t)n、-t nを配分する。分母は各勾配の二乗と逆質量の積の総和にCompliance項を加える。

線分が平行・退化する場合は安定した最近接点計算へフォールバックし、NaNや無限大を出さない。

### 7.4 接触重複排除・寿命

接触キーはメッシュID、プリミティブID、接触タイプ、必要に応じて頂点IDを含む正規化キーとする。重複候補は接触解決前に排除する。毎フレーム変化する接触点の対応付けは、プリミティブキーと法線・接触点距離で行う。

初期版では接触λをフレーム間で保持しない。サブステップ内の反復のみ保持する。接触数が急増した場合は候補数・重複数を記録する。

### 7.5 摩擦

法線接触を解いた後、接触点の相対接線変位 delta_u_tを計算する。接線方向の補正を求め、Coulomb上限

```text
length(lambda_tangent) <= friction_coefficient * lambda_normal
```

でクランプする。μ_fは摩擦係数。静止摩擦・動摩擦を初期版で分離せず、単一係数から開始する。接線方向が不定な場合は摩擦補正を行わない。

### 7.6 反発

反発は位置補正に混ぜず、接触解決後の速度処理で適用する。接触法線方向の相対速度が接近方向である場合のみ、反発係数eを用いて法線速度を更新する。初期版ではe=0を既定値とし、反発を必要とする試験でのみ有効にする。

## 8. Broadphase / Narrowphase

### 8.1 Spatial Hashセル

セル座標は

```text
cell_coord = floor(position / cell_width)
```

とし、cell_widthはセル幅。とし、hはセル幅。セル幅の初期候補は代表衝突Proxyの平均辺長または代表接触厚みから設定し、複数値をベンチマークする。固定値を全アセットに強制しない。

Hash値と整数セル座標を両方保持する。Hash値一致だけで同一セルと判断しない。

### 8.2 粒子・三角形登録

- 粒子は現在位置の1セルへ登録する。
- 三角形はAABBが重なる全セルへ登録する。
- CCD対象では前後位置を含むSwept AABBを用いる。
- 三角形の登録セル数がLarge Primitive閾値を超える場合、通常セル登録から除外しLarge Primitive配列へ登録する。
- Large Primitive配列が上限を超えた場合は階層グリッド経路を使用するか、ステップをエラーにする。全粒子との無制限総当たりへフォールバックしない。

### 8.3 Count / Prefix Sum / Fill

GPU処理は以下の順とする。

1. Primitiveごとの登録セル数をCount。
2. Exclusive Prefix Sumで出力オフセットを算出。
3. 合計登録数と出力容量を比較。
4. 容量内ならFillでセル登録を出力。
5. Hashとセル座標をキーにSortまたはグループ化。
6. 同一セル内の粒子・三角形を列挙して候補を生成。
7. 候補キーを正規化し、重複排除。

Prefix Sumはブロック内Scan、ブロック合計Scan、オフセット加算の階層構成とする。Dispatch上限を超えるデータは複数バッチに分割する。

### 8.4 Overflow

全ての可変長出力にcapacity、required_count、overflow flagを持つ。overflow時はそのフレームの物理状態を確定しない。GPU処理を停止し、遅延読み戻しした診断情報をCPUで確認し、容量を増やして再実行する。再試行上限を超えた場合は明示的なSimulation Errorとする。

容量を切り詰めて物理を継続する動作は禁止する。エラー発生フレームの位置バッファは更新前の状態へ戻せるよう、予測位置と確定位置を分離する。

### 8.5 Narrowphase

VTは頂点と三角形の最近接点、面の向き、厚みを検査する。EEは線分間最近接点を求める。Broadphase候補はAABBでさらに絞り、自己衝突では隣接関係テーブルにより除外する。

近傍除外は同一面・共有辺・共有頂点に加え、Soft Bodyでは同一四面体または隣接四面体の境界近傍を考慮する。除外範囲を広げすぎると折り畳み部の衝突が抜けるため、除外レベルを設定・可視化する。

## 9. CCD

### 9.1 適用判定

頂点または辺の1サブステップ移動距離が接触厚みの一定割合を超える場合、または前位置と予測位置の離散判定で接触が検出されないまま相対軌道が接近している場合にCCD候補とする。

初期判定:

```text
length(relative_displacement) > eta * contact_thickness
```

ηは初期値0.5、dは接触厚み。これは対象選別の閾値であり、衝突の正しさを保証するものではない。

### 9.2 Swept Broadphase

頂点・辺・三角形の前位置と予測位置を包含するSwept AABBを生成する。Swept AABBが重なる候補だけを連続Narrowphaseへ渡す。

### 9.3 Continuous Narrowphase

VTでは頂点軌道と三角形運動の相対運動を考慮し、時刻t∈[0,1]での接触厚み到達を探索する。EEも両辺の運動を含む距離最小化を行う。

初期版は保守的なTime of Impact探索を用い、区間を細分化して接触可能区間を絞る。探索回数上限、許容時間誤差、最小進行時間を持つ。退化・平行ケースはSpeculative Contactへ切り替える。TOIでサブステップを分割するか、TOI位置に接触制約を作るかは比較試験し、初期実装はSpeculative Contactを優先して安定性を評価する。

CCDが対象選別から漏れたケースを検出するため、高速移動テストでは全対象に対する参照判定を行い、見逃し率を測定する。

## 10. アニメーション連携

### 10.1 目標位置

各物理頂点に対応するRest位置をスキニングし、現在のアニメーション目標位置 x_anim_current を得る。前回目標位置 x_anim_previousから

```text
velocity_animation = (x_anim_current - x_anim_previous) / delta_t
```

を計算する。目標位置と目標速度はGPUバッファへ格納する。

### 10.2 追従

Animation Trackingは頂点ごとのComplianceまたは追従係数として持つ。ClothではXPBD追従制約として扱う。Soft Bodyでは外部追従力または目標速度との速度ブレンドとして扱い、材料弾性と混同しない。

追従係数を強くするとアニメーションへ近づき、弱くすると物理慣性が残る。パラメータの意味と単位はUI・JSONで統一する。

### 10.3 不連続イベント

Clip切替、Teleport、Reset、時間巻き戻し時にAnimation Discontinuityフラグを立てる。設定に応じて以下を選ぶ。

- Reset: 物理位置・速度をアニメーション目標へ再初期化。
- Preserve Velocity: 物理速度を維持し、目標速度差分だけリセット。
- Blend: 一定時間をかけて目標位置・速度へ移行。

初期版の既定値はClip切替でBlend、Teleport / ResetでResetとする。フラグの立て忘れを検出するため、目標速度が設定上限を超えた場合は警告する。

## 11. Render Feedback

### 11.1 Soft Body埋め込み

四面体内部頂点は4頂点の重心座標で位置を補間する。

```text
x_render = beta0 * x0 + beta1 * x1 + beta2 * x2 + beta3 * x3
```

境界外頂点は対応境界三角形の補間位置とRest Pose offsetを使用する。offsetはRest Pose境界面の法線・接線基底で定義し、変形後は境界三角形から再構築した局所基底へ変換して適用する。基底の急変を避けるため、境界三角形IDは固定し、フレームごとの最近傍再検索をしない。

### 11.2 Cloth埋め込み

同一トポロジーは直接インデックス対応を使う。異なるトポロジーはRest Pose上の最近接表面三角形と重心座標を保存する。大きな距離の頂点は対応失敗とする。

### 11.3 法線・接線

描画三角形ごとに変形後の面法線を計算し、描画頂点へGatherして正規化する。UVシーム・ハードエッジはRender Vertex単位で法線を集約し、Physics VertexのWelding結果をそのまま描画法線へ流用しない。

接線はUV微分から再計算し、法線に直交化する。UV退化三角形は前フレーム接線または安定した直交ベクトルへフォールバックする。

## 12. 物理属性・ペイントデータ

### 12.1 材料と頂点ウェイト

材料はメッシュ単位の基準値、頂点ウェイトは0～1の乗算・補間係数として保存する。材料値の範囲とウェイトの意味はチャンネルごとに定義する。物理方式で無効なチャンネルをロードした場合は警告し、値を破棄せず保持可能な拡張フィールドへ格納する。

### 12.2 Render MeshからPhysics Meshへの転送

Render Vertex Bindingを逆方向に使って単純平均しない。Physics Vertexに対応するRender Mesh近傍頂点・面から、Rest Pose上の距離と面積重みを用いて転送する。転送元の範囲・重み・誤差を診断し、対応が曖昧なPhysics Vertexは未確定として表示する。

### 12.3 ペイント操作

- Paint: 現在値をブラシ目標値へStrengthに応じて補間。
- Smooth: 近傍頂点の加重平均へ近づける。
- Average: 選択範囲の平均値を算出し適用。
- Erase: メッシュ基準値へ戻す。

ブラシ半径はモデル空間の距離単位で指定する。画面ピクセル半径を使う場合はカメラ距離からモデル空間へ変換する。ペイント結果はJSONまたはアセットサイドカーファイルへ保存する。

## 13. 時間管理・試験環境

### 13.1 固定時間刻み

描画delta Timeを直接物理へ渡さない。CPU側Accumulatorで固定時間刻みのシミュレーションステップを要求する。フレームあたりの最大ステップ数を設定し、超過時は遅延として記録する。シミュレーション時間を黙って飛ばさない。

### 13.2 1ステップ進行

Pause中のStep要求は、固定時間刻みのシミュレーションステップをちょうど1回実行する。描画フレーム更新とは独立させる。複数要求はキューに保持するか、UI上で一回分にまとめるかを選択可能にする。

### 13.3 JSONスキーマ

トップレベルにschema_version、asset_id、solver_settings、initial_state、animation、forces、collisions、materials、vertex_weights、test_sequenceを持つ。

未知のschema_versionはロード拒否。未知の任意拡張フィールドは保持して再保存できる。必須フィールド欠落、NaN/Infinity、負の時間刻み、範囲外パラメータはロードエラーとする。既定値適用は任意項目だけに限定し、ログへ記録する。

### 13.4 再現性

乱数を使う処理はSeedを明示し、GPUスレッド実行順序に依存する非決定的な加算を避ける。Gatherは固定隣接順序で加算する。同一GPU・同一ドライバ・同一ビルドで再現性を計測する。異なるGPU間は許容誤差評価とする。

## 14. GPU Pass・同期設計

基本的な1サブステップのPass順:

1. Animation Target Update
2. External Force / Prediction
3. Soft Body Deformation Gradient
4. Soft Body Polar Rotation / Stress
5. Soft Body Tet Force Output
6. Soft Body Vertex Force Gather
7. Soft Body Velocity / Position Update
8. Cloth Stretch Color Passes
9. Cloth Shear Constraint Output
10. Cloth Bend Constraint Output
11. Cloth Vertex Gather / Apply
12. Broadphase Count
13. Prefix Sum
14. Broadphase Fill
15. Cell Sort / Group
16. Candidate Generation / Deduplication
17. VT / EE Narrowphase
18. Contact Constraint Output
19. Contact Vertex Gather / Apply
20. Friction / Velocity Update
21. Render Position Feedback
22. Render Normal Gather
23. Render Tangent Update

反復が必要なPassは設定回数だけ実行する。Pass間のSRV/UAV競合を避けるため、同一リソースを読み書きする場合は別バッファへPing-Pongし、必要なUAV unbind / resource transitionを明示する。D3D11 Immediate Contextのコマンド発行順を基準とし、Deferred Contextを使う場合もリソース依存関係を保つ。

## 15. GPUメモリ・容量管理

各バッファは初期容量と最大容量を持つ。容量はアセット構築時の推定値と実測ピークから決める。可変長候補・接触・セル登録・補正寄与は、Count / Scan後に必要数を検査してからFillする。

メモリ不足時は、現在のシミュレーション状態を確定せずエラーにする。低優先度の可視化バッファは無効化可能だが、物理接触や補正寄与を削って処理を続行しない。バッファ拡張時は古いリソースを安全に解放し、GPU使用中リソースの破棄タイミングを管理する。

## 16. 診断・エラー処理

診断コードを少なくとも以下に分類する。

- InvalidAsset / InvalidTopology
- DegenerateTetrahedron / InvertedTetrahedron
- InvalidMaterialParameter
- RenderBindingError
- BroadphaseOverflow / ContactOverflow / ContributionOverflow
- CCDIterationLimit
- InitialPenetrationLimit
- NonFiniteState
- GpuResourceAllocationFailure

致命的エラー時は該当シミュレーションステップをコミットしない。予測位置と確定位置を分離し、異常な状態を次フレームへ伝播させない。警告はメッシュID、要素ID、Pass名、関連カウンタを含む。

## 17. 初期試験仕様・受け入れ基準

数値は初期ベンチマークの合否基準案であり、レビュー後に固定する。実測により根拠なく達成できない場合は、理由と変更値をレビュー記録へ残す。

| 試験 | 条件 | 初期合格基準案 |
|---|---|---|
| T01 Rest Pose | 無負荷・固定姿勢 | 10秒相当で最大ドリフトがバウンディングボックス対角の0.1%未満 |
| T02 Soft Body Compression | 体積10%圧縮後に解放 | 体積誤差5%以内へ復帰し、局所変形が視認できる |
| T03 Soft Body Rotation | 剛体回転のみ | 回転による内部ひずみが初期長さの0.5%未満 |
| T04 Cloth Stretch | 既知の張力を付与 | Compliance増加で伸びが単調に増加 |
| T05 Cloth Bend | 片持ち布を曲げる | Bend Compliance変更で曲げ量が単調に変化 |
| T06 VT / EE | 2物体の接触 | 双方の補正が発生し、接触後の最大貫通が設定厚みの25%以下 |
| T07 Self Collision | Cloth折り畳み | 隣接除外部以外の貫通を検出し、反復後に減少 |
| T08 CCD | 1ステップで厚みの2倍以上移動 | 離散判定の見逃しケースを連続判定で検出 |
| T09 Binding | 既知の埋め込み頂点 | Rest Pose再構成誤差の最大値がメッシュ対角の0.1%以下 |
| T10 Overflow | 意図的に小容量を設定 | 候補切捨てなしでエラー・再試行が発生 |
| T11 Reproducibility | 同一条件を複数回実行 | 同一GPUで位置RMS差がメッシュ対角の0.01%以下 |
| T12 Performance | GTX 1660 SUPER | Pass別時間、GPUメモリ、要素数、候補数を取得できる |

性能FPS自体はメッシュ規模と品質設定を併記して評価する。3～40 FPSの目安を検証する代表シーンを別途固定する。

## 18. 実装順序と完了条件

1. 共通アセット・glTF/GLB・OBJ Importer
   - 座標系、単位、トポロジー検査が通る。
2. Physics Mesh Builder・四面体生成・キャッシュ
   - 品質検査と再生成判定が通る。
3. Render Vertex Binding・GPU Render Feedback
   - Rest Pose誤差基準を満たす。
4. Test Environment・GPU計測
   - 固定時間刻み、Pause、Step、JSON、GPUタイムスタンプが動作。
5. Cloth内部ソルバー
   - Stretch / Shear / Bendの単体試験が通る。
6. Soft Body Corotational FEM
   - 剛体回転、圧縮、局所押込み、復元試験が通る。
7. Broadphase / Narrowphase / 双方向接触
   - VT / EE、Soft Body / Clothの全組み合わせ試験が通る。
8. CCD・初期貫通処理
   - 高速移動・初期貫通試験が通る。
9. アニメーション連携
   - 平行移動、回転、クリップ切替、Teleport試験が通る。
10. 物理特性ペイント
    - 保存・再読込・メッシュ再生成後の転送検査が通る。
11. 性能・品質最適化
    - Pass別計測と代表シーンの基準を満たす。

各段階でレビューと試験結果を記録する。後段機能のために前段の不正データを黙って許容しない。

## 19. 未確定事項・レビュー時の重点確認

以下は初版詳細設計のレビューで必ず確認する。

1. Corotational応力式と反転要素の回復処理が、極端な変形で安定するか。
2. 半陰的Eulerと必要サブステップ数がGTX 1660 SUPERの目標に適するか。
3. 体積係数を材料式へ導入する方法が物理的意味を保つか。
4. Shear距離制約とBend二面角制約の組み合わせがメッシュ方向依存を生まないか。
5. Gather Jacobiの緩和係数とXPBD λ更新の組み合わせが収束するか。
6. 接触摩擦の位置補正・速度処理の二重適用が起きないか。
7. CCDの保守的探索が高速ケースを見逃さず、計算量が許容できるか。
8. Large Primitiveの別経路が極端な三角形比率で破綻しないか。
9. 非同期overflow検出とステップ再実行がD3D11のリソース寿命・時間管理と整合するか。
10. Render Bindingの局所基底offsetが強い曲げ・境界反転で破綻しないか。
11. CPUアニメーション評価とGPU物理更新の同期が固定時間刻みを維持できるか。
12. 初期受け入れ数値が実測可能であり、試験データが再現可能か。

## 20. バージョン履歴

- 1.0: 基本設計1.1を受けて初版詳細設計を作成。初期アルゴリズム、数式、GPU Pass、データ構造、エラー処理、試験仕様を定義。
