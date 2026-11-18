# Skin Physics 基本設計書

- 文書バージョン: 1.9
- 状態: 要件整合・設計閉鎖候補 / 独立Critical Review待ち
- 対応要件: requirements.md 1.1
- 対象: Skin Physics
- 前版: 1.7
- Runtime統合設計: [runtime_integration_design.md](runtime_integration_design.md)
- 数値・物理受け入れProfile: [numerical_acceptance_profile_v1.json](numerical_acceptance_profile_v1.json)

## 1. 設計方針

Skin PhysicsはSoft BodyとClothを同一Simulation Worldで扱い、複数Model Instance / Mesh Componentの相互衝突・自己衝突・Skeleton Animation連携をGPU上で実行するC++17 / Direct3D 11静的Libraryとする。v1のPublic Runtimeは単一Bodyに限定せず、1 World内の複数Soft Body / Cloth / physics_mode none Meshを扱う。

1. Render MeshとPhysics Meshを分離する。
2. Soft BodyとClothの材料モデル・制約ソルバーを分離し、衝突管理とGPU実行管理を共通化する。
3. 物理計算と描画頂点への反映をGPU内で完結させる。
4. オフライン変換工程とランタイム処理を分離する。
5. Direct3D 11 / Shader Model 5で利用できない浮動小数点Atomic Addに依存しない。
6. 全ての可変長GPU出力について、容量超過を検出し、候補や補正を黙って欠落させない。
7. 検証環境とGPU計測を初期段階から整備する。

## 2. 全体構成

- Model Importer: 形式別読み込みと共通内部表現への変換
- Physics Model Builder: Physics Mesh、制約、対応付け、衝突用データの生成
- Asset Cache: 生成済み物理データの保存・読込
- Simulation Manager: World / Model / Mesh登録、時間管理、非同期GPU Tick Scheduler、GPU Commit / Rollback、Diagnosticsの統括
- SkinPhysics Public Runtime: Host向け静的Library API、Handle、Resource / World lifecycle
- D3D11 Resource Interop: Host-owned Device / Immediate Context、Committed Render Bindingの共有
- Soft Body Solver: 四面体弾性モデルの解決
- Cloth Solver: 表面メッシュの伸縮・せん断・曲げ
- Collision System: Broadphase、Narrowphase、共通接触制約の生成・解決
- Animation Interface: 現在・前回のアニメーション状態を物理入力へ変換
- Render Feedback: 物理結果から描画頂点・幾何属性への反映
- Property Editor: 頂点物理特性の編集・可視化
- Test Environment: JSON試験環境、再生制御、可視化、計測

Soft Body / Clothは各自の内部材料制約を解く。両者および自己衝突は共通Collision Systemが接触制約を生成し、接触する双方のPhysics Meshへ補正を配分する。

## 3. データ表現

### 3.1 Render Mesh

描画頂点属性、インデックス、マテリアル、スキニング情報を保持する。物理計算の都合で描画トポロジーを変更しない。

### 3.2 Physics Mesh

物理頂点、要素、隣接情報、逆質量、物理属性を保持する。

- Soft Body: 四面体要素、境界三角形、要素隣接
- Cloth: 三角形表面、Stretch / Shear / Bend等の制約トポロジー

物理頂点の状態は少なくとも現在位置、前位置または速度、予測位置、逆質量を管理する。必要な履歴バッファはソルバーの更新方式に合わせて設計する。

### 3.3 Collision Proxy

Render Mesh / Physics Meshと独立した衝突用形状を保持できる。Broadphase用の簡略形状とNarrowphase用の判定形状を分離可能にする。Proxyのみで最終接触位置を決める場合は近似誤差を計測する。

### 3.4 Render Vertex Binding

描画頂点とPhysics Meshの対応はオフライン構築し、ランタイムで再検索しない。

Soft Bodyは次の順で対応付ける。

1. 四面体内部にある頂点は四面体インデックスと4頂点の重心座標を保存する。
2. 四面体外の頂点は、境界面上の対応三角形、三角形重心座標、Rest Poseでのオフセットを保存する。
3. 最近傍境界への対応付け距離とRest Pose再構成誤差を検査する。
4. 許容誤差超過は警告またはビルド失敗とし、無条件に不正な対応を採用しない。

概念データ:

```cpp
enum class render_binding_type : uint32_t
{
    tetrahedron,
    boundary_triangle
};

struct render_vertex_binding
{
    render_binding_type type;
    uint32_t element_index;
    float weight[4];
    float rest_offset[3];
};
```

四面体内部のweight[0..3]は重心座標、境界三角形の場合はweight[0..2]を三角形重心座標として扱う。rest_offsetの座標系と変形時の適用方法は詳細設計で定義する。上記は概念構造であり、実際のGPUパッキングは別途決定する。

Clothは物理表面三角形への対応と重心座標を保存する。トポロジーが同一の場合は直接インデックス対応を許可する。対応不能な頂点や距離超過を検出する。

### 3.5 頂点・要素の安定した識別

ペイント属性やキャッシュの再利用に備え、頂点番号だけに依存しない対応識別情報を持たせる。再生成後のトポロジー変更で対応が一意に定まらない場合は、属性転送結果を要確認として扱う。

## 4. オフライン物理メッシュ生成

### 4.1 Soft Body四面体化

fTetWildをオフライン生成器の第一候補とする。処理の参考段階は表面前処理、四面体化、境界面回復、最適化、内部領域抽出である。採用前にライセンス、依存関係、ビルド、出力品質を確認する。

生成後に以下を検査する。

- 入力表面の開境界、非多様体、自己交差
- 退化・反転四面体
- 体積、アスペクト比等の四面体品質
- 元表面と生成境界の距離誤差
- Render Vertex Bindingの再構成誤差
- 極端に小さい要素や局所的な密度偏り

入力の自動修復で形状が大きく変わる場合は警告し、元データを黙って置き換えない。

### 4.2 キャッシュ

Physics Mesh、要素接続、隣接情報、材料属性、Render Vertex Binding、Collision Proxyを保存する。入力アセット識別子、生成設定、キャッシュ形式バージョン、生成器バージョンを保持し、不一致時は無効化する。

## 5. シミュレーションパイプライン

固定時間刻みのLogical Tickを基本単位とし、World内の全Soft Body / Cloth / Contact / Animation Trackingが同じ時刻を進む。World共通SubstepはForce / Impulse Event境界およびTOIでIntegration Segmentへ分割し、全Solverを同じSegment時間で進める。Segment単位の予測・速度更新・減衰・Kinematic速度の規則は詳細設計§6.6 / §7.1.1 / §7.1.2を正とする。

1. Logical Tick開始時にWorld共通Substep数を算出する。
2. 各Substep境界のAnimation / Root Transform / Kinematic Poseを評価する。
3. 全Meshへ同じSubstep Delta Timeを適用し、Soft BodyとClothの外力・Impulse・予測位置を更新する。
4. 全参加MeshのSwept AABBからBroadphase候補を生成する。
5. VT / EE Narrowphase、Contact Patch、接触制約を生成する。
6. Soft Body内部力、Cloth内部制約、Animation Tracking、Contactを規定順で反復解決する。
7. CCDでTOIが見つかった場合、World全体をTOI時刻まで再積分し、Impactを一度だけ適用して残時間を同じSubstep内で再積分する。
8. 位置補正後の速度を再構築し、摩擦・反発を処理する。
9. Physics MeshからRender MeshへPosition / Normal / Tangentを反映する。
10. Render CandidateとPhysics Stateを検証し、Logical Tick単位でCommitする。

Soft BodyのCFL / Animation Tracking安定条件からWorld共通Substep数を決定する。ClothとContactにも同じSubstep Delta Timeを適用し、v1ではMeshごとの独立Subcyclingを行わない。TOIによる時間分割もWorld全体へ適用するため、異なるソルバーが異なる時刻の形状を使って接触することを防ぐ。

外力・Impulseの空間分配、Cloth予測式、Contact縮約、Broadphase容量、TOI prefix構成は詳細設計で規定する。各GPU Pass間の依存関係とUAV unbind / SRV bindを明示する。Production Runtimeでは1 Logical TickをGPU Commit transactionとし、Dispatch受付とGPU Commitを分離する。Failure時は確定済みStateを維持し、同一Tick Inputから再実行する。Host向け詳細は [runtime_integration_design.md](runtime_integration_design.md) を正とする。

## 6. ソルバー設計方針

### 6.1 XPBD共通方針

Cloth制約および接触制約はXPBDを基本候補とする。XPBDとして扱う制約は、制約関数、コンプライアンス、逆質量、サブステップ時間、制約ごとのLagrange multiplier λを持つ。

Gather方式では、制約ごとに独立した補正寄与を出力し、頂点側が隣接制約の寄与を収集する。単純な平均補正をXPBDと呼ばない。λ更新と位置補正の数式、正規化規則、反復間のλ保持・リセット条件は詳細設計で定義する。

### 6.2 Cloth

- Stretch: 静的トポロジー制約としてGraph Coloringを第一候補とする。
- Shear / Bend: Gather Jacobiを第一候補とする。
- 自己衝突・動的接触: 共通Collision Systemが生成する接触制約をGather方式で解く。

Graph Coloringでは同一色の制約が頂点を共有しないことを検証し、色単位で順次Dispatchする。色数が過大になるメッシュではGather方式との性能比較を行う。

### 6.3 Soft Body構成モデル

Soft Bodyの主モデルは四面体ベースの弾性モデルとする。単純な辺長・体積制約だけでは局所変形やせん断応答を十分に表現できないため、主モデルとして採用しない。

基本設計上の第一候補を**Corotational FEM系の四面体弾性モデル**とする。局所回転を考慮し、剛体回転に対する不要な抵抗を抑えながら、変形・せん断・圧縮・膨張を評価する。体積保持は材料モデル内の体積応答と独立した調整パラメータの両面から検討する。

実装上は、四面体ごとの局所変形評価と頂点Gatherを組み合わせる。FEMの力・剛性をXPBDのλ式と同一視しない。Corotational FEMをそのままXPBDと呼ばず、以下の候補を詳細設計で比較して最終決定する。

- 陽的 / 半陰的な弾性力ベースの積分
- Corotational FEMの力ベース解法
- コンプライアンス制約として構成した四面体ひずみ・体積制約
- Shape Matching等の近似モデル

比較軸は、局所へこみ、体積変化、せん断、剛体回転、安定性、反復収束、GPU演算量、メモリ量、GTX 1660 SUPERでの計測値とする。必要なら低コストな距離・体積制約を比較用ベースラインとして実装するが、FEM相当の物理モデルとして扱わない。

### 6.4 Direct3D 11でのGather

Shader Model 5の汎用浮動小数点Atomic Addに依存しない。

基本方式:
1. 制約単位のDispatchで制約ごとのλ更新と頂点別補正寄与を出力する。
2. 頂点→関連制約のCSR形式隣接リストを構築する。
3. 頂点単位のDispatchで隣接制約の寄与をGatherする。
4. 逆質量、寄与数、正規化・緩和規則を適用して位置を更新する。

各制約の寄与をどの頂点へ何個出力するかを明示し、バッファ上限とオーバーフローを検出する。補正寄与の平均化は制約の物理的な重みを変える可能性があるため、詳細設計で数学的根拠と収束性を検証する。

## 7. 衝突システム

### 7.1 共通接触制約

Collision SystemはSoft Body / Clothの組み合わせを横断して接触制約を生成する。片側のソルバーだけで相手を押し出す非対称な処理を基本方式にしない。接触制約は双方の頂点と逆質量を参照し、双方へ補正を配分する。

### 7.2 Broadphase

GPU Spatial Hashを第一候補とする。ただし、粒子と三角形を同じ方法で登録しない。

- 粒子: 所属セルへ登録する。
- 三角形: 現在フレームまたはSwept AABBが重なるセルへ登録する。
- 登録後: Hash値だけで一致判定せず、元の整数セル座標を比較する。
- 大型三角形: 多数セルへの登録コストを抑えるため、別のLarge Primitiveリストまたは階層グリッド等へ分離する。
- 候補の重複: 同じ組み合わせを一意化するか、接触生成前に重複排除する。
- 自己衝突: 同一要素・隣接要素等の除外情報を参照する。

可変長登録はCount → Prefix Sum → Fillの段階構成を基本候補とする。固定容量を超えた場合はoverflowフラグと必要容量を記録し、容量拡張後に再実行する。安全な再実行ができない場合は当該ステップをエラーとして扱い、候補を黙って切り捨てない。

Spatial Hash、Radix Sort、階層グリッドの最終構成とセルサイズは、粒子数・三角形数・大型三角形比率を変えたベンチマークで決定する。

### 7.3 Narrowphase

主な接触候補はVertex-Triangle (VT) と Edge-Edge (EE) とする。VTでは頂点と三角形最近接点・重心座標を求める。EEでは両辺の最近接点と線分パラメータを求める。

自己衝突では同一要素、共有頂点・共有辺、近傍要素等の除外ルールを適用する。除外範囲はメッシュ解像度に応じて検証する。

### 7.4 双方向接触応答

VT接触では、接触点の補正を三角形側3頂点へ重心座標で配分し、反対側の頂点にも逆質量に応じた補正を適用する。

EE接触では、最近接点を構成する各辺の2頂点へ線分パラメータに基づいて補正を配分し、双方の辺へ作用させる。

接触ごとのλ、法線方向、摩擦接線、接触の重複、接触寿命、サブステップ間のλ保持条件を管理する。Contact Temporal Quality Limitは初期値1/120秒、0を明示無効とし、XPBDの安定保証ではなくWorld共通Substepの品質上限として扱う。接触参加頂点の有効逆質量が閾値以下なら非応答ContactとしてPosition / ImpulseをSkipする。複数Contactが頂点を共有するVelocity ImpulseはGlobal Projected JacobiでCoupleし、Residual未収束時はTickをCommitしない。摩擦 / 反発は詳細設計§8.5のCoupled Contact Solverを正とする。

### 7.5 CCD

Broadphaseでは前位置と予測位置を包含するSwept AABBを用いて候補を絞る。全候補へ高価なCCDを実行せず、相対速度、移動距離、衝突厚み、離散判定の結果等からCCD対象を選択する。

NarrowphaseではVT / EEの連続判定を候補とし、Time of Impactまたは保守的なSpeculative Contactを生成する。詳細設計で数式、退化ケース、反復中の位置更新との整合性、偽陽性・偽陰性の扱いを定義する。

### 7.6 初期貫通

初期化時にBroadphase / Narrowphaseを実行し、初期接触を抽出する。修正量上限と反復上限を設定する。上限を超える貫通、修正後も残る貫通、形状不正は警告またはロード失敗とする。過大な一括変位でメッシュを破壊しない。

## 8. アニメーション連携

CPU Animation EvaluatorはClip Key / Loop / Blend Weight / Root Transform / Collider Sample境界ごとにSpan Packetを作り、GPU Trajectory Evaluatorが任意tauのBone / Skinning / Collider Transform / Contact Point Velocityを評価する。CPU Absolute Timeはdouble、GPUはSpan-local normalized floatとSegment-relative tauを使い、Absolute World TimeをfloatへCastしない。固定SM5 Packet ABIと各Segmentでのnormalized cursor更新を検証する。v1は最大2 Trackの非Additive Blendとし、Blend Weight時間微分を含める。Animation Target Velocityは最終Skinning Target PositionのSimulation Time微分を正本とし、STEP / LINEAR / CUBICSPLINE / Clip Blendの補間微分とPlayback Speedを反映する。Render Frameや前StepのPosition差分から速度を推定しない。

Fixed / Kinematic VertexはSegment終端TargetとTrajectory Velocityを維持する。Dynamic Vertexは非接触内部Constraintを解いた後、Contact Position Constraintを解く前にContact前Velocityを再構築して保存する。Contact位置補正後のPosition差分からVelocityを再構築しない。Teleport / Reset / Clip Discontinuity / Time Rewindでは明示Discontinuity Policyが決めたTarget / Velocityを渡し、不連続を跨ぐ有限差分で速度を生成しない。詳細規則はdetailed_design.md §5.3 / §6.6 / §8.5 / §11.4を正とする。

Kinematic Animation / ColliderのSwept AABBはSegment端点を結ぶChordを基準とし、Chord偏差の保守的Boundで膨張させる。Conservative Advancement / TOI位置はGPU Animation Trajectory Packetをtau時刻で評価し、区間Speed Boundを使う。Deviation / Speed Boundを得られない場合はSegment細分化またはCCDUnresolvedとする。Kinematic接触相対Velocityには同じTrajectoryの解析微分を使う。

Hair Cards / Hair BundlesはClothとして扱う。Strand Hairの伸縮・曲げ・ねじりを表す専用モデルは将来拡張とする。

## 9. Render Feedback

Soft Bodyは四面体埋め込みまたは境界三角形埋め込みから描画位置を補間する。Clothは直接対応または表面三角形補間を用いる。

対応付け時にRest Pose再構成誤差を測定し、閾値超過頂点数・最大誤差・平均誤差を記録する。対応付け失敗を黙って最近傍頂点へ置換しない。

法線は変形後の三角形からGPUで再計算する方式を基本候補とする。接線はUV微分等から再構築するか、法線との直交性を補正する。UVシームやハードエッジの重複頂点を考慮し、物理頂点のWeldingと描画頂点属性の分離を維持する。

## 10. 物理特性データ

メッシュ単位の基準材料パラメータと頂点単位のウェイトを分離する。Soft BodyとClothで有効なチャンネルを明示する。

Soft Bodyでは弾性・せん断・体積応答・減衰・アニメーション追従を管理する。ClothではStretch / Shear / Bend / Damping / Animation Trackingを管理する。

ペイントはRender Mesh基準で行い、Physics Meshへの転送結果を可視化する。再生成時は安定した識別情報または幾何対応を用いる。曖昧な転送は自動確定せず警告する。

## 11. 試験・デバッグ環境

初期段階から試験環境を整備する。

可視化:
- Render Mesh / Physics Mesh / Collision Proxy
- 四面体品質、境界対応、制約、Broadphaseセル、衝突候補、接触法線
- 速度、変位、体積変化、物理特性ヒートマップ
- overflow、CCD適用候補、初期貫通修正量

計測:
- GPU各Pass時間
- 頂点数、四面体数、三角形数、制約数
- Broadphase登録数、候補数、重複数、overflow回数
- Narrowphase確定接触数、CCD実行数
- GPUバッファ容量・使用量
- 最大変位、体積誤差、静定時間

JSONはバージョン、固定時間刻み、初期状態、アニメーション、外力、衝突設定、物理属性、アセット識別情報を保持する。未知バージョン・不正値・必須項目欠落を検出する。

## 12. 性能評価

主評価環境はGTX 1660 SUPER 6GBとする。3～40 FPSは開発時の許容目安であり、特定規模に対する保証値ではない。

総フレーム時間に加えて、Soft Body、Cloth、Broadphase、Narrowphase、接触解決、Render Feedbackの時間を個別に記録する。メッシュ規模、接触密度、サブステップ数、反復数を変化させる。

メモリはOS・描画処理の使用量を考慮した予算を設定する。各バッファの最大容量、成長条件、overflow時の再試行回数を明確にする。容量を超えた接触を黙って削除して性能を維持する挙動は禁止する。

## 13. 開発段階案

1. 共通モデル表現・Importer基盤
2. Physics Mesh生成・品質検査・キャッシュ
3. Render Vertex Binding・GPUフィードバック
4. 試験環境・GPU計測基盤
5. Cloth内部ソルバー
6. Soft Body Corotational FEM
7. 共通Broadphase / Narrowphase / 双方向接触
8. CCD・初期貫通処理
9. アニメーション連携
10. 物理特性ペイント
11. 各Solver単体の品質・性能評価
12. GPU Runtime Transaction / Async Scheduler / Render Interop統合

Phase 1～11の既存Fixture / Testと依存関係を維持する。Phase 12は本書のGPU Runtime Integration DesignとGate R0～R6を通す。各段階の完了条件は詳細設計・testing_strategy.mdで定義する。Soft Bodyの構成式をNeo-Hookean XPBDへ無条件に置換せず、初期版はCorotational force-based FEMを維持する。

## 14. 詳細設計との責務境界・校正への引継ぎ

本書で示した基本アーキテクチャに対する数式・GPUデータ構造・失敗時契約は、[detailed_design.md](detailed_design.md) v1.11および[runtime_integration_design.md](runtime_integration_design.md) v1.8で具体化済みである。以下は実装者が独自に方式を選ぶ未決定事項ではなく、詳細設計に定義された契約を実装し、Fixture / WARP / Physical GPUで検証する対象である。

- Soft Body Corotational FEM、応力・内部力・Segment積分・減衰: detailed_design.md §6。
- Cloth Stretch / Shear / Bend / XPBD Lambda、外力・Impulse、World Integration Segment: detailed_design.md §7。
- VT / EE、Contact Patch、双方向応答、摩擦・反発、退化接触: detailed_design.md §8。
- Hierarchical Spatial Hash、Large Primitive LBVH、Capacity / No-Silent-Drop: detailed_design.md §9-10。
- CCD / TOI、Segment retime、初期貫通修正: detailed_design.md §10-11。
- Animation Target / Velocity、Render Binding、属性保持: detailed_design.md §5-6 / §12-13。
- GPU Pass、Tick Transaction、Capacity、JSON、再現性、Gate R0-R6: detailed_design.md §14-19、runtime_integration_design.md §5-10。
- 残る数値校正・実機計測はdetailed_design.md §19、testing_strategy.md、numerical_acceptance_profile_v1.jsonを正とする。

## 15. 参考OSS・技術資料

以下はアルゴリズムの参考候補であり、ランタイム依存を意味しない。

- fTetWild: https://github.com/wildmeshing/fTetWild
  - オフライン四面体メッシュ生成。
- gpu-cloth-sim: https://github.com/alien-life/gpu-cloth-sim
  - GPU Cloth、頂点Welding、Proxy衝突、GPU描画連携。
- XPBD-Cloth: https://github.com/steampower33/XPBD-Cloth
  - XPBD、Graph Coloring、Gather/Jacobi系処理、GPU Spatial Hash。固定候補上限やAtomic実装はそのまま移植しない。
- nebula: https://github.com/20niship/nebula
  - GPU XPBD Cloth / Tetrahedral Soft Bodyの構成参考。報告性能は衝突計算の影響を含むためソルバー単体性能として扱わない。
- IPC Toolkit: https://github.com/ipc-sim/IPC
  - CCD、接触、摩擦の理論的参考。ランタイム依存にはしない。

参考実装のアルゴリズムをそのまま採用せず、Direct3D 11 / Shader Model 5と対象GPUの制約に合わせて検証する。

## 16. バージョン履歴

- 1.0: 初版。全体構成、物理方式候補、GPUソルバー候補、衝突方針を定義。
- 1.1: 初版レビュー対応。Soft Bodyモデル選定方針、XPBDとGatherの役割分離、双方向接触の補正配分、Spatial Hashの大型要素・overflow、CCD適用方針、アニメーション速度、Hair対応範囲、描画頂点対応の検査を明確化。
- 1.2: GPU Runtime / Static Library境界、Host-owned D3D11 Device / Immediate Context、非同期GPU Tick Commit、Rollback / Retry、Committed-only Render Binding、複数Model / Mesh World、数値受け入れProfileとPhase 12統合Gateを追加。
- 1.9: SM5固定幅PacketのTrack / Channel / Blend / Collider RangeとNode Role / Inverse Bind参照、補間Flags / Quaternion Coefficients / Column-Major Matrix ABIを確定。
- 1.8: SM5固定幅Animation / Collider Packet ABI、CPU double絶対時刻とGPU Span-local float時刻、各Segmentでのnormalized cursor更新、Animation / Collider arbitrary-tau評価、Coupled Contact / bounded Segment Schedulerを追加。
- 1.7: GPU Animation Trajectory Packet / Spanによる任意tau評価、最大2 Track Blend、非応答Contactの退化処理、Coupled Projected Jacobi Impulse、Contact Temporal Quality Limit、Logical Tick内Integration Segment数上限を追加。
- 1.6: Animation Target Velocityを最終Skinning Trajectoryの時間微分へ統一し、Kinematic CCDの保守的Path Bound、Contact Position correctionとContact前Velocityの分離を追加。
- 1.5: Force / Impulse EventとTOIによるIntegration Segment分割、Soft Body / Cloth共通のSegment予測、線形粘性減衰の積分、Fixed / Kinematic Velocityの所有規則を追加。
- 1.4: World共通SubstepによるSoft Body / Cloth / Contact / CCDの時間整合、Cloth動力学予測、Runtime Tickの時間予約、Render Lease・Resource Lifecycle契約を追加.


## 17. GPU Runtime / Library統合方針（v1.9）

Skin Physicsは既存のVisual Studio Target `SkinPhysics` をC++17 / x64 / Direct3D 11専用の静的Libraryとして提供する。CMakeを必須化しない。Host ApplicationがWindow、Input、UI、Scene / Game Logic、Renderer、D3D11 Device / Immediate Contextを所有し、SkinPhysics RuntimeはSimulation World、GPU Physics State、Compute Resource、Tick Scheduler、Commit / Rollback、非同期Queryを所有する。

- 1つのWorldに複数Model Instance、複数Mesh Component、Soft Body / Cloth / physics_mode noneを混在登録できる。
- Mesh / Model / PrimitiveはWorld Global IDとStable IDを持ち、Contact PairとDiagnosticsに使う。
- HostとRuntimeは同じImmediate Contextを単一Submission Threadから操作する。独立Physics GPU Queueを仮定しない。
- GPU StateはRuntime内部の唯一の正とし、通常FrameでPhysics Vertex / Contact / Lambda全量をCPUへReadbackしない。
- GPU Submission、GPU Logical Tick Commit、CPU-observed completed prefixを分離する。Event QueryはGPU Command完了を示し、Tick成功はGPU Status Bufferで判定する。
- Capacity Overflow、CCD Unresolved、NonFinite、Physical Quality Gate FailureはWorking StateをCommitしない。必要に応じて同じTick開始Snapshot・入力でRetryし、解決不能ならWorldをBlock / Failedとする。
- Hostが利用できるGPUデータはCommitted State専用のRead-only Render Bindingとする。Render BindingはPosition / Normal / TangentのSRV、Stride / Count / Resource Generationを持つ。内部Solver Buffer / Scratch / Writable UAVを公開しない。
- Fixed Delta TimeはTest Environmentから設定可能。初期値は既存の1/60秒とし、Time ScaleはAccumulatorへの時間供給倍率として扱う。Soft BodyのCFL / Tracking、Cloth Temporal Quality Limit、Contact Temporal Quality Limit、World設定上限を集約してWorld共通Substep数を決定し、Soft Body / Cloth / Contact / Animation Trackingへ同一Substep時間を適用する。Contact capの初期値は1/120秒、0は明示無効。Animation Span Key / Loop / Blend / Root、Force / Impulse Event境界とTOIでIntegration Segmentへ分割し、maximum_integration_segments_per_tick（初期値512）を超える場合はReservationを保持してBlockする。全参加Solverを同じSegment時間で進め、v1ではSolverごとの独立Subcyclingを行わない。
- in-flight GPU Tickは既定2件、設定可能範囲1 / 2 / 4件とする。各TickのImmutable Dispatch Planと先行Tick Gateを必須とし、Status Query Ring容量は設定上限以上とする。性能比較で変更する場合は同じR0 Gateを再実行する。
- GTX 1660 SUPER 6GBで3～40 FPSをWorkloadごとに測定する。全Asset規模に対する固定保証値を設けない。品質・衝突範囲・Iteration・Substep・時間を黙って削減しない。
- CPU Reference / Analytical Fixture / WARPはCorrectness検証、物理GPUは性能・Memory検証と明確に分離する。

Library API、Device / Context Ownership、Tick State Machine、Render Interop、Device Loss、Error分類、Gate R0～R6は [runtime_integration_design.md](runtime_integration_design.md) を正とする。v1の具体的な初期パラメータ、接触Patch Reduction、Broadphase / CCD上限、Capacity Growth、Render Binding Ring、実装前Review Checklistは [design_closure.md](design_closure.md) を参照する。数値比較の初期Engineering Thresholdは [numerical_acceptance_profile_v1.json](numerical_acceptance_profile_v1.json) とし、実測校正前の合格実績として扱わない。

### 17.1 SoftBodyDemo設計から取り込む共通原則

- GPUをRuntime Physics Stateの唯一の正とする。
- GPU Commit Control、Snapshot、Rollback / Retry、非同期Status Readback。
- CPU Double Oracle、解析Fixture、WARPと物理GPUの役割分離。
- D3D11 Immediate Contextの単一Command Stream、Dispatch境界、SRV / UAV明示管理。
- CSR Gatherを使用し、浮動小数点Atomic Addへ依存しない。

### 17.2 Skin Physicsへ転用しない設定

- 単一Soft Body / 単一Colliderという制約。
- 200,000 Dynamic Particle / 平均40 FPS必須という性能条件。
- 固定1/120秒Logical Tick。
- Neo-Hookean FEM XPBDへの置換およびTet Lambda Recomputeを前提にしたSolver構成。

これらはSkin Physicsの要件・初期詳細設計と一致しないため採用しない。


## 18. 要件整合・設計閉鎖

- v1のImporter対象、Self Collision除外、Broadphase初期値、Contact Patch Manifold、CCD初期値、GPU Capacity Growth、Render Binding Ring、Fixed Delta変更手順を [design_closure.md](design_closure.md) で固定する。
- Soft Body / Cloth / 混在衝突 / 自己衝突 / Animation / Paint / JSON / Test Environmentの要件を維持する。
- SoftBodyDemo固有の単一Soft Body、単一Collider、200k粒子、40 FPS必須、1/120秒固定、Neo-Hookean FEM XPBDはSkin Physicsへ適用しない。
- 本書は独立Critical Review提出候補であり、承認・Production Runtime実装・物理GPU性能達成を意味しない。
