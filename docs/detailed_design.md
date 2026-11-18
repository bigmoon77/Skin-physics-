# Skin Physics 詳細設計書

- 文書バージョン: 1.10
- 状態: 要件整合・実装前契約閉鎖候補 / 独立Critical Review待ち
- 対応要件: requirements.md 1.1
- 対応基本設計: basic_design.md 1.8
- Runtime境界・非同期GPU Tick: runtime_integration_design.md v1.7
- 数値・物理受け入れProfile: numerical_acceptance_profile_v1.json
- 前版: 1.7
- 実装対象API: Direct3D 11 / HLSL Shader Model 5
- 言語・ビルド: C++17 / Visual Studio .sln・.vcxproj

## 1. 本書の目的と設計判断

本書は基本設計で残したアルゴリズム、数式、GPUデータ構造、処理順序、数値処理、エラー処理を実装可能な粒度へ具体化する。本書のレビューが合格するまで実装を開始しない。

### 1.1 初期版の方式

| 領域 | 初期採用方式 | 補足・代替 |
|---|---|---|
| Soft Body | 四面体Corotational Linear Elasticity、Segment単位の半陰的積分 + 指数線形減衰 | 反転要素はSVD補助処理と体積ペナルティ |
| Cloth Stretch | XPBD距離制約、Graph Coloring | 色数超過時はGather Jacobi |
| Cloth Shear | XPBD内積ひずみ制約、Gather Jacobi | 三角形局所Rest基底 |
| Cloth Bend | XPBD符号付き二面角制約、Gather Jacobi | 退化ケースの検出必須 |
| 接触 | XPBD片側不等式制約、Gather Jacobi | 法線位置補正、摩擦・反発は速度Impulse |
| Broadphase | 多段階GPU Spatial Hash + Count/Scan/Fill | 候補量によって容量拡張またはSimulation Error |
| Narrowphase | 双方向VT + EE | Soft Bodyは境界面のみ |
| CCD | Swept AABB + Conservative Advancement | TOIでサブステップを分割 |
| Skinning | CPUでPose評価、GPUでPhysics Mesh Skinning | 内部四面体頂点のウェイトはオフライン調和拡張 |
| Render Binding | オフラインEmbedding + GPU補間 | ランタイム最近傍検索なし |
| 四面体生成 | fTetWildをオフライン候補として使用 | 採用前にライセンス・生成品質を検査 |

Soft Bodyは力ベースFEMとして独立させ、Clothと接触はXPBDとする。FEMの材料応力とXPBDのλを同じ仕組みとして扱わない。

### 1.2 単位系・座標系

内部単位系はSIとする。

- 長さ: meter
- 質量: kilogram
- 時間: second
- 力: newton
- 密度: kg/m^3
- 弾性率: pascal
- 重力加速度: m/s^2

glTFのメートル基準を内部単位へ変換する。OBJは単位情報を保証できないため、インポート時に単位倍率の明示入力を要求する。単位不明のまま物理値を自動推定しない。

内部座標系は右手系Y-upとし、Importerで座標軸・三角形表裏・法線を一度だけ変換する。描画、物理、アニメーションでは同じ内部座標系を用いる。Static Mesh / Mesh Nodeに含まれる初期ScaleはOffline Asset Build時にPosition / Normal / Tangent / Rest GeometryへBakeし、質量・Rest Volume・制約・Bindingを変換後Geometryから再生成する。Runtime Model Root TransformはTranslation + Rotationのみの剛体Transformとし、非一様Scale・負Scale・ゼロScale・実行中のScale変更を禁止する。Skeleton BoneのScale AnimationはSkin Matrixに含まれる正値成分のみ許可し、特異または符号反転するScaleはImport Errorとする。ビルドはVisual Studio .sln / .vcxprojを正規手段とし、CMakeは必須としない。ランタイムの物理計算に外部物理ライブラリをリンクしない。Importer依存は入力変換層に限定し、四面体化器はオフラインAsset Buildでのみ使用する。

## 2. 初期対応アセット仕様

### 2.1 Importer

初期対応形式:

- glTF 2.0 / GLB: メッシュ、スケルトン、スキニング、アニメーション、マテリアル。
- OBJ: 静的メッシュのみ。スケルトン・アニメーションは対象外。

glTFアニメーションはSTEP / LINEAR / CUBICSPLINEを読み込み、補間処理を共通内部表現へ変換する。初期版ではMorph Targetによる頂点変形を対象外とし、検出した場合は警告する。

外部Importerライブラリを使う場合は、ライセンス・依存関係・MSVC対応を確認し、Importerモジュール内に閉じ込める。ランタイム物理処理からImporterの型を参照しない。

### 2.2 共通アセット

共通モデルはMesh単位に以下を保持する。

- 頂点位置、法線、接線、UV、カラー
- 名前付きVertex Groupと物理Attribute Channel
- 三角形インデックス、サブメッシュ、Material slot
- Bone hierarchy、inverse bind matrix、bone weights
- Animation clip、keyframe、補間方式
- 物理方式（none / soft_body / cloth）、物理属性、衝突設定
- 単位・座標系・軸変換情報

Skinning influenceは初期版で最大8本とする。インポート時に上位ウェイトを選択して正規化する。ウェイト合計が0、joint index不正、NaN等は入力エラーとして報告する。各Meshのphysics_modeはnone / soft_body / clothのいずれかとし、noneは物理計算対象外とする。collision設定はphysics_modeと独立し、物理を適用しないMeshもColliderとして登録できる。

## 3. CPU / GPU責務

### 3.1 CPU側

- アセット読み込み、入力検査、Physics Mesh生成・キャッシュ
- 四面体・三角形の隣接リスト、制約リスト、Graph Coloringの構築
- アニメーションClip評価、Bone行列生成
- GPUリソース生成・破棄、Fixed Step AccumulatorとGPU Submission SchedulerのCPU制御
- JSON入出力、試験環境制御、Model / Mesh / Collider登録
- 非同期GPU診断結果・Commit Statusの回収と容量再設定
- Host / Rendererと共有するD3D11 Immediate ContextのSubmission Thread管理

CPUは通常フレームのPhysics Vertex位置・速度・Contact / Lambda配列をReadbackしない。GPU診断結果とLogical Tick Statusの小規模Bufferだけを非同期回収し、通常処理を同期Readbackで停止させない。RuntimeはHostから渡されたD3D11 Device / Immediate Contextを使い、Hostと同じ単一Submission ThreadでGPU commandを発行する。

### 3.2 GPU側

- Physics MeshのSkinning目標位置と目標速度の生成
- Soft Bodyの変形勾配、局所回転、応力、内部力
- Cloth制約の予測・解決
- Broadphase / Narrowphase / CCD
- 接触位置補正、速度摩擦、反発
- Render Mesh位置・法線・接線更新
- Logical Tick / Substepの状態検証、Commit / Reject Control
- GPUタイムスタンプ計測対象Pass

GPUはRuntime Physics Stateの唯一の正とする。HostからGPU内部Solver Buffer / Scratch / Writable UAVを公開しない。Render向けにはCommit済みPhysics Stateから生成した専用Read-only Render Bindingを公開する。

### 3.3 GPUリソース構成

初期構成はSoAを基本とし、帯域・strideの実測後にAoSへ変更する。

| バッファ | 形式例 | 更新頻度 | 主な用途 |
|---|---|---|---|
| position_read / position_write | SRV/UAV StructuredBuffer<float4> | 反復ごと | 位置Ping-Pong |
| segment_start_position | StructuredBuffer<float4> | Integration Segment開始ごと | 内部Constraint後のVelocity再構築基準。Contact位置補正前に保持 |
| velocity | RWStructuredBuffer<float4> | Integration Segmentごと / Impulse適用後 | 予測・内部Constraint反映・Contact Impulse後の速度 |
| inverse_mass | StructuredBuffer<float> | 初期化時 | 逆質量 |
| rest_position | StructuredBuffer<float4> | 初期化時 | Rest Pose |
| animation_target | UAV出力 / SRV入力 StructuredBuffer<float4> | Integration Segment開始・終了境界 | Skinning目標位置。CCD prefix時刻は同じTrajectory Evaluatorから取得 |
| animation_trajectory_packet | SRV StructuredBuffer<AnimationTrajectoryRecord> | Logical Tick / Animation Spanごと | Bone単位のTranslation / Rotation / Scale補間係数、Parent、Blend、Root Transform。TOI任意時刻評価の入力 |
| animation_trajectory_bound | SRV StructuredBuffer<AnimationTrajectoryBound> | Animation Spanごと | Mesh単位の保守的な最大速度・最大加速度・Chord Deviation Bound。Swept AABB / CAに使用 |
| animation_velocity | UAV出力 / SRV入力 StructuredBuffer<float4> | Integration Segment開始・終了境界 / TOI時刻 | Skinning済みTarget Trajectoryの時間微分 |
| animation_lambda | RWStructuredBuffer<float4> | Segment反復 | Cloth追従制約の3軸λ Scratch |
| tet_indices | StructuredBuffer<uint4> | 初期化時 | 四面体接続 |
| tet_rest_data | StructuredBuffer<TetRestData> | 初期化時 | Bm、V0、材料ID |
| vertex_tet_offsets / ids | StructuredBuffer<uint> | 初期化時 | Vertex→Tet CSR |
| tet_force_contrib | UAV出力 / SRV入力 StructuredBuffer<float4> | Integration Segment開始状態ごと | 四面体ごとの4頂点力。Segmentごとに再評価 |
| cloth_constraints | StructuredBuffer<ConstraintData> | 初期化時 | Stretch / Shear / Bend |
| constraint_lambda | RWStructuredBuffer<float> | Segment内反復ごと | XPBD λ Scratch |
| vertex_constraint_offsets / ids | StructuredBuffer<uint> | 初期化時 | Vertex→Constraint CSR |
| cell_records | UAV出力 / SRV入力 StructuredBuffer<CellRecord> | Integration Segmentごと | 当該SegmentのSwept AABBによるSpatial Hash登録 |
| contact_candidates | UAV出力 / SRV入力 StructuredBuffer<ContactCandidate> | Integration Segmentごと | 当該SegmentのVT / EE候補 |
| contact_data | SRV/UAV StructuredBuffer<ContactData> | Integration Segment / Contact Iterationごと | 接触法線・係数・λ Scratch。Segment開始時Reset |
| vertex_contact_offsets / ids | StructuredBuffer<uint> | Integration Segmentごと | 現SegmentのContact集合に対するVertex→Contact CSR |
| contact_contrib | UAV出力 / SRV入力 StructuredBuffer<float4> | Contact Iterationごと | Contact位置補正寄与。Velocity再構築には直接加算しない |
| render_binding | StructuredBuffer<RenderBinding> | 初期化時 | Render Vertex対応 |
| diagnostic_buffer | RWStructuredBuffer<uint> | Passごと | Count / Overflow / Error |

float4のw成分の意味はバッファ単位で固定し、CPU/HLSL間で共有する構造体にはサイズ・オフセット検証を入れる。構造体にC++標準のfloat3とDirectXMath型を混在させない。GPU用データはPODの明示レイアウトとする。

GPU ResourceはSRV/UAV両方のViewを必要に応じて作成し、同じBufferを同時にSRVとUAVへBindしない。生成PassはUAV、後続読取PassはUAV unbind後にSRVとして使用する。可変長配列はCount → Prefix Sum → Capacity Check → Fillで構築する。固定上限へ直接書き込むAppend方式で候補を切り捨てない。

## 4. Physics Mesh・隣接データ

### 4.1 共通頂点状態

概念上の状態:

    struct physics_vertex_state
    {
        float4 position;
        float4 segment_start_position;
        float4 velocity;
        float4 predicted_position;
        float inverse_mass;
    };

GPUでは更新頻度に合わせて複数バッファへ分割する。

- inverse_mass = 0: ハード固定頂点
- inverse_mass > 0: 動的頂点
- 負値、NaN、Infinity: アセット構築時または更新時にエラー

Body全体をハード固定しない。骨に直接固定する頂点群と、Animation Trackingで追従する柔軟な頂点群を分ける。

### 4.2 Soft Body四面体

四面体tは頂点x0..x3、Rest行列Dm、逆行列Bm、Rest体積V0、材料IDを持つ。

    Dm = [x1-x0, x2-x0, x3-x0]       // columns
    V0 = abs(det(Dm)) / 6
    Bm = inverse(Dm)

    Ds = [x1_current-x0_current, x2_current-x0_current,
          x3_current-x0_current]
    F = Ds * Bm

形状関数勾配は、Dmの逆行列の各行をN1..N3の勾配とし、N0の勾配はその負和とする。四面体頂点順序は符号付き体積が正になるよう統一する。V0が閾値以下、または条件数が許容範囲外の要素は採用しない。

### 4.3 質量集中

各四面体質量をm_t = density * V0とし、4頂点へm_t / 4ずつ加算する。全四面体からの寄与を頂点単位で総和し、inverse_mass = 1 / massを生成する。共有頂点の質量を二重計上しない。

Clothは三角形ごとにm_face = areal_density * areaを計算し、3頂点へm_face / 3ずつ配分する。体積密度を用いる場合はareal_density = density * thicknessをImporterまたは材料設定で明示する。

### 4.4 物理メッシュ解像度

Soft Body生成設定は次を保持する。

- target_tet_edge_length: 目標四面体辺長（model space meter）
- target_tet_count: 目標四面体数（任意。edge lengthと同時に強制しない）
- max_tet_count: 上限
- surface_approximation_tolerance: 元表面に対する許容誤差
- tetra_quality_threshold: 最小品質閾値
- generator_id / generator_version / generator_options

ユーザーは目標辺長または目標要素数を選択して解像度を調整する。Physics Model Builderは選択された値をオフライン四面体化器の対応パラメータへ変換する。生成器が目標値を厳密に満たせない場合は実際の頂点数・四面体数・辺長分布を記録し、設定との差を警告する。max_tet_count超過は生成失敗とする。キャッシュキーには入力アセット識別子、生成器バージョン、全生成設定を含める。

### 4.5 Asset Cache

Physics Asset CacheにはPhysics Vertex / Element、Rest Pose、四面体Bm・V0・材料ID、質量・逆質量、全CSR隣接表、Cloth制約・Graph Coloring、Skin Weight、Render Vertex Binding、Collision Proxy、Physics Attribute Paintを保存する。Cache Headerにはschema_version、source_asset_id、source_asset_hash、generator_id / version / options、coordinate_system_version、material_layout_versionを含める。必要な識別情報が一致するCacheはそのままロードし、ランタイムで四面体化・Binding再生成を行わない。不一致・破損・未知の必須バージョンはCacheを無効化し、明示的なAsset Build工程で再生成する。Cacheの再生成先と原因を診断ログへ記録する。

## 5. SkinningウェイトのPhysics Meshへの転送

四面体の内部頂点は元のRender Mesh頂点に存在しないため、単純な頂点番号対応はできない。

### 5.1 境界頂点

Physics Mesh境界頂点について、元Render Mesh表面の最近接三角形をRest Poseで検索し、その三角形3頂点のbone weightsを重心座標で補間する。最大8 influenceへ整理し、合計1に正規化する。

### 5.2 内部頂点

内部四面体頂点のbone weightsは、境界頂点のウェイトをDirichlet境界条件とする四面体メッシュ上の調和拡張で求める。初期版では正の辺重みを持つグラフLaplacianを使い、最大値原理を保てる構成にする。

    edge_weight(i,j) = 1 / length(rest_position_i - rest_position_j)^2
    L_ii = sum_j(edge_weight(i,j))
    L_ij = -edge_weight(i,j)

    L_II * weight_interior = -L_IB * weight_boundary

Iは内部頂点、Bは境界頂点。内部頂点を含む各連結成分が少なくとも1つの境界頂点へ接続していることを事前検査し、境界へ接続しない成分は解を定義できないためアセットエラーとする。jointごとに疎行列反復解法を行う。解の残差、各頂点のweight sum、負値を検査し、最後に0以上へ射影・正規化する。Bone influenceが多いアセットでは必要jointごとに解くための計算量を記録する。

### 5.3 GPU Skinning / 任意時刻Animation Trajectory

glTFのSkin Matrixは次で定義する。

    skin_matrix[j] = inverse(mesh_node_global_matrix)
                   * joint_global_matrix[j]
                   * inverse_bind_matrix[j]

inverseBindMatricesが省略されている場合は全Jointで単位行列を使う。Animation Target PositionとAnimation Target Velocityは同じSimulation Timeの最終Skinning Trajectoryから生成し、前Step / Render Frame差分からVelocityを近似しない。

#### Animation SpanとGPU入力Packet

Logical Tick内のAnimation入力を、次の境界でAnimation Spanへ分割する。

- 有効な各ClipのSTEP / LINEAR / CUBICSPLINE Key境界。
- Loop / Ping-Pong等のClip Time wrap境界。
- Blend Weight CurveのKey境界。
- Root Transform / Kinematic Collider Transformの入力Sample境界。
- Teleport / Reset / Time Rewind等の明示Discontinuity境界。

各Animation Spanは単一の補間区間だけを含み、Span内のClip TimeはPlayback Speedを含む線形写像とする。TOIはSpan内で発生し、Key境界を跨がない。Logical Tick内のSpan境界をWorld Integration Segment境界へ追加する。Spanを分割できない欠落Sample / Invalid KeyはBLOCKED_BY_INPUTまたはInvalid Animation Inputとし、古いPoseを延長しない。

CPU Animation Evaluatorは各Spanについて不変のanimation_trajectory_packetを作成し、GPUへUploadする。Packetは次を含む。

- SpanのWorld Simulation Time [t0,t1]、Animation Input Version、Span ID。
- BoneごとのParent Index、Inverse Bind Matrix、Mesh Node参照。
- 最大2本のAnimation Track。各TrackのClip Time区間、Playback Speed、Loop状態、補間種別、Channel有無。
- BoneごとのTranslation / Scale補間係数c0..c3、Rotation補間パラメータ、Key境界情報。
- Blend Weight Curveの係数w0..w3、Discontinuity EventとPolicy。
- Root TransformのTranslation / Rotation Trajectoryと解析微分。
- Kinematic ColliderごとのStable Collider ID、Sample Span [t0,t1]、Translation Linear係数、Shortest-Arc Slerp Quaternion、解析的Linear / Angular Velocity、最大Local Radius。
- Span全域の保守的速度・加速度Boundを構成するCoefficient Interval。Collider BoundにはTranslation Speed、Angular Speed * Maximum Radius、Slerp Sagittaを含める。

GPUはCAの各Iterationで指定されたtauを受け取り、同じPacketを用いてAnimation / Kinematic Transformを評価する。独立Kinematic Colliderはset_kinematic_pose_sampleで受け取った時刻付きTransform SampleをCPUでSpan化し、Translation Linear Interpolation + Shortest-Arc Slerpで評価可能なCollider Trajectory Recordへ変換する。Collider Spanは両端Sampleでbracketし、Span内の位置 / Quaternion / Linear Velocity / Angular Velocityを解析的に算出する。GPU Skinning / Trajectory EvaluatorはBone Local Transform、Bone Global Transform、Skin Matrix、Physics Vertex Target Position / Velocityを同一tauで算出し、Collider Transform Evaluatorは同じtauからCollider Position / Contact Point Velocityを算出する。TOI位置、Contact Relative Velocity、Tracking Targetが異なるPose Sampleを参照してはならない。Animation PacketはTick ReservationへVersion固定し、Retryでは同一Packetを再利用する。

#### CPU Metadata / GPU Packet ABIと時刻精度

CPU SchedulerはWorld Simulation Timeの絶対時刻、Tick Reservation、Animation Span境界をdoubleで保持する。GPU Trajectory PacketはShader Model 5で利用できるuint32 / floatのみを使い、double、bool、Pointer、std::string、std::vectorを含めない。

GPU PacketはSoAを基本とし、少なくとも次の固定幅Recordを使う。

    struct animation_span_gpu_record
    {
        uint32_t span_id;
        uint32_t animation_input_version_low;
        uint32_t animation_input_version_high;
        uint32_t first_channel_record;
        uint32_t channel_record_count;
        uint32_t first_collider_record;
        uint32_t collider_record_count;
        uint32_t flags;
        float span_duration_seconds;
        float inverse_span_duration;
        float u_segment_start;
        float speed_bound;
        float acceleration_bound;
        float chord_deviation_bound;
        float quaternion_normalization_lower_bound;
        float reserved0;
    };

    struct animation_blend_gpu_record
    {
        uint32_t source_track_index;
        uint32_t target_track_index;
        uint32_t weight_curve_flags;
        uint32_t reserved0;
        float4 weight_coefficients;  // scalar Cubic Hermite c0..c3
        float duration_seconds;
        float inverse_duration;
        float reserved1;
        float reserved2;
    };

    struct kinematic_collider_gpu_record
    {
        uint32_t stable_collider_id_low;
        uint32_t stable_collider_id_high;
        uint32_t span_sample_pair_id;
        uint32_t flags;
        float4 translation_start;
        float4 translation_end;
        float4 rotation_start;
        float4 rotation_end;
        float4 angular_axis_and_angle;
        float duration_seconds;
        float inverse_duration;
        float linear_speed_bound;
        float angular_speed_bound;
        float maximum_local_radius;
        float sagitta_bound;
        float reserved0;
        float reserved1;
    };

    struct animation_channel_gpu_record
    {
        uint32_t bone_index;
        uint32_t parent_index;
        uint32_t interpolation_flags;
        uint32_t channel_mask;
        float4 translation_c0;
        float4 translation_c1;
        float4 translation_c2;
        float4 translation_c3;
        float4 rotation_parameter0;
        float4 rotation_parameter1;
        float4 rotation_parameter2;
        float4 rotation_parameter3;
        float4 scale_c0;
        float4 scale_c1;
        float4 scale_c2;
        float4 scale_c3;
    };

animation_span_gpu_record is exactly 64 bytes, animation_channel_gpu_record is exactly 208 bytes, animation_blend_gpu_record is exactly 48 bytes, and kinematic_collider_gpu_record is exactly 128 bytes under 4-byte field packing. C++ `static_assert(sizeof(...))`, `static_assert(alignof(...) == 4)`, `offsetof` assertions, HLSL StructuredBuffer stride validation, and a WARP GPU ABI Readback Fixture verify the same values. `animation_span_gpu_record::u_segment_start` is the normalized Animation Span coordinate at the current Integration Segment start. After every positive-time Segment advance within the same Span, GPU updates it by `u_segment_start += dt_segment * inverse_span_duration`; a TOI prefix uses `tau` in place of `dt_segment`. If the next Segment begins at a Span boundary, Scheduler selects the next immutable Span Record and initializes its `u_segment_start=0`. GPU verifies that every positive-time advance makes u_segment_start strictly increase; failure returns `ANIMATION_TIME_PRECISION_UNRESOLVED` and prevents Commit. No CPU Readback is used to update the normalized cursor.

固定RecordのByte Offsetは次の通りとする。OffsetはBuffer先頭からのbyte単位であり、全Fieldは4-byte alignmentとする。

| Record | Field Offset |
|---|---|
| animation_span_gpu_record (64 B) | span_id=0, version_low=4, version_high=8, first_channel=12, channel_count=16, first_collider=20, collider_count=24, flags=28, span_duration=32, inverse_span_duration=36, u_segment_start=40, speed_bound=44, acceleration_bound=48, chord_deviation_bound=52, quaternion_normalization_lower_bound=56, reserved0=60 |
| animation_channel_gpu_record (208 B) | bone_index=0, parent_index=4, interpolation_flags=8, channel_mask=12, translation_c0=16, translation_c1=32, translation_c2=48, translation_c3=64, rotation_parameter0=80, rotation_parameter1=96, rotation_parameter2=112, rotation_parameter3=128, scale_c0=144, scale_c1=160, scale_c2=176, scale_c3=192 |
| animation_blend_gpu_record (48 B) | source_track_index=0, target_track_index=4, weight_curve_flags=8, reserved0=12, weight_coefficients=16, duration=32, inverse_duration=36, reserved1=40, reserved2=44 |
| kinematic_collider_gpu_record (128 B) | stable_id_low=0, stable_id_high=4, span_sample_pair_id=8, flags=12, translation_start=16, translation_end=32, rotation_start=48, rotation_end=64, angular_axis_and_angle=80, duration=96, inverse_duration=100, linear_speed_bound=104, angular_speed_bound=108, maximum_local_radius=112, sagitta_bound=116, reserved0=120, reserved1=124 |

Collider RecordはStable Collider ID、Span Sample Pair ID、Translation係数、Shortest-Arc Slerp Quaternion、Maximum Local Radius、Linear / Angular Velocity Bound、Sagitta Boundを固定幅uint32 / floatで保持する。C++側とHLSL側でsizeof / offsetof / stride / field semanticの一致をStatic AssertionとGPU ABI Readback Fixtureで検証する。float3、DirectXMath型、enum classの暗黙Underlying Type、C++ boolをGPU共有Recordへ直接持ち込まない。

CPUはdoubleのSpan開始・終了時刻とSegment開始時刻から、正規化Span座標u_segment_startを計算してfloatへ変換する。GPUへの時間入力はAbsolute World Timeを含めず、Segment開始からの相対時間tau [s]だけを渡す。

    u_tau = u_segment_start + tau * inverse_span_duration
    u_tau = clamp(u_tau, 0, 1)

TOIが確定して残Segmentを開始するときは、GPUがu_segment_start = u_tauへ更新し、同じAnimation Span / Packet Versionを継続使用する。CPUへTOI時刻Readbackして再Uploadする同期経路は作らない。Animation Key / Loop / Blend / Root / Collider Sample境界をSpan終端としているため、u_tauが1へ到達する前に次Spanへ跨がない。

CPUはdouble→float変換後のSpan Duration / inverse duration / u_segment_start / Speed Boundを検査する。正の有効Segment時間をGPU floatで表現できない、u_tauがtau増加に対して単調に進まない、または境界判定誤差がSpan内許容値を超える場合はAnimation Spanを決定的に細分化する。細分化深度8でも表現できなければBLOCKED_BY_INPUT / CCDUnresolvedとしてCommitしない。絶対World Timeをfloatへ直接CastしてShaderへ渡す実装は禁止する。

Speed / Acceleration / Deviation BoundをfloatへUploadする場合は外向きに丸め、Boundを小さくしない。Boundの量子化後も同じNo-Miss条件を維持できない場合はSpanを分割し、それでも証明できない入力を拒否する。

#### Channel補間と時間微分

Span内正規化時間をu=(t-t0)/(t1-t0)、u in [0,1]とする。Translation / Scaleの各Componentは次の多項式で表す。

    p(u) = c0 + c1*u + c2*u^2 + c3*u^3
    dp/dt = (c1 + 2*c2*u + 3*c3*u^2) / (t1 - t0)

STEPはc0=value、他係数0。LINEARはc0=p0, c1=p1-p0、c2=c3=0。glTF CUBICSPLINEはSpan両端の値p0 / p1、符号付きClip Time幅Δcを使い、正方向Playbackではp0のOutgoing Tangentとp1のIncoming Tangent、逆方向Playbackではp0のIncoming Tangentとp1のOutgoing Tangentをm0 / m1として選ぶ。h0=m0*Δc、h1=m1*Δcとし、c0=p0、c1=h0、c2=-3p0+3p1-2h0-h1、c3=2p0-2p1+h0+h1へ変換する。ΔcにはPlayback SpeedとWorld Span時間を含める。負のPlayback SpeedではTangentsのKey側も方向に応じて入れ替える。ScaleはSpan全域で正かつ特異でないことを保証する。

RotationはQuaternionを使う。STEPは一定Quaternion、LINEARは最短弧Slerp、CUBICSPLINEはglTF Hermite QuaternionをComponentごとに評価後normalizeする。CUBICSPLINEの未正規化Quaternionをr(t)、q=r/|r|とすると、

    q_dot = (r_dot - q * dot(q, r_dot)) / |r|

を使用する。SlerpのQuaternion / Angular Velocityも解析的に評価する。Quaternionの符号はSpan開始で連続になるHemisphereへ揃える。CUBICSPLINEの|r|の保守的下限が正規化epsilon以下になる区間は決定的に細分化し、なお保証できなければInvalid Animation Inputとして拒否する。正規化Quaternionの有限差分でVelocityを代用しない。

#### Clip Blend

v1は同時に2本までのAnimation Trackを許可し、Additive Animationは対象外とする。Translation / Scaleは、

    p_blend = (1-w) * p_source + w * p_target
    v_blend = (1-w) * v_source + w * v_target
              + w_dot * (p_target - p_source)

で評価する。RotationはSource / Target QuaternionのHemisphereを揃えた後、

    r = (1-w) * q_source + w * q_target
    q_blend = normalize(r)
    q_dot_blend = (r_dot - q_blend * dot(q_blend, r_dot)) / |r|

とする。Blend Weightは0～1のCubic Hermite Curveで、u=(t-t_blend_start)/T、T=t_blend_end-t_blend_startとして、w(u)=(2u^3-3u^2+1)w0+(u^3-2u^2+u)T*m0+(-2u^3+3u^2)w1+(u^3-u^2)T*m1を使う。m0 / m1はWorld Time微分[1/s]。dw/dtはこの式のu微分をTで除して求める。Curve全域で0<=w<=1を区間検証し、範囲外へOvershootするBlendは入力Errorとする。Rotation Blendはr_dot=(1-w)*q_dot_source+w*q_dot_target+w_dot*(q_target-q_source)を使ってq_dot_blendを計算する。Span PacketにWeight係数と微分係数を保持する。3本以上の同時Blend / Additiveはv1で明示Unsupportedとする。

#### Hierarchy / Skinning微分

Bone Global MatrixはParent順に評価し、微分は積の微分を使う。

    M_global = M_parent * M_local
    M_global_dot = M_parent_dot * M_local
                    + M_parent * M_local_dot

Skin Matrixの微分には逆行列の微分を含める。

    A = inverse(mesh_node_global)
    A_dot = -A * mesh_node_global_dot * A
    S_j = A * joint_global[j] * inverse_bind[j]
    S_j_dot = A_dot * joint_global[j] * inverse_bind[j]
              + A * joint_global_dot[j] * inverse_bind[j]

Linear Blend SkinningではInfluence Weight付きでSkin Matrix / Matrix Derivativeを合成し、Physics Vertex Rest Positionへ適用してTarget Position / Velocityを得る。

#### Conservative Trajectory Bounds

Animation Packet生成時にCPUは各Spanの係数から区間演算（Interval Arithmetic）でLocal Transformの速度・加速度上限を算出する。Matrix積の微分BoundはProduct Ruleを上限演算へ適用し、Inverse Matrixは最小特異値の下限が確保できる場合だけ評価する。SkinningではInfluence Weightの総和と非負性を検証し、Bone / Mesh Node / Root Transformを通じてPhysics Mesh全体の保守的な最大頂点速度v_animation_maxと最大加速度a_animation_maxを求める。上限演算は丸め誤差を外側へ膨張させ、実値を過小評価しない。GPUはこのBoundを改変せず利用する。

Twice differentiableなSpanでは、Endpoint Chordに対する最大偏差を、

    deviation_bound = a_animation_max * (t1 - t0)^2 / 8

で上限化する。STEP / Teleport境界は同一Spanへ跨がせず、Span境界で明示Discontinuityとして処理する。Slerp Colliderの回転は最大半径r_max、区間角度thetaからSagitta Boundを求め、角速度上限をSpeed Boundへ含める。Boundが有限に得られないSpanは最大深度8まで分割し、それでも有限Boundを証明できなければCCDUnresolved / BLOCKED_BY_INPUTとしてCommitしない。

このBoundはMesh全体で保守的に一つの上限を使う初期方式であり、AABBを過大に膨張させる場合がある。最適化でMesh Cluster / Primitive単位へ細分化しても、Boundの保守性とNo-Miss条件を緩和してはならない。

Animation EvaluatorがTarget Position / Velocity / Boundを同じSpan Packetから供給できない区間はBLOCKED_BY_INPUTとする。Render用PoseとPhysics用Poseの時間軸を混在させない。

## 6. Soft Body Solver詳細

### 6.1 採用モデルと適用範囲

初期版は四面体Corotational Linear Elasticityを採用する。有限回転を扱うが、ひずみ自体は線形弾性近似である。極端な圧縮・伸長を無制限に正確に表現するモデルではない。大変形試験で品質が不足する場合は、Neo-Hookean等の超弾性モデルを次版候補として比較する。

### 6.2 材料パラメータ

UIで独立調整可能な基準材料値は、せん断弾性率mu、体積弾性率K、密度rhoとする。加えて、要件で求める元形状への復元と体積保持の個別調整用に、shape_recovery_scaleとvolume_retention_scaleを設ける。両Scaleは0以上の無次元値で、初期値1.0。muとKはいずれも正値。muとKは6.3節のCorotational応力へ直接使う。shape_recovery_scaleとvolume_retention_scaleがともに1の場合、従来のLame定数はlambda_lame = K - 2 * mu / 3に相当する。

EとPoisson比nuで入力する場合は以下に変換する。

    mu = E / (2 * (1 + nu))
    K = E / (3 * (1 - 2 * nu))

初期UIではnuを0.0～0.45に制限する。E/nuモードとmu/Kモードを混在させず、内部保存値はmu/K/rhoへ統一する。体積とせん断の応答を別々に調整できる。Soft Bodyの各四面体に使うscale値は、その4頂点のPaint値の算術平均とする。Paint未設定頂点はMesh基準値1.0を使う。

### 6.3 Corotational応力

変形勾配Fの右極分解をF = R * Sとする。Rはdet(R)=+1を満たす回転、Sは対称ストレッチである。Corotational Linear Elasticityのひずみエネルギーと第一Piola応力は次を基準とする。norm(A)^2はFrobenius norm squared、すなわちtrace(transpose(A)*A)とする。

    epsilon_corot = transpose(R) * F - I
    trace_epsilon = trace(epsilon_corot)
    epsilon_dev = epsilon_corot - (trace_epsilon / 3) * I

    mu_effective = mu * shape_recovery_scale
    K_effective = K * volume_retention_scale

    Psi = mu_effective * norm(epsilon_dev)^2
         + 0.5 * K_effective * trace_epsilon^2

    P = R * (2 * mu_effective * epsilon_dev
           + K_effective * trace_epsilon * I)

shape_recovery_scaleは主にせん断・偏差ひずみへの復元抵抗を、volume_retention_scaleは体積ひずみへの抵抗を独立に調整する。両方を1にしたとき、従来のCorotational Linear Elasticityと一致する。

初版の応力をlambda_lame * (det(F)-1) * cofactor(F)と置き換えない。この項は上記Corotational Linear Elasticityの体積応力と同じ式ではない。

### 6.4 回転抽出・反転要素

通常要素はScaled Newton polar iterationを使用する。

    R0 = F
    A = transpose(inverse(Rk))
    gamma = sqrt(frobenius_norm(A) / frobenius_norm(Rk))
    R_next = 0.5 * (gamma * Rk + A / gamma)

各反復で直交性残差 norm(transpose(R)*R - I) と更新差を検査する。最大反復回数は初期値8とし、収束しない場合はSVD補助Passへ送る。det(F)が正で、最小特異値が閾値を超え、Rがdet(R)=+1となる要素のみ通常経路で確定する。

異常要素は別Passで3x3 Jacobi SVDを行う。F = U * Sigma * transpose(V)とし、特異値を降順に並べる。必要な場合は最小特異値の軸を反転し、D=diag(1,1,det(U*transpose(V)))としてR=U*D*transpose(V)を構成する。特異値が閾値以下の場合は下限へクランプし、非有限値を出さない。SVD補助Passの対象数とGPU時間を計測する。

さらに反転領域には以下の体積ペナルティを加える。これは有限の二次ペナルティであり、数学的な非反転ペナルティとして反転を絶対に禁止するものではない。k_inversionの単位は応力と同じPaとする。

    J = det(F)
    Psi_inversion = 0.5 * k_inversion * min(J - J_min, 0)^2
    P_inversion = k_inversion * min(J - J_min, 0) * cofactor(F)

総応力はP_total = P + P_inversion。ペナルティはJ >= J_minで0となる。Jが閾値を下回る要素には診断フラグを付ける。反転が一定数・時間以上続く場合はSimulation Warning/Errorとし、単に要素を無効化して力を消さない。

### 6.5 四面体内部力

Rest Shape Function勾配を用い、頂点iの内部力を次で求める。

    f_internal_i = -V0 * P_total * grad_rest(N_i)

四面体単位で4頂点への寄与を出力し、Vertex→Tet CSRを使って頂点ごとに総和する。

    force_vertex_i = sum_over_incident_tets(force_tet_i)

力は総和であり、隣接四面体数で平均化しない。浮動小数点Atomic Addは使わない。

### 6.6 時間積分・減衰

v1では全ての変形Physics VertexをWorld共通Integration Segment単位で更新する。通常Substep、Force / Impulse Event境界、TOI境界のいずれでも同じ積分規則を使用する。ここでいうSegmentは正の有限時間幅を持つ区間であり、Segment開始状態を (x_segment_start, v_segment_start)、区間時間をdt_segmentとする。

Dynamic Vertexに対するSegment中の運動方程式は、Segment内で評価した合力F_segmentを一定とみなす線形粘性減衰モデルとする。

    dv/dt = inverse_mass * F_segment - damping_rate * v

F_segmentはSolverごとに次のように構成する。

- Soft Body: gravity / User Force / Corotational FEM internal force / Kelvin-Voigt Animation Tracking forceの合計。内部力とTracking forceはSegment開始状態 (x_segment_start, v_segment_start) および同時刻のAnimation Target / Velocityで評価し、当該Segment内では固定する。
- Cloth: gravity / User Forceの合計。Stretch / Shear / Bend / Animation Tracking / Contactは位置制約としてSegment内で解く。Clothへ将来Force型の追加項を導入する場合はF_segmentへ加え、単位と安定時間刻みを更新する。

User ImpulseはForceへ変換しない。Impulse J_segmentはSegment開始境界で一度だけ速度へ加え、その直後から当該Segmentの減衰を受ける。Force Eventの有効状態はSegment内で一定でなければならない。Impulse Eventの時刻は必ずSegment境界へ挿入する。

数値計算は次の順で行う。damping_rateは0以上の有限値とする。

    z = damping_rate * dt_segment
    decay = exp(-z)

    damping_integral =
        dt_segment                                      // z == 0
        dt_segment * (1 - z/2 + z^2/6 - z^3/24)        // 0 < z < 1e-3
        (1 - decay) / damping_rate                    // z >= 1e-3

    v_after_impulse = v_segment_start
                    + inverse_mass * J_segment

    acceleration = inverse_mass * F_segment

    v_predicted = decay * v_after_impulse
                + damping_integral * acceleration

    x_predicted = x_segment_start + dt_segment * v_predicted

damping_integralは一定加速度に対する線形粘性減衰の速度積分係数である。小さいzでは級数近似を使い、1-exp(-z)の桁落ちを避ける。上式は速度についてSegment内一定加速度・線形減衰の解を使い、位置について半陰的Euler更新を行う。Soft Bodyの非線形内部力はSegment開始時に評価するため、この一定力近似の誤差をWorld Substep / TOI Segment解像度と収束試験で評価する。

inverse_massが0のFixed / Kinematic Vertexには上記のDynamic更新式を適用しない。代わりにSegment終端時刻のAnimation Targetと、その時刻におけるAnimation Target Trajectoryの時間微分を設定する。

    x_predicted = animation_target(t_segment_end)
    v_predicted = animation_velocity(t_segment_end)

Animation Velocityは同一内部時間軸でサンプリングされたTarget Trajectoryの時間微分とする。線形補間では区間勾配、CUBICSPLINEでは評価曲線の終端微分を使う。Velocity Sampleが得られない場合に前Frame値を流用しない。必要なPose Bracketが欠落していればBLOCKED_BY_INPUTとする。

XPBD位置解決後の速度再構築はDynamic Vertexだけに適用する。Fixed / Kinematic Vertexは再構築値で上書きせず、Animation Interfaceが与えた終端Velocityを維持する。Contact Impulse / Friction / Restitutionは逆質量に従ってDynamic Vertexへだけ速度反映し、Kinematic VertexのVelocityは変えない。Kinematic Velocityは相対速度、摩擦、反発、CCDの入力として使用する。

Impact後に次Segmentが存在する場合、Impact後の位置・速度を次Segment開始状態とする。Logical Tick全体の始点・終点差分でImpact後Velocityを再構築してはならない。

### 6.7 Animation Tracking力

柔軟頂点の追従は材料弾性と分離したKelvin-Voigt型追従力とする。物理頂点はワールド空間で積分し、アニメーション目標位置は各Stepのスキニング結果から生成する。骨格の並進・回転で目標位置と目標速度が変化し、動的頂点の慣性と追従力の差によって遅れ・揺れが生じる。シミュレーション中に動的頂点をBone Localへ毎フレーム再配置しない。

    force_animation =
        tracking_weight * (
            tracking_stiffness * (animation_target - position)
          + tracking_damping * (animation_velocity - velocity))

tracking_stiffnessはN/m、tracking_dampingはN*s/m。tracking_weightは0～1。材料のmu/Kと独立して調整できる。過大な値は時間刻み安定条件に影響するため、後述の安定性推定に含める。

### 6.7.1 アニメーション不連続イベント

Animation Interfaceは通常再生、Clip Blend、Teleport、Reset、Time Rewindを区別する。Clip Blendは最大2 Trackであり、Source / Targetの補間結果とBlend Weight Curveを§5.3のPacketで評価する。Additive / 3 Track以上のBlendはUnsupportedとする。Discontinuity Eventでは次のPolicyを選択する。

- preserve_velocity: Dynamic Physics Velocityを直前値のまま維持し、目標Pose / Trajectory履歴だけを新しいSpanへ合わせる。
- reset_velocity: Dynamic Physics Position / Velocityを新Target Position / Trajectory Velocityへ再初期化し、Contact / CCD CacheとSegment Snapshotを無効化する。
- blend_velocity: Dynamic Positionは維持し、Dynamic Velocityをv_after=(1-w_event)*v_before + w_event*v_target(t_event+)へ再初期化する。w_eventはDiscontinuity Descriptorの0～1値。Target / Velocity履歴をEvent後Spanへ合わせる。

通常のClip Blend遷移はTrajectory自体の微分へw_dot項を含めるため、Policy適用によるVelocity Resetを要求しない。Discontinuity Policyは明示イベントにだけ適用する。Teleport / Reset / Time RewindではContact / CCD Cache、前回位置、速度、Segment SnapshotをPolicyに従って再初期化する。

### 6.8 明示積分の安定時間刻み

Segment開始時に評価した陽的内部力と半陰的Eulerによる位置更新は条件付き安定であり、速度に線形粘性減衰を厳密適用しても、1/240秒にしただけで全メッシュ・全材料の安定性を保証しない。

材料の縦波速度を

    c_p = sqrt((K_effective + 4 * mu_effective / 3) / density)

各四面体の最小高度を

    h_t = 3 * V0 / max_face_area

として、初期サブステップ上限の推定値を次とする。Animation Tracking stiffnessがある場合は、頂点質量m_iと追従剛性k_iからomega_i=sqrt(k_i/m_i)を求め、delta_t_tracking_limit=c_tracking/omega_maxも評価する。c_trackingは初期値0.2として実測検証する。最終上限はFEMとTrackingの小さい方を採用する。

    delta_t_limit = cfl_factor * min_t(h_t / c_p_t)
    cfl_factor = 0.2  // 初期評価値。実測で検証する

これは要素形状・材料・境界条件を含めた厳密な安定保証式ではなく、初期の保守的スクリーニングである。

ClothはXPBDであるため、ここでの時間刻み上限はFEM CFLのような明示積分の安定保証ではなく、拘束振動数に基づくTemporal Quality Limitとする。各Constraint cについて、Rest / Current Geometryで評価したGradientと有効逆質量から、

    omega_c = sqrt(k_c * sum_i(inverse_mass_i * dot(gradient_i, gradient_i)))
    omega_cloth_max = max_c(omega_c)
    dt_limit_cloth_quality = c_cloth / max(omega_cloth_max, epsilon_omega)

と推定する。初期係数c_cloth=0.2はEngineering Baselineであり、Stretch / Shear / Bendの解像度収束・反復収束試験で校正する。全頂点固定、Compliance無限大、または拘束剛性0のConstraintは推定対象から除外する。推定値は厳密安定保証と称さず、品質上限として使う。

World共通Substep ScheduleはFEM CFL、Animation Tracking、Cloth Temporal Quality Limit、Contact Temporal Quality Limit、World設定上限の最小値を集約する。Cloth品質LimitをScenarioで無効にした場合はその項を無限大として扱う。Contact Compliance 0のContactに固有振動数を割り当てず、contact_max_substep_delta_timeを独立した時間品質上限として使用する。初期値は1/120 s、0は明示無効とする。0を指定しworld_max_substep_delta_timeも0ならContact固有のdt上限を適用しない。
上限超過時は警告し、設定に応じてWorld共通Substep数を増やす。最大World Substep数を超えたらWORK_BUDGET_EXCEEDEDとしてLogical TickをCommitせず、時間を保持する。Soft Bodyだけを細かく進めてCloth / Contactを旧時刻に残す独立Subcyclingはv1で禁止する。

World共通Substep数はLogical Tick開始時に一度だけ決定する。全参加Soft Body MeshのFEM / Animation Tracking制限、全参加Cloth MeshのTemporal Quality Limit、Contact Temporal Quality Limit、World設定のminimum_substeps / maximum_substepsを集約し、

    dt_limit_world = min(all_mesh_dt_limit_fem,
                         all_mesh_dt_limit_tracking,
                         all_mesh_dt_limit_cloth_quality,
                         contact_max_substep_delta_time if > 0 else +infinity,
                         world_max_substep_delta_time if > 0 else +infinity)

    N_world = max(minimum_substeps,
                  ceil(fixed_delta_time / dt_limit_world))

とする。contact_max_substep_delta_time=0は+infinityとして扱い、world_max_substep_delta_time=0も+infinityとする。全項が無制限ならN_worldはminimum_substepsとなる。N_world > maximum_substeps、整数オーバーフロー、または非有限値はTickを拒否する。実際のSubstep時間はdt_world = fixed_delta_time / N_worldとし、全Soft Body / Cloth / Contact / Animation Trackingへ同一値を渡す。ClothのXPBD ComplianceとContact Complianceは必ずこのdt_worldから算出する。

TOI分割はWorld共通Substepの内部Segmentとして扱う。TOI以前・TOI以後の双方でSoft Body、Cloth、Animation Tracking、Contact対象の全てを同じSegment時間だけ進める。v1ではSoft BodyだけをTOIまで進めてClothを固定するようなソルバー別時間進行を行わない。minimum_substeps、maximum_substeps、world_max_substep_delta_timeはWorld Descriptor / Scenarioに保存し、Run Manifestへ記録する。

### 6.9 体積・せん断評価

muとshape_recovery_scaleは主に元形状への復元・せん断応答、Kとvolume_retention_scaleは体積応答を制御する。局所圧縮・膨張試験では要素ごとのJと全体体積を別々に記録する。Corotational Linear Elasticityは大変形時に線形化誤差が増えるため、体積10～30%変化、局所押込み、反転直前の試験を行い、適用範囲を数値で示す。

## 7. Cloth Solver詳細

### 7.1 XPBD共通式

制約関数C(x)、Compliance alpha、実際のWorld共通Integration Segment時間dt_segment、Segment内の制約ごとのlambdaを用いる。TOIおよびForce / Impulse境界でWorld共通Substepが分割された場合、各Segmentは独立したXPBD Integration Intervalとなる。

    alpha_tilde = alpha / (dt_segment * dt_segment)
    delta_lambda_raw =
        (-C(x) - alpha_tilde * lambda_old)
        / (sum_i(inverse_mass_i * dot(gradient_i, gradient_i))
           + alpha_tilde)

Compliance alphaは制約関数の単位に依存する。エディタで扱う材料値は物理的な伸縮弾性率・せん断弾性率・曲げ剛性とし、Physics Mesh構築時にRest Geometryを使って各制約の剛性kへ変換する。各制約のalphaは1/kとする。制約関数の単位、kの単位、alphaの単位をConstraintDataに記録する。

- Stretch距離制約: Cはm、kはN/m、alphaはm/N。
- Shearひずみ制約: Cは無次元、kはJ、alphaは1/J。
- Bend角度制約: Cはrad、kはN*m、alphaは1/(N*m)。

初期版のCloth材料入力は面内引張剛性E_2D (N/m)、面内せん断剛性G_2D (N/m)、曲げ剛性D (N*m)とする。Stretch辺のdual width wは内部辺では隣接2三角形の辺直交高度の平均、境界辺では隣接三角形の高度の半分とし、Rest LengthをL0とするとk_stretch=E_2D*w/L0。Shear三角形はk_shear=G_2D*area。Bend hingeのdual width wは隣接2三角形の辺直交高度の平均とし、共有辺長lを使いk_bend=D*w/lとする。これらは離散化の初期モデルであり、解像度変更時のエネルギー一致を試験する。

分母が閾値以下、全頂点が固定、または値が非有限の場合は補正を出さず診断する。Contactについては§8.5のepsilon_inverse_massで応答可能性を先に判定し、非応答ContactのLambdaを生成しない。Cloth Stretch / Shear / Bend / Animation Tracking / Contactの各XPBD λは、各World共通Integration Segment開始時に0へ初期化し、そのSegment内の全Solver反復で保持する。Segment境界（通常Substep境界、Force / Impulse境界、TOI前後）では必ず0へ戻し、Segment間およびLogical Tick間のWarm Startはv1で行わない。すべてのComplianceは当該Segmentのdt_segmentからalpha_tilde=alpha/(dt_segment*dt_segment)を算出する。

### 7.1.1 Cloth動力学予測・外力適用

Clothの予測はWorld共通Integration Segmentごとに1回行う。World Substepの開始時だけに限定しない。Force / Impulse Event境界またはTOIでSubstepが分割された場合、直前Segmentの終了状態を次Segmentの開始状態として使い、次Segmentのdt_segmentで予測する。重力は一様加速度としてPhysics Vertex iへmass_i * gravityで適用する。

ユーザーForce / Impulse Descriptorはワールド空間の作用点・方向・半径を持ち、Soft Body / ClothのPhysics Vertexへ同じ質量測度付きFalloffで分配する。頂点iのRest Lumped Massをm_i、Segment開始位置から作用点までのワールド空間距離をr_i、Force半径をRとする。Rは正の有限値を必須とし、サポート内頂点に対して、

    s_i = r_i / R
    phi(s_i) = (1 - s_i)^4 * (4 * s_i + 1)   // 0 <= s_i < 1
    q_i = m_i * phi(s_i)
    w_i = q_i / sum_j(q_j)

とする。s_i >= 1の頂点はq_i=0。質量はSoft Bodyの四面体Lumped Mass、Clothの三角形Lumped Massから得る。sum_j(q_j)は同じForce Descriptorの対象World / Model / Mesh範囲内で動的・固定を含む全サポート頂点について求める。これによりDescriptorのForce / Impulse合計量を頂点数やMesh解像度によらず保存する。固定頂点へ配分された分は拘束反力として扱う。サポート頂点が0、または重み総和が閾値以下の場合はNO_FORCE_TARGETを記録し、そのEventを適用しない。

ForceのMagnitudeはN、ImpulseのMagnitudeはN*sとし、方向は正規化済みベクトルとする。Force Eventの開始・終了時刻とImpulse Event時刻はWorld共通Segment境界へ追加する。Segment時間区間は半開区間[t_segment_start, t_segment_end)として扱い、ForceはSegment内で有効状態を一定にする。Impulseは時刻を含むSegmentの開始境界で一度だけ適用する。Logical Tick終端と一致するEventは次のLogical Tick開始境界に属する。イベントIDとTick Reservation内の消費記録を保持し、Retry / TOI prefix構成で同じImpulseを二重適用しない。0時間または負時間のSegmentを生成しない。

各外部Event eは対象Filter内で独立にw_i,eを正規化する。Segment開始時のPhysics Vertex iへの入力は、

    F_external_i = m_i * gravity
                 + sum_e(w_i,e * direction_force_e * magnitude_force_e)

    J_segment_i = sum_events_at_segment_start(
                      w_i,e * direction_impulse_e * magnitude_impulse_e)

とする。Force開始・終了境界でWorld共通Segmentを分割するため、ForceはSegment全体へ作用する。ImpulseはSegment開始時に一度だけ適用する。F_externalとJ_segmentを同じForceへ変換して二重加算しない。

Clothには§6.6の共通Segment積分式を適用し、F_segment = F_external、J_segment = J_segment_iとする。Soft Bodyは§6.6の定義に従いFEM internal forceとKelvin-Voigt Animation Tracking forceもF_segmentへ加える。ClothのAnimation TrackingはXPBD Attachment Constraintとして位置解決時に扱う。

Cloth Stretch / Shear / Bend / Animation Tracking等の非接触内部Constraintを解決した後、Contact Position Constraintを解く前にDynamic Vertexだけについて、

    v_pre_contact = (x_internal_solved - x_segment_start) / dt_segment

でContact前Velocityを再構築して保存する。Contact Position XPBDはPositionだけを補正し、保存済みv_pre_contactを変更しない。Fixed / Kinematic Vertexは§6.6に従い、Animation TargetとAnimation Trajectory Velocityを維持する。Friction / Restitution Impulseは保存済みContact前Velocityから一度だけ計算し、Kinematic VertexのVelocityは変更しない。Impact後のVelocityは次Segmentへ引き継ぐ。Logical Tick全体の始点・終点差分でImpact後Velocityを上書きしない。

### 7.1.2 World Integration Segmentの処理順

1. Logical Tick内のWorld Substep境界、Animation Span Key境界、Force開始 / 終了、Impulse時刻を昇順に統合する。World Substep境界も含めてLogical Tick内の全Segment数を算出する。予定Segment数がRuntime Descriptorのmaximum_integration_segments_per_tick（初期値512）を超える場合はGPU Physics Pass開始前にWORK_BUDGET_EXCEEDEDとしてTick Reservationを保持する。TOIで残Segmentを追加する場合もSegment数へ加算し、上限を超えるなら残SegmentをDispatchしない。等しい時刻のEventはEvent ID順でまとめ、重複境界を除去する。異なる時刻のKey / Event境界を結合・省略しない。隣接する境界時刻が同一または数値的に逆転した場合は同一境界へ統合し、dt_segment <= 0のSegmentを発行しない。Animation / Force / Impulseの確定境界から算出したLogical Tick内Segment数がmaximum_integration_segments_per_tickを超える場合、GPU Physics Pass開始前にWORK_BUDGET_EXCEEDEDとしてTick Reservationを保持する。Key境界や異なる時刻のEvent境界を結合・省略して上限内へ丸めない。
2. Segment開始時にPhysics Position / Velocity、Animation Input Version、Impulse Event Cursor / 適用済みEvent ID状態をSnapshotする。Segment境界でXPBD Lambda / Contact Lambda / Impact Scratchを初期化する。
3. Animation Span PacketをSegment開始前に固定し、Segment開始・終了時刻およびTOI任意時刻のTarget / Velocityを同じGPU Trajectory Evaluatorで取得する。Span Key境界はSegment境界へ追加済みとする。必要なSample Bracket / Packet / Boundが揃わない場合はBLOCKED_BY_INPUTとし、予測へ進まない。
4. Segment開始位置から各Force DescriptorのFalloff Weightを計算する。Gravity / ForceはSegment内で一定とし、ImpulseはSegment開始時刻のEvent IDを確認して一度だけ消費する。RetryはSnapshotと同じEvent Cursorから再生し、破棄されたWorking Stateへの適用を重複累積させない。
5. Soft BodyはSegment開始位置・速度からCorotational FEM内部力とKelvin-Voigt Tracking力を評価する。Clothは外力を評価し、両Solverは§6.6の共通速度積分規則で予測する。Fixed / Kinematic VertexはSegment終端Animation Target / Velocityを設定する。
6. 全参加Physics MeshのSwept AABBを作り、Broadphase / Narrowphase / CCD候補を生成する。Soft Body / Cloth / Contactは同じSegmentの予測時刻を参照する。
7. TOIがない場合は、Cloth Stretch / Shear / Bend / Animation Tracking等の非接触内部Constraintを規定反復数だけ解く。
8. Positive-Time TOIがある場合は、次の残Segmentを含む実行済み / 予定Segment数がmaximum_integration_segments_per_tick以下であることを先に確認する。上限を超える場合は同じTickをCommitせずReservationを保持する。§11.4で定義したSemi-implicit Euler離散pathを使ってprefix状態を構成する。Dynamic Vertexはx_tau = x_segment_start + tau * path_velocity、v_tau = path_velocityとし、dt=tauのVelocity Predictionを再実行しない。Fixed / Kinematic Vertexはtau時刻のAnimation Target Trajectory位置を直接評価し、接触相対Velocityには同時刻のTrajectory微分を使う。Endpoint ChordはSwept AABBの基準とし、Deviation Boundで膨張させる。TOI prefixにおける非接触内部Constraintをdt=tauで解く。
9. Discrete / TOIの双方で、非接触内部Constraint後・Contact Position Constraint前にDynamic Contact前Velocityを再構築して保存する。通常Segmentではdt_segment、TOI prefixではtauを使う。次にContact候補を現在の内部Constraint後Positionで再検査し、接触が成立しているものだけContact Position XPBDを解く。Contact位置補正はPositionのみを変更し、保存済みContact前Velocityを変更しない。
10. 保存済みContact前VelocityとKinematic接触点VelocityからFriction / Restitution Impulseを一度適用する。TOI Impactはimpact_resolvedに記録し、Impact後Velocityから残時間の新Segmentを開始する。Contact位置補正後またはImpact後に同じSegmentのVelocity Reconstructionを実行しない。TOI候補がprefix再検査で成立しない場合はImpactを発生させず、tau位置から残時間を継続し、必要ならCCD候補を再構築する。

Soft BodyとClothの内部解法は異なるが、時間境界・Force / Impulse Eventの意味・Kinematic Velocity所有者はWorld共通である。各Substepの単純な1回更新へ戻す実装、Solverごとの異なるSegment時間、固定頂点への差分Velocity上書きは禁止する。

### 7.2 Stretch

辺(i,j)の制約:

    C_stretch = length(x_i - x_j) - rest_length
    gradient_i = normalize(x_i - x_j)
    gradient_j = -gradient_i

Rest LengthはRest Poseから算出する。Graph Coloringの各制約更新は次の逐次XPBDとする。

    delta_lambda = delta_lambda_raw
    lambda_next = lambda_old + delta_lambda
    position_i += inverse_mass_i * gradient_i * delta_lambda

StretchはGraph Coloringを基本方式とする。同色内の制約は頂点を共有せず、色ごとに順次Dispatchする。色ごとにin-place更新し、次色は前色の結果を読む。

### 7.3 Shear

三角形の頂点をx0,x1,x2とし、

    u = x1 - x0
    v = x2 - x0
    u0 = rest_x1 - rest_x0
    v0 = rest_x2 - rest_x0
    L0 = length(u0) * length(v0)
    C_shear = (dot(u, v) - dot(u0, v0)) / L0

    gradient_x1 = v / L0
    gradient_x2 = u / L0
    gradient_x0 = -(u + v) / L0

とする。各三角形1制約として保持する。Rest Poseの内積を基準とするため剛体回転で変化しない。ShearはGather Jacobiで解く。三角形の向きや分割に依存する偏りを試験する。

### 7.4 Bend

共有辺をx0-x1、対向頂点をx2,x3とする。

    e = x1 - x0
    u = x2 - x0
    v = x3 - x0
    n1 = normalize(cross(e, u))
    n2 = normalize(cross(v, e))
    e_hat = normalize(e)

    s = dot(e_hat, cross(n1, n2))
    c = dot(n1, n2)
    theta = atan2(s, c)
    C_bend = wrap_angle(theta - rest_dihedral_angle)

勾配はacos(dot(n1,n2))を直接微分せず、atan2の微分と正規化ベクトルのJacobianを用いる。実装で用いる微分手順:

    Dnormalize(a) = (I - normalize(a) * transpose(normalize(a))) / length(a)

    de  = dx1 - dx0
    du  = dx2 - dx0
    dv  = dx3 - dx0

    dc1 = cross(de, u) + cross(e, du)
    dc2 = cross(dv, e) + cross(v, de)

    dn1 = Dnormalize(cross(e,u)) * dc1
    dn2 = Dnormalize(cross(v,e)) * dc2
    deh = Dnormalize(e) * de

    ds = dot(deh, cross(n1,n2))
       + dot(e_hat, cross(dn1,n2) + cross(n1,dn2))
    dc = dot(dn1,n2) + dot(n1,dn2)
    dtheta = (c * ds - s * dc) / (s*s + c*c)

各頂点座標を単位方向に摂動したときのdthetaを解析的に評価し、4頂点のgradientを構築する。実装時は有限差分近似勾配と比較し、相対誤差1e-3以下を目標に検証する。共有辺長・両面積が閾値以下の場合は制約を無効化し診断する。BendはGather Jacobiで解く。

### 7.5 Graph Coloring

制約の共有頂点を競合としてGreedy Coloringする。同色内の頂点重複をビルド時に検証する。色数、最大色サイズ、色別制約数を保存する。Stretchの色数が設定上限を超える場合はGather Jacobiへ切り替える。切替時に制約λを初期化し、同じサブステップ途中で方式を変更しない。

### 7.5.1 Animation Tracking制約

Clothの追従は頂点ごとのXPBDアタッチメント制約とする。固定頂点は逆質量0として目標位置へ直接固定する。柔軟頂点は次のベクトル制約を持つ。

    C_animation = position - animation_target
    alpha_tilde = alpha_animation / (dt_segment * dt_segment)

各軸に対してXPBD更新を行い、lambda_animationはfloat3で頂点単位にSegment Scratchへ保持する。各World共通Integration Segment開始時にリセットし、そのSegmentの反復中だけ維持する。Animation TrackingはStretch / Shear / BendのComplianceと独立して調整する。

### 7.6 Gather Jacobi

Jacobi反復では全制約が同一のposition_readを参照し、制約別λと補正寄与を別バッファへ出力する。Vertex→Constraint CSRで頂点ごとに補正を総和し、position_writeへ出力する。Dispatch完了後にPing-Pongする。

    delta_lambda_projected = delta_lambda_raw       // equality constraint
    delta_lambda_applied = omega * delta_lambda_projected
    lambda_next = lambda_old + delta_lambda_applied
    delta_position_i = inverse_mass_i
                    * gradient_i(C)
                    * delta_lambda_applied

    position_next_i = position_read_i
                    + sum_over_incident_constraints(delta_position_i)

不等式制約では先にlambda_projected = max(0, lambda_old + delta_lambda_raw)とし、delta_lambda_projected = lambda_projected - lambda_oldとする。その後omegaを掛けてλと位置の両方を更新する。

    0 < omega <= 1

これにより位置だけを緩和してλを全量更新する不整合を避ける。制約寄与数で無条件に平均化しない。omegaは安定化パラメータであり、材料Complianceと別に管理する。初期値0.5は比較試験開始値であって固定推奨値ではない。omega=1、0.75、0.5等で誤差・収束・GPU時間を比較する。

## 8. 共通接触制約

### 8.1 接触対象と生成方向

全ての物理Meshと有効Colliderのペアはデフォルトで衝突対象とする。Soft Body / Clothのself_collision_enabledは初期値trueとし、Mesh単位で無効化できる。Meshごとのcollision_enabled、self_collision_enabledと、Mesh Pair単位の明示的な除外リストを用意する。除外設定はBroadphase前に適用する。Soft Body / Cloth / Static Collider / Kinematic Colliderの有効な組み合わせをCollision Matrixで管理する。

接触対象はSoft Body境界面とCloth表面である。Soft Bodyの内部四面体面は外部接触プリミティブに登録しない。

VTはメッシュペアごとにA頂点→B三角形、B頂点→A三角形の双方を候補化する。EEは辺ペアを一度だけ正規化キーで生成する。自己衝突では同一・近接トポロジー要素を除外する。候補キーはMesh ID、Primitive ID、接触種別を含み、重複をNarrowphase前に排除する。

### 8.1.1 Collision-only Collider

初期版では衝突専用オブジェクトとしてStatic / Kinematic Triangle Meshをサポートする。Colliderは物理頂点の逆質量を0とし、時刻付きRigid Transform Sampleで移動する形状として扱う。接触点xのKinematic Collider速度は、剛体の並進速度v_linearと角速度omegaから、v_point = v_linear + cross(omega, x - transform_origin)で求める。Slerp区間ではQuaternionの解析微分からomegaを評価し、Transformの位置差分だけで頂点速度を近似しない。CCDのSwept AABBはSegment端点Transformを結ぶChord Pathを基準に作成し、Chord偏差Boundで膨張させる。Narrowphase / Conservative Advancementはtau時刻の実Rigid Transformを評価する。Contact Relative VelocityにはTOI時刻の剛体Transform Trajectory微分を使う。接触位置補正・ImpulseによってCollider自体は変形させない。

変形オブジェクトの衝突面は初期版ではSoft Body境界面とCloth Physics Mesh表面を使う。低解像度Collision Proxyを変形メッシュへ割り当て、そのProxy接触を高解像度Physics Meshへ戻す埋め込み応答は初期版の対象外とする。将来追加時はProxy頂点からPhysics MeshへのEmbeddingと、接触勾配のPhysics Vertexへの合成を定義する。

### 8.2 接触不等式XPBD

接触は非貫通条件C(x)>=0、lambda>=0とする。

    delta_lambda_raw =
        (-C(x) - alpha_tilde * lambda_old)
        / (sum_i(inverse_mass_i * dot(gradient_i, gradient_i))
           + alpha_tilde)

    lambda_projected = max(0, lambda_old + delta_lambda_raw)
    delta_lambda_projected = lambda_projected - lambda_old
    delta_lambda_applied = omega_contact * delta_lambda_projected
    lambda_next = lambda_old + delta_lambda_applied

接触位置補正はdelta_lambda_appliedを使う。0 < omega_contact <= 1とし、初期比較値は0.5 / 0.75 / 1.0。contact Complianceは初期版で0を既定値とするが、完全剛体接触が振動する場合に限り小さいComplianceを比較する。alpha_tildeは実際のdt_segmentから算出し、contact_lambdaはSegment Scratchに保持する。

### 8.3 Vertex-Triangle

接触法線nは三角形Bから頂点Aへ向くように定義する。両面衝突では前サブステップ開始時の符号付き距離を優先し、符号が不定な場合のみ相対速度を使う。

    q = beta0 * b0 + beta1 * b1 + beta2 * b2
    C_vt = dot(n, p - q) - contact_thickness

    gradient_p  = +n
    gradient_b0 = -beta0 * n
    gradient_b1 = -beta1 * n
    gradient_b2 = -beta2 * n

    effective_inverse_mass =
        wp + beta0^2 * wb0 + beta1^2 * wb1 + beta2^2 * wb2

各反復で最近接点と重心座標を再計算する。VTとEEの複数接触が同じ局所接触パッチへ集中すると過拘束になるため、同一Mesh Pair・同じ連結Surface Patch内でContact Patchを形成する。local_min_edge_lengthは、VTでは対象三角形の3辺、EEでは対象2辺のRest Edge Lengthのうち正かつ非退化な値の最小値とする。該当値がない場合はL_refを使う。Same Connected Surface PatchはPhysics Surface Triangleの共有Edge adjacencyからAsset Build時に割り当てたSurface Patch IDで判定する。EEは各辺に隣接するFaceのうちStable Primitive IDが最小のものを代表Faceとする。

初期クラスタ判定は接触点距離がcluster_radius = max(0.5 * contact_thickness, 0.02 * local_min_edge_length)以下、かつ法線内積がcos(10度)以上の場合とする。候補は(Mesh Pair, Surface Patch ID A/B, Contact Key)順に処理し、最初のContactをSeedとして、Seed距離・Seed法線条件を満たす候補だけを追加する。複数Seedに入る候補は距離が最小のSeedへ割り当て、同距離はSeed Contact Key順で決める。連鎖的な近傍判定でPatch全体が無制限に拡大しないよう、候補同士の推移的連結だけで統合しない。

v1初期値は1 Patchあたり最大4 Contact。Contact Key順でtie-breakする決定的なManifold Reductionを行い、(1)最深点、(2)最深点から最遠点、(3)既存2点が作る線分から最遠点、(4)既存3点が作る三角形から最大面積となる点を順次残す。候補が線状・点状の場合は幾何epsilonで同一点と判定される重複を除き、独立したSupport Point数だけを残す。

縮約品質は同じ入力予測状態・同じSolver反復回数・同じIntegration Segment時間・同じContact Complianceで、全Candidateを使う未縮約Reference Manifoldと比較する。初期Engineering Threshold:
- 縮約Manifoldの反復後最大残存貫通量が未縮約Referenceより増える量 <= 0.05 * max(contact_thickness, epsilon_length)。
- 全Candidate法線を平均法線へ整列し、各Candidate法線から最も近いRetained法線までの最大角度 <= 5度。
- CandidateとRetainedのContact Positionを平均法線に直交する2D基底へ射影したSupport Hull面積の保持率 >= 0.90。Retainedが退化して面積0の場合はCandidateが面積0のケース以外で失敗とする。
- 線状PatchではSupport Hull長さ保持率 >= 0.90。点状Patchではepsilon内の重複を除いたSupport Point数が一致すること。

いずれかを満たさない場合は、Contact間の相互距離が最大のPairを2つのSplit Seedとし、各Candidateを近いSeedへ割り当てる。同距離はSeed Contact Key順で決定する。Child Patchへ同じReduction / Quality Testを再帰適用する。分割深度上限は8。上限到達後も品質条件を満たさない場合はContact Manifold Reduction FailureとしてTickをRejectする。Patch総数 / Contact数はCapacity Check対象とする。上記閾値は初期Engineering Thresholdであり、Run Manifestへ記録してFixture / Hardware試験で校正する。contact_lambdaは同一Integration Segmentの同一Contact Keyで反復中保持する。Segment開始時に0へ初期化し、Contact KeyまたはPatch Generationが変わった場合も初期化する。

### 8.4 Edge-Edge

    qA = (1 - s) * a0 + s * a1
    qB = (1 - t) * b0 + t * b1
    C_ee = dot(n, qA - qB) - contact_thickness

    gradient_a0 = +(1 - s) * n
    gradient_a1 = +s * n
    gradient_b0 = -(1 - t) * n
    gradient_b1 = -t * n

有効逆質量は4頂点のinverse_mass * gradient_coefficient^2の総和とする。平行・退化辺では区間最近接点の安定分岐を用いる。最近接距離が幾何epsilon以下のときは§8.6の法線決定規則を適用し、法線を一意に決定できない候補を有限値保証の名目で任意方向へ補正しない。

### 8.5 摩擦・反発 / Coupled Contact Impulse

位置XPBD接触と速度Impulseを分離する。Contact Position補正で生じた変位をVelocity / Restitutionへ混ぜない。各Integration Segmentでは次の順で処理する。

1. Segment開始状態からDynamic Velocity / Positionを予測する。
2. ClothのStretch / Shear / Bend / Animation Tracking等の非接触内部Constraintを解く。
3. 内部Constraint後の状態からDynamic VertexのContact前Velocityを一度だけ再構築し、immutableなv_pre_contact Snapshotへ保存する。
4. Contact Position XPBDを解く。Contact補正はPositionだけに反映し、Snapshotを変更しない。
5. 全ContactのVelocity ImpulseをCoupled Contact Solverで解く。Solverはv_pre_contactからWorking Velocityを作り、Contact Contributionを反復Gatherする。

Contact位置補正後にPosition差分からVelocityを再構築してはならない。通常SegmentのContact前Velocityは、

    v_pre_contact_i = (x_internal_solved_i - x_segment_start_i) / dt_segment

TOI prefixは分母をtauとする。Fixed / Kinematic VertexはTrajectory微分を維持する。

#### 有効逆質量と非応答Contact

Contact iの頂点係数をc_ik、参加頂点の逆質量をw_kとし、

    W_ii = sum_k(w_k * c_ik^2)

を有効逆質量とする。VT / EEのNormal / Tangent方向は同じScalar係数を使うため、等方的Particle Massでは同じ対角有効逆質量になる。

W_iiが有限でない場合はNonFinite ErrorとしてTickを拒否する。W_ii <= epsilon_inverse_massの場合、そのContactはNON_RESPONSIVE_CONTACTとしてImpulse対象から除外する。Contact Position XPBDも同じ事前判定でSkipし、Complianceにより0/0や無意味なLambdaを作らない。Static-Static、Kinematic-Kinematic、または接触係数が掛かるDynamic Vertexを持たないContactは正常な非応答Contactであり、CCD応答 / Position / Impulseを生成しない。Diagnostic Countへ記録する。epsilon_inverse_mass = 1e-12 / mass_reference_worldとする。mass_reference_worldはWorld内すべての参加Dynamic Physics VertexのMedian Positive Mass [kg]であり、epsilon_inverse_massの単位は[1/kg]。mass_reference_worldが存在しない場合は前述の通り全Contactを非応答とする。Dynamic Massが存在しないWorldは全Contactを非応答とする。

#### Contact Relative Velocity / Normal / Friction

Contact点A / Bの相対速度はVT重心係数 / EE最近接係数で補間する。

    v_A = sum_i(beta_i * v_contact_pre_i)
    v_B = sum_j(gamma_j * v_contact_pre_j)
    v_rel = v_A - v_B
    v_n = dot(v_rel, n)
    v_t = v_rel - v_n * n

法線nはBからAへ向き、接近時はv_n < 0。Kinematic Collider接触点速度は§8.1.1のv_linear + omega cross radiusを使う。

#### Coupled Projected Jacobi Impulse Solver

Contact間でDynamic Vertexを共有する場合、各Contactを同じv_pre_contactから独立計算して一度Gatherする方式では相互干渉を解けない。v1は全ContactをCanonical Contact Key順に整列し、Global Projected Jacobiで速度Impulseを反復する。Contact Patchの最大4点Reduction後も、異なるPatch間で共有Vertexを持つContactを同一World Solverへ含め、結合効果を反復Gatherで伝播する。

Contact iごとにAccumulated Normal Impulse j_n_i >= 0とWorld Tangent Impulse Vector j_t_iを保持する。初期値は0。Contact XPBDの最終Normal Lambdaをlambda_n_iとすると、Support Friction Impulseはj_support_i=max(0,lambda_n_i)/dt_contactとする。dt_contactは通常Segmentでdt_segment、TOI prefixでtau。XPBD Lambdaの単位はkg*m、Lambda/dtの単位はN*sである。

Iteration開始時のWorking Velocityは、v_pre_contactへ前Iterationまでに蓄積した全Contact ImpulseをVertex CSRでGatherしたものとする。Contact iの符号付き頂点係数をc_ik、法線をn_iとすると、

    v_work_k = v_pre_contact_k
             + inverse_mass_k
               * sum_i(c_ik * (j_n_i * n_i + j_t_i))

とする。各Contact iは同じIteration開始Snapshotからv_n_i_current / v_t_i_currentを評価し、

    target_normal_velocity = -restitution * min(v_n_i_pre, 0)
    delta_j_n = -omega_impulse
                * (v_n_i_current - target_normal_velocity) / W_ii
    j_n_i_new = max(0, j_n_i + delta_j_n)

    delta_j_t = -omega_impulse * v_t_i_current / W_ii
    j_t_candidate = j_t_i + delta_j_t
    j_t_i_new = project_to_disk(
        j_t_candidate,
        friction_i * (j_support_i + j_n_i_new))

としてAccumulated Impulseを更新する。project_to_diskはNormalに直交する2D Tangent Plane上で実行する。Iteration内の全Contactが同一Velocity Snapshotを読むJacobi更新とし、変更量だけを次でVertex単位にGatherする。

    delta_v_k = inverse_mass_k
                 * sum_i(c_ik * (delta_j_n_i * n_i + delta_j_t_i))
    v_work_next_k = v_work_k + delta_v_k

Contact Key順のContribution Reductionにより同一GPU / Driver / Buildで同じ順序を維持する。Contact iのc_ikはVT / EEのNormal / Tangent Gradient係数と同じ符号付き係数を使う。

初期値はcontact_impulse_iterations=8、omega_impulse=0.5、最大許可Iterationは16。各Iteration後、Normal / TangentのProjected Fixed-Point Residualを次で評価する。

    r_n = abs(j_n - max(0, j_n - (v_n_current - target_normal_velocity) / W_ii))
    r_t = length(j_t - project_to_disk(j_t - v_t_current / W_ii,
                                       friction * (j_support + j_n)))
    J_ref = mass_reference_contact * (length_reference_contact / dt_contact)
    residual = max(r_n, r_t) / J_ref

mass_reference_contact / length_reference_contactはContact Solverへ参加するDynamic Meshの正値Mass / Rest Edge Length中央値から決める。contact_impulse_residual_tolerance以下へ収束した場合だけSolverをResolvedとする。初期値は1e-4。16 Iterationでも未収束ならCONTACT_IMPULSE_UNRESOLVEDとしてLogical TickをCommitしない。DescriptorでIteration数を1～16の範囲に設定でき、初期値8、Relaxationは0より大きく1以下、初期値0.5とする。Residual定義と正規化はNumerical Acceptance Profileを正とする。

RestitutionはSegment開始時に保存したv_n_i_preから目標分離速度を設定し、Contact Position correction由来のVelocityを入力にしない。v_n_i_pre >= 0では反発Impulseを生成しない。Normal accumulated impulseは常に0以上。摩擦ImpulseはSupport / Restitution Normal Impulse合計のCoulomb Disk内に制限する。摩擦係数は0以上、反発係数は0～1。Initial Restitutionは0。

#### TOI / Discrete Contact Event

Discrete ContactとTOI Contactで同じCoupled Solverを使う。TOI Eventは同時TOI Group内でCanonical Contact Keyにより一意化し、impact_resolvedにより重複適用を防止する。Impact後Velocityから残Segmentを開始し、同一TOI Segment内でVelocity Reconstructionを実行しない。全World ContactのSolverが完了してからv_finalをPersistent Stateへ書き戻す。

### 8.6 退化接触の法線決定

長さepsilonはmax(1e-7 * L_ref, 1e-9 m)、面積epsilonはepsilon_length^2を初期値とする。候補ごとに以下の優先順位で法線を選ぶ。

1. 同一Contact Keyで前SubstepのPersistent Contact Cacheに存在し、有限かつ正規化済みの前回法線。
2. 現在形状から算出した非退化幾何法線。VTでは三角形法線、EEでは2辺の外積方向を使う。両面衝突の符号は前Substepの符号付き分離方向で整合させる。
3. Rest Poseの面法線 / Hinge Binormalを現在の局所変形基底で写した参照方向。
4. 相対速度ベクトル。速度がepsilon以下の場合は利用しない。

候補法線はContactの正規向き（VTは三角形Bから頂点A、EEはCanonical PairのBからA）へ符号を統一する。候補間で向きが矛盾する場合は前回法線との内積で連続性を優先する。上記を適用しても法線が定まらない、または退化幾何によりNarrowphaseの距離・勾配が有限に計算できない場合はCONTACT_GEOMETRY_UNRESOLVEDとしてLogical TickをCommitしない。Stable IDから任意軸を生成して押し出す処理は禁止する。

Contact Cacheに法線があってもMesh Topology / Contact Key / Patch Generationが変化した場合は流用しない。無効Cacheを検出した場合は再評価し、失敗すればUnresolvedとする。

## 9. Broadphase Spatial Hash

### 9.1 多段階セル

単一セル幅では大型プリミティブが過剰なセルへ登録されるため、階層セル幅を用いる。

    cell_width(level) = base_cell_width * 2^level

base_cell_widthは、Point Primitiveを除いたCollision Primitive（Edge / Triangle）の正のAABB最大辺長中央値、最大Contact Thicknessの2倍、cell_width_floorの最大値とする。cell_width_floor=max(1e-6 m, 1e-3 * L_ref)、L_refはMeshのmedian positive rest-edge length。v1初期値はmax_cells_per_primitive=64、max_hash_levels=24。cell_width(level)=base_cell_width*2^levelとし、各Primitiveは接触厚みで膨張させたAABBが64セル以下となる最も細かいlevelへ登録する。最大levelでも64セルを超えるPrimitiveはLarge Primitiveとして別Bucketへ登録する。Large Primitive BucketのBVHは30-bit Morton Code（空間各軸10-bit）とStable Primitive IDの辞書順KeyでsortするGPU Linear BVHとし、Karras方式でBinary Nodeを構築する。Morton量子化BoundsはLarge Primitive全AABBのUnion Boundsとし、ゼロExtent軸は量子化値0に固定する。NodeはAABB min/max、left/right child、parent、escape_index、leaf primitive IDを持つ。Stable Primitive IDをTie-breakに含め、Morton Codeが同一でもLeaf順・Node構築順を一意にする。TraversalはStackless depth-firstとし、AABB不一致またはLeaf処理後はescape_indexへ進む。Large Primitive数N=0の場合はBVH Build / QueryをSkipし、Node / Pair Capacityを0として通常Hash候補だけを使う。N=1の場合は単一LeafをRootとして作成し、Karras Internal Node BuildをSkipする。N>=2の場合のみKarras方式を実行する。Node数はN=0で0、N>=1で2*N-1として事前Count / Capacity Checkし、uint32 / ByteWidth上限を検査する。escape_indexの終端SentinelはNode Countとし、Root Subtreeの処理完了後にTraversalを終了する。Regular PrimitiveはLarge Primitive BucketのBVHを自身の膨張AABBでQueryしてRegular-Large Pairを生成する。各Large Primitiveも同じBVHをQueryするが、LeafのStable Primitive IDが自身より大きい場合だけLarge-Large Pairを出力し、自己Pair・逆順重複を抑止する。Canonical Pair Keyで重複を除く。これにより専用StackとStack Overflow経路を持たず、BVH Node / Pair / Dedup Capacity不足だけをBlock / Retry対象とする。Hash / BVHの全候補はCanonical Pair KeyでUnion / DedupしてからNarrowphaseへ渡し、総当たりFallbackは禁止する。

各query Primitiveは自分のlevelから最大levelまでを検索し、各levelでquery AABBと重なる全セルを列挙して整数セル座標を比較する。Primitiveは自身のlevelより細かいlevelへ検索を戻らない。異なるlevelのペアは小さいPrimitive側が大きいPrimitiveのlevelを検索することで検出する。VTは頂点AABB対三角形AABB、EEは辺AABB対辺AABBを検索する。Large Primitive Bucketは通常PrimitiveとLarge Primitiveの双方からAABB Queryできる。各Large Primitiveは自身のAABBでBVH全体をQueryし、Stable Primitive IDの大小で自己Pairを一度だけ出力する。Large-Large専用の全件比較 / 総当たり / Tree Self-Traversalは行わない。候補は通常Hash由来Pairと同一のCanonical Pair Keyで重複排除する。BVHのMorton / Stable ID複合Keyは(key_hi=morton_code, key_lo=stable_primitive_id)の2つのuint32値として比較し、64-bit整数演算をHLSLへ要求しない。

### 9.2 接触厚みとセル範囲

各Primitive AABBは自身の接触半径分だけ膨張させる。ペアでは両側の膨張領域が重なることで候補化する。頂点を一セルに登録するだけで隣接セルを検索しない方式は禁止する。query AABBが重なるセルを全て検索する。

### 9.3 登録・候補生成

GPU処理順:

1. Primitiveごとの登録level・セル範囲・登録数を計算。
2. Countで総登録数を算出。
3. Hierarchical Exclusive Prefix Sum。
4. Capacity Check。
5. FillでCellRecordを出力。
6. Hash、level、整数セル座標でSort / Group。
7. 同一セルグループ内で候補を列挙。
8. Candidate Pairを正規化し、重複排除。
9. AABB再検査後にNarrowphaseへ渡す。

Hash値一致だけで同一セルと判定しない。Hash、level、元の整数セル座標を比較する。Prefix Sumと登録数の加算はuint上限を超えないよう各段でoverflow検査を行う。上限超過時はCountOverflowとしてFillを停止する。1回のバッファへ収まらない場合は、上限以下のBatchへ分割する。D3D11 BufferのByteWidthおよびDispatch group countのAPI上限もCPU側で事前検査する。

### 9.4 接触候補の完全性

VTは頂点AABBと三角形AABB、EEは辺AABB同士のセル重複から候補化する。AABB膨張量・level検索範囲・pair正規化の整合性を単体試験する。Broadphase候補がNarrowphaseで接触した全ペアを包含することを小規模全探索参照実装と比較して検証する。

### 9.5 Overflow

CellRecord、Candidate、Contact、Contributionの全可変長バッファにcapacity、required_count、overflow flagを持たせる。Count/Scan段階でcapacity超過を検出した場合、Fill以降とシミュレーション状態のCommitを行わない。

Simulation Managerは対象インスタンスをBlockedByCapacity状態へ移し、GPUイベント完了後に診断情報を非同期回収する。各GPU Passはsimulation_statusを先に確認し、異常フラグが立ったStepの後続PassとState Commitを実行しない。必要容量へ拡張し、確定済み状態のスナップショットから同じlogical stepを初期状態として再実行する。Blocked中に当該インスタンスの時間を進めない。他インスタンスは継続可能とする。再試行上限を超えた場合はSimulation Errorにし、候補を切り捨てない。

## 10. Narrowphase・自己衝突

VTとEEの最近接距離を正確に計算し、接触厚み以下だけを接触として生成する。候補段階の接触と確定接触数を別々に計測する。

### 10.1 初期貫通修正

初期化時に一度Broadphase / Narrowphaseを実行し、初期接触を抽出する。位置XPBD接触を設定上限回数まで解き、頂点ごとの総修正量と残存貫通を計測する。各初期接触の制約評価値は、1反復あたり`C_effective = max(C, -0.1 * local_min_edge_length)`として過大な一括補正要求を制限する。この有効制約値をλ更新にも用い、位置だけを後からClampしてλと状態が乖離しないようにする。

Meshごとに`max_initial_repair_distance`を設定し、各Physics Vertexについて初期化開始位置からの修正移動量累積をこの値以下に制限する。Gatherで得た仮の頂点補正について、各頂点ごとに接触Constraint寄与の絶対変位量を総和し、その値と残り修正予算からscaleを計算する。各接触Constraintの全参加頂点scaleの最小値をそのConstraintの共通scaleとする。修正量累積は反復ごとの寄与絶対値の総和とし、異なる反復間で符号相殺させない。その共通scaleをdelta_lambdaと全頂点補正に適用してからλと位置を確定する。これにより双方の補正とλを整合させたまま最大修正量を守る。修正予算を使い切っても残存貫通が許容値を超える場合は警告またはAsset Load Errorとし、初期化状態をCommitしない。

自己衝突除外はPhysics MeshのStable Vertex / Primitive IDからAsset Build時に生成し、Runtime Broadphase前に適用する。v1の既定除外は次の通り。

- 同一Primitive。
- 同一Mesh内で少なくとも1つのPhysics Vertexを共有するPrimitive Pair（共有Vertex / Edge / Faceを含む、topological 1-ring）。
- Soft Bodyでは同一Tetに属する境界Face Pair、および共有Physics Vertexを持つ境界Face Pair。
- Mesh Pair除外リストで明示された組み合わせ。

追加の2-ring / 3-ring除外は任意のScenario設定として保持できるが、既定値は0追加ringとする。これにより折り畳み部の接触を広範囲に抑止しない。除外Pair数・対象距離・残存接触を可視化し、Self CollisionのFold / Edge contact fixtureで偽陰性を検証する。

## 11. Continuous Collision Detection

### 11.1 適用条件

相対移動距離が接触厚みの一定比率を超える候補をCCD対象とする。

    length(relative_displacement) > eta * contact_thickness

etaの初期値は0.5。これは候補の選別値であり、CCD自体の正しさを保証する値ではない。薄い布・高速移動試験で見逃し率を測定し、必要なら対象範囲を広げる。離散終端位置が非接触でもSwept AABBがNarrowphase候補を作る場合、相対移動距離が小さいことだけを理由に候補から除外しない。初期接触・persistent contact・初期貫通はこの速度選別の対象外とする。

### 11.2 Swept AABB

各頂点・辺・三角形の現在Integration Segment開始位置と予測位置を含むSwept AABBを生成する。接触厚みで膨張させ、Broadphase候補を絞る。

### 11.3 Conservative Advancement

Dynamic Physics VertexのCCD pathはSemi-implicit Eulerが生成したSegment開始 / 終端状態を結ぶ離散線形軌道とする。各候補のDynamic path速度上限はlength(path_velocity)、Kinematic Vertex / Colliderの速度上限はkinematic_path_speed_boundとする。 VT / EEのv_bound式に使うpath_speed_bound(primitive)は、Dynamic primitiveではlength(path_velocity)、Kinematic primitiveではAnimation / Rigid Transform Evaluatorが返す保守的なkinematic_path_speed_boundである。Kinematic Vertex / ColliderのNarrowphase / Conservative Advancementは各tau時刻でAnimation / Rigid Transform Trajectoryを直接評価した幾何位置を使う。KinematicのEndpoint ChordはSwept AABBの基準形状として使い、全区間の保守的Chord偏差BoundでAABBを膨張させる。CAの距離d(tau)は実Trajectory位置から計算し、速度上限v_boundはDynamic path速度上限とKinematic Trajectoryの区間速度Boundを合成する。秒単位の時刻tauにおける距離をd(tau)、実接触厚みをr、相対速度上限をv_boundとする。

VT:

    v_bound = path_speed_bound(vertex)
             + max(path_speed_bound(tri0), path_speed_bound(tri1), path_speed_bound(tri2))

EE:

    v_bound = max(path_speed_bound(edgeA0), path_speed_bound(edgeA1))
             + max(path_speed_bound(edgeB0), path_speed_bound(edgeB1))

保守的な進行量:

    delta_tau = safety * (d(tau) - r) / max(v_bound, epsilon)
    tau_next = tau + delta_tau
    safety = 0.8  // 初期値

d(tau)<=r+distance_toleranceでTOIを確定する。tau_next>dt_segmentかつ終端距離がrより大きければ当該Segmentでは接触なし。Conservative Advancementは最大32反復を初期上限とする。`tau_tolerance = max(1e-7 s, 1e-5 * dt_segment)`、`distance_tolerance = max(1e-6 * L_ref, 1e-4 * contact_thickness)`を初期値とし、L_refは当該Physics Meshのmedian positive rest-edge lengthとする。`safety=0.8`は維持する。停止判定は距離条件または保守的区間No-Hit条件で行い、tau_tolerance到達だけでNo-Hitにしない。進行不能または32反復上限に到達した場合、未解決CCDとして診断し、候補を黙って破棄しない。

### 11.4 TOI処理

現在のWorld Integration Segment内の最小TOIをGPU Reductionで求める。TOI時刻は現在のIntegration Segment開始からの秒単位tauで扱い、0 <= tau <= dt_segmentとする。Dynamic Vertexの離散pathはpath_velocity=(position_predicted-position_segment_start)/dt_segment、x_path(tau)=position_segment_start+tau*path_velocityとする。Kinematic Vertex / Colliderの距離計算とTOI位置はtau時刻のAnimation / Rigid Transform Trajectoryを直接評価し、Endpoint ChordはSwept AABBのBroadphase基準形状としてだけ使う。v_boundはDynamic path速度とKinematic Trajectoryの区間速度上限の和を使う。Conservative Advancementの進行量は次とする。

    delta_tau = safety * (d(tau) - contact_radius) / max(v_bound, epsilon)
    tau_next = tau + delta_tau

TOIが確定したら、検出に使った離散軌道と同一の位置・速度をTOI prefix状態として採用する。§6.6のSemi-implicit EulerはSegmentの予測終端Velocityを使ってPositionを更新するため、Dynamic VertexのSegment内CCD pathは次で定義する。

    path_velocity = (position_predicted - position_segment_start) / dt_segment
    x_path(tau) = position_segment_start + tau * path_velocity
    v_path(tau) = path_velocity

このpathはv1の離散積分方式に対応するSegment内の衝突判定軌道である。TOI prefixをdt=tauで再積分してはいけない。そうするとDamping / Forceの更新結果が変化し、Conservative Advancementが検出した軌道と異なる位置へ到達する。Positive-Time TOI時は予測Destinationを破棄し、上記x_path(tau) / v_path(tau)から全Dynamic Vertexのprefix状態を構成する。Fixed / Kinematic VertexのTOI位置はtau時刻のAnimation Target Trajectoryを直接評価し、接触相対Velocityには同時刻のTrajectory微分を使う。Endpoint ChordはSwept AABBの基準だけに使い、kinematic_path_deviation_boundでAABBを保守的に膨張させる。Conservative Advancementは実Trajectory位置の距離d(tau)を使い、v_boundにはAnimation / Skinning Trajectoryの区間速度上限を加える。

prefix状態の構成後、tauをXPBD Integration Segment時間としてCloth Stretch / Shear / Bend / Animation Tracking等の非接触内部Constraintを先に解く。Soft Bodyはprefix軌道状態を維持する。Contact Position Constraintを解く前にDynamic VertexのContact前Velocityを(x_internal_solved - x_segment_start) / tauで再構築して保存する。次にTOI Contact Position Constraintを解くが、Contact位置補正後にVelocityを再構築しない。保存済みContact前VelocityをCoupled Projected Jacobiへ渡し、ResidualがAcceptance範囲へ収束した場合だけImpactを一度Commitする。W_ii <= epsilon_inverse_massの非応答ContactはPosition / ImpulseをSkipする。

Kinematic Animation CurveとEndpoint Chordの差は、有限個のサンプル最大値で保証しない。Animation Evaluatorは各Kinematic Geometry Primitive / VertexについてSegment内全時刻のChord偏差上限kinematic_path_deviation_boundと、CA用の区間速度上限kinematic_path_speed_boundを返す。CUBICSPLINEのTranslationはHermite曲線をBezier Control Hullへ変換し、Chordとの差分Bezier Control Pointの最大Normを保守的上限とする。Quaternion / Bone Hierarchy / Skinningを含む変換は区間演算または保守的な導関数Boundを階層合成・Skin Weight合成へ伝播する。固定軸Slerpで動く剛体Colliderの回転Chord偏差は、Collider原点からの最大距離r_maxと角度thetaからsagitta_bound = r_max * (1 - cos(theta / 2))で求める（theta <= piとなるまで分割）。Translation Curveの偏差Boundと回転BoundはSwept AABBへ加算膨張する。Kinematic Trajectoryの区間速度上限はTranslation / Skinningの導関数Boundと、剛体Colliderではmax(|v_linear|) + max(|omega|) * r_maxを使う。

kinematic_path_deviation_boundがdistance_tolerance以下になるまで最大分割深度8で決定的にSegment細分化する。深度8でもBoundが許容値を超える、またはDeviation / Speed Evaluatorが保守的Boundを返せない場合はCCDUnresolvedとしてTickをCommitしない。Conservative AdvancementはChord上の距離ではなく実Trajectory位置で評価した距離d(tau)を使い、v_boundへkinematic_path_speed_boundを含める。TOI prefix位置にはtau時刻のAnimation Target / Rigid Transform Trajectoryを直接評価し、Contact Relative Velocityには同時刻のAnimation Target Trajectory微分、Kinematic Colliderではv_linear + omega cross radiusを使う。実サンプル点の偏差は診断・校正用に計測するが、合否は保守的Boundで決める。Run Manifestへ最大Deviation Bound / Speed Bound・分割深度・未解決数を記録する。

残時間dt_segment-tauはImpact後のPosition / Velocityから新しいWorld Integration Segmentとして進める。Impact後Velocityは残Segment開始Velocityとして固定し、同じTOI Segmentの末尾で再Velocity Reconstructionしない。新SegmentではSoft Body内部力 / Animation Tracking力 / External Forceを新しいSegment開始状態で評価し、Event Cursorから未消費Eventだけを適用する。必要ならBroadphaseを再構築する。TOIを含むStepではLogical Tick全体の始点・終点差分からVelocityを再構築してImpact結果を上書きせず、最後のSegmentで得たVelocityを保持する。

1 World共通Substep内のPositive-Time TOI分割上限は初期値2回。TOIを同時Eventとしてまとめる許容時間は`max(1e-7 s, 1e-5 * dt_segment)`とし、Event内のImpulseはContact Key順に一度だけ適用する。TOI以前のprefixは§11.4のx_path(tau) / v_path(tau)からWorld全体で構成する。Positive-Time TOI時はEndpoint Contactを確定せず、同じ離散path上のTOI位置でtauをXPBD時間として内部Constraint / Contact Constraintを解き、Impactを適用する。残時間はImpact後の状態から全参加Solver共通の新Segmentとして進める。Solverごとに異なるtauを使う・Contact相手の状態を旧位置へ固定する処理は禁止する。上限を超えた場合は残時間を進めずCCDIterationLimitとしてLogical Tickを未Commitにし、World共通Substep数またはCCD budgetを増やして同じTick開始Snapshotから再試行する。再試行上限はScenario設定に従い、上限到達時はWorldをBlockedByCCD / Failedへ遷移させる。CCDの解決不能を貫通許容として扱わない。

## 12. Render Vertex Binding

### 12.1 四面体内部

四面体内の描画頂点は4頂点の重心座標で位置を補間する。

    x_render = beta0*x0 + beta1*x1 + beta2*x2 + beta3*x3

Rest Poseでの埋め込み誤差を計測し、最大誤差が基準を超える頂点はBuild Errorまたは明示的な警告とする。

### 12.2 境界外頂点

境界三角形への最近接点q、重心座標、Rest Pose offsetを保存する。rest_offsetはRest Poseのモデル空間ベクトルとして保存し、Rest Pose境界三角形から局所基底T0=[e1_0,e2_0,n0]を作る。変形後は対応三角形からT1=[e1_1,e2_1,n1]を作る。

    rest_offset = x_render_rest - q_rest
    x_render = q_deformed + T1 * inverse(T0) * rest_offset

対応三角形が退化・反転した場合は直前有効基底を使うのではなく、その頂点をBinding Errorとしてマークし、診断する。頂点IDの再検索は毎フレーム行わない。Render Feedbackは位置、法線、接線のみを更新し、UV、カラー、マテリアル割当、描画インデックス、その他の変形対象外属性は元Render Meshから維持する。

### 12.3 Cloth Binding

同一トポロジーは直接index対応。異なるトポロジーはRest Pose表面の三角形・重心座標を保存する。距離誤差が閾値超過した頂点を自動的に最近傍頂点へ置き換えない。

### 12.4 法線・接線

Render Mesh三角形ごとに変形後の面法線を求め、Render Vertex単位でGatherして正規化する。Physics VertexのWelding情報をそのまま法線集約に使わない。UV seam / hard edgeは描画属性単位で分離して扱う。

接線は変形後位置とUV微分から再計算し、法線に直交化する。UV退化三角形は安定した直交ベクトルへフォールバックし、診断数を記録する。

## 13. 物理特性・ペイント

材料はMesh単位の基準値、頂点ウェイトは0～1のチャンネル値として保持する。Soft Bodyはmu/K/rho、減衰、Animation Tracking stiffness/dampingを管理する。ClothはStretch/Shear/Bend compliance、面密度、減衰、Animation Trackingを管理する。

Render MeshからPhysics Meshへ属性を転送する場合、Render Vertex Bindingを逆方向に単純平均しない。Soft Body境界Physics VertexはRender Mesh表面三角形から重心座標で補間する。Soft Body内部頂点は境界Physics Vertexのチャンネル値をDirichlet境界条件とするグラフLaplacian調和拡張で伝播する。Cloth頂点は直接対応または表面三角形補間を使う。転送誤差、残差、重みの分散を記録し、曖昧な転送は未確定とする。Physics Mesh再生成時は元Render Meshの安定IDを優先し、トポロジー変更で対応が一意に決まらない場合は自動適用せず警告する。

Paint / Smooth / Average / Eraseを提供する。Paintはtarget_valueへstrengthに応じて補間する。各操作はブラシ半径、strength、target_value、channel、対象Physics Meshを入力とする。Heatmapでチャンネル値を可視化する。各頂点チャンネル値は0～1とし、0は当該チャンネルの効果なし、1はMesh基準値そのままとする。初期版のチャンネルは次の通り。

| チャンネル | Soft Body | Cloth |
|---|---|---|
| Stretch | 無効 | 有効 |
| Shear | 無効 | 有効 |
| Bend | 無効 | 有効 |
| Volume | 有効（volume_retention_scale） | 無効 |
| Shape Recovery | 有効（shape_recovery_scale） | 無効 |
| Damping | 有効 | 有効 |
| Animation Tracking | 有効 | 有効 |

頂点ウェイトのソルバー反映規則は次の通りとする。

- Cloth Stretch: 辺両端ウェイトの平均をw_stretchとし、`alpha_effective = alpha_base / max(w_stretch, epsilon)`。w_stretchがepsilon以下ならその制約を無効化する。
- Cloth Shear: 三角形3頂点ウェイトの平均をw_shearとして同様にComplianceへ反映する。
- Cloth Bend: Hingeを構成する4頂点ウェイトの平均をw_bendとして同様に反映する。
- Soft Body Shape Recovery / Volume: 四面体4頂点の平均ウェイトをそれぞれshape_recovery_scale / volume_retention_scaleへ乗算する。
- Damping: 各頂点の減衰率を`damping_rate_i = base_damping_rate * damping_weight_i`とする。
- Animation Tracking: 頂点のtracking_weightを`base_tracking_weight * animation_tracking_weight_i`とする。

物理方式で無効なチャンネルは非表示または編集不可とする。ブラシ半径はモデル空間長さで扱い、画面ピクセル入力はカメラからモデル空間へ変換する。ペイントデータはJSONまたはアセットサイドカーファイルで保存する。

## 14. 時間管理・JSON・再現性

### 14.1 固定時間刻み

描画Delta Timeを直接ソルバーへ渡さない。Simulation ManagerがAccumulatorでFixed Delta TimeのLogical Tick要求を管理し、Runtime SchedulerがGPUへのSubmissionを管理する。既存fixed_step_controllerはCPU reference / test環境の同期Callback用として保持し、GPU Production RuntimeのCommit判定へそのまま流用しない。GPUへのSubmission受付とGPU Tick Commitは別の状態である。

フレームあたりの最大Step / GPU in-flight上限に達した場合は遅延を記録し、Accumulatorへ時間を保持する。性能維持のために時間を黙って飛ばさない。GPU完了StatusはEvent Queryと非同期Status Readbackで確認する。Event QueryはCommand完了を示し、Tickが成功したことはGPU Status Bufferで判定する。

### 14.2 Step / Pause

single_step要求はPause / Stop中にFixed Delta Timeを1回だけ要求する。GPU実行は非同期であり、APIがSubmissionを受け付けた時点で完了扱いにしない。複数Step要求は受理順にStep Request Queueへ保持する。Pause / Stop中はStep Request Queueをwall-clock Accumulatorより優先し、1要求につき1 Logical TickだけをSubmissionする。Accumulatorにwhole tickやfractional remainderが残っていてもStep Requestを先に処理し、そのAccumulatorは変更しない。GPU容量エラー / CCD Unresolved / 入力不足 / Render Backpressure中はStep Requestを保留し、解消不能ならFailureを返す。

Pauseはwall-clock時間の追加とAccumulator由来の新規Reservationを止め、既にGPUへSubmission済みのTickは安全に完了させる。未割当Accumulatorは保持する。StopはPaused状態へ移行し、未Reservation Step Requestを取消してcancelled_requested_timeへ記録する。Stopも未割当wall-clock Accumulatorと物理Stateは保持する。ResetはInitial Stateへ戻し、Accumulator（whole tick / fractional remainder）、Tick Reservation / Sequence、Step Request Queue、Solver Lambda、Contact / CCD Cache、Requested / Cancelled / Committed / Observed Timeline Counters、診断カウンタを初期化する。in-flight GPU commandのResource退役は同期waitを通常Frameへ入れず、Query / Deferred Releaseで処理する。

### 14.3 Test Environment操作

Test Environmentは次の操作を提供する。

- Play: 現在状態から時間進行を再開し、保持しているwall-clock Accumulatorを処理する。
- Pause: wall-clock時間の追加とAccumulator由来の新規Reservationを停止し、状態とAccumulatorを保持する。
- Stop: Pause状態へ移行し、未Reservation Step Requestを取消してcancelled_requested_timeへ記録する。物理Stateとwall-clock Accumulatorは保持する。
- Reset: 保存済みInitial Stateへ復帰し、Accumulator全体、Step Queue、Timeline / Sequence / Solver Cache / 診断カウンタを初期化する。
- Step: Pause / Stop中に1 Fixed Tickだけを要求する。複数要求は要求順に処理し、wall-clock Accumulatorより優先する。既存Accumulatorの時間を消費しない。
- Time Scale: 実時間DeltaをAccumulatorへ加える倍率。固定物理時間刻み自体は変更しない。
- Fixed Delta Time: シミュレーションの基本時間刻み。
- Gravity: 重力ベクトルを変更する。
- Impulse / Force: kind（instant impulse / sustained force）、位置（m）、方向（正規化ベクトル）、強度、正の半径（m）、継続時間（s）、World Simulation Time原点からの絶対開始時刻start_timeを指定する。Forceのend_timeはstart_time + durationから導出し、Descriptorで二重指定しない。Impulseはevent_timeの絶対時刻を持ち、duration=0としてWorld共通Segment境界で一度だけ適用する。Impulseの強度単位はN*s、ForceはNとし、Forceは指定時間と各Integration Segmentの重なり時間に応じて積分する。空間分配は§7.1.1の質量測度付きFalloffで行い、対象World / Model / Mesh範囲をDescriptorで明示する。
- Animation: Clip、再生時刻、速度、Blend、Discontinuity Policyを指定する。

Time Scale変更はAccumulatorへの時間供給量にのみ適用する。物理時間刻み変更は別設定とし、変更時はXPBD lambdaをリセットして新しい時間刻みで開始する。World内のSubstep数はLogical Tick開始時に一度決定し、Soft Body / Cloth / Contactが同一Substep時刻・同一dtを使う。§6.8で定義したWorld共通Substepをv1の唯一の時間分割方式とする。

#### 14.3.1 Force / Impulseの適用契約

Force / Impulse Descriptorは対象World / Model / Mesh filter、ワールド空間作用点、正規化方向、Magnitude、正のRadius、開始・終了時刻を持つ。start_time / end_time / event_timeは全てWorld Simulation Time原点からの絶対秒であり、Render Frame時刻・Clip Local Time・Logical Tick indexを直接渡さない。MagnitudeはForce=N、Impulse=N*sとする。対象Physics VertexのRest Lumped MassとWendland C2 Falloffを使い、q_i = m_i * phi(r_i/R)、w_i=q_i/sum(q)として正規化する。Soft BodyはTet Lumped Mass、ClothはTriangle Lumped Massを使い、固定頂点も支持反力を受ける頂点として分配集合へ含める。各Eventのw_i,eは、そのEventの対象Filter内で独立に総和1へ正規化する。分配Weightは各World共通Segmentの開始位置で距離r_iを評価し、Segment途中では変化させない。directionは単位ベクトル、magnitudeはForceならN、ImpulseならN*sである。

ForceはF_i=w_i*F_totalとして連続的に作用し、Force開始・終了時刻でWorld共通Segmentを分割する。各Segment内のForce積分量はF_i*dt_segmentであり、別途Impulseへ変換して重複加算しない。ImpulseだけをJ_i=w_i*J_eventとしてSegment開始時に一度適用する。イベントIDと適用済みTick / Segmentを記録し、Retry時は同じSource State / Input Snapshot / Segment時刻から同じイベントを再適用する。対象頂点が存在しない、Falloff重み総和がepsilon以下、イベント時刻が既にCommit済みTimelineより前に到着して反映できない場合はDiagnosticを返し、イベントを黙って捨てない。

### 14.4 JSON

トップレベルにschema_version、asset_id、asset_cache_version、asset_generation_settings、solver_settings、initial_state、animation、gravity、forces、collisions、materials、vertex_weights、test_sequence、random_seedを持つ。test_sequenceは開始条件、終了条件、再生・停止・Reset条件、測定区間を保持する。forcesはkind、位置、方向、強度、半径、継続時間、World Simulation Time原点からの絶対start_timeを持つ。Forceのend_timeはstart_time + durationから導出し、二重保存しない。Impulseはduration=0と絶対event_timeを持つ。Step番号をEvent時刻として入力しない。collisionsは接触厚み、摩擦、反発、collision_enabled、self_collision_enabled、Mesh Pair除外リストを保持する。solver_settingsはfixed_delta_time、time_scale、substeps、iterationsを保持する。

未知のschema_version、必須項目欠落、NaN/Infinity、負の時間刻み、範囲外パラメータはロードエラー。未知の任意拡張項目は保持して再保存できる。省略可能項目への既定値適用はログへ記録する。

### 14.5 再現性

Gatherの隣接IDを固定順序にし、同一GPU上で浮動小数点加算順序が変わらないようにする。整数Interlockedは容量カウント等に限定し、物理値の加算に使用しない。同一GPU・同一ドライバ・同一Buildで再現性を評価する。GPU間は許容誤差評価とする。

### 14.6 可視化・結果比較

Debug ViewはGPU上のPhysics / Render Bufferからデバッグ用線分・点・矢印を生成し、GPU内で描画する。通常のデバッグ可視化のためにPhysics VertexをCPUへReadbackしない。表示項目は以下とする。

- Render Mesh / Physics Mesh / Collision-only Collider / Collision Proxy
- 四面体辺、四面体品質、境界面、Render Binding対応点
- Stretch / Shear / Bend制約と制約値
- Broadphase階層セル、候補ペア、確定接触、接触法線
- 速度・変位ベクトル、四面体J・体積誤差
- Physics Attribute Heatmap（全チャンネル）
- CCD対象候補、TOI、overflow、初期貫通修正量

各テストRunの結果はRun Result JSONとして保存する。Run Resultはscenario_id、scenario_hash、asset_id / cache_version、build_id、GPU / driver情報、実行時間、Pass別GPU時間、Buffer割当Byte数、候補数、確定接触数、overflow / CCD回数、体積誤差、最大変位、各受け入れ試験結果を含む。同一scenario_hashのRunを並べて指標差と許容誤差を比較できる。GPUメモリはライブラリ所有のBuffer / Texture割当Byte数に加え、IDXGIAdapter3::QueryVideoMemoryInfoが利用可能な場合はDXGI_MEMORY_SEGMENT_GROUP_LOCALのCurrentUsage / Budgetを記録する。後者はプロセス全体の使用量とOS予算であり、Soft Body専用の使用量として誤認しない。IDXGIAdapter3が利用できない環境ではライブラリ所有リソース合計を必須記録値とし、使用量・予算の未取得を明示する。

## 15. GPU Pass・同期・Dispatch

v1のGPU Runtime初期値はpolar iteration最大8回、polar orthogonality residual <=1e-5、CFL factor=0.2、Cloth / Contact Gather relaxation候補omega={0.5, 0.75, 1.0}、CCD safety=0.8、CCD最大32 CA iterations、TOI split<=2 / substepとする。これらは初期Engineering Baselineであり、Numerical Acceptance Profile / Scenario Manifestへ値を記録し、fixture・WARP・物理GPU試験を通過するまで妥当性を確定しない。


### 15.1 Logical Tick / Substep transaction

Logical Tick全体をGPU Commit transactionとする。Tick開始時のactive_state_bankはRead-onlyで保持し、inactive_state_bankだけを更新するため、active_state_bankがLogical Tick開始Snapshotを兼ねる。Integration Segment開始状態は専用scratchへ保存し、TOI prefix / Event Segment再計算時はinactive_state_bankをそのscratchから復元する。Substep scheduleはLogical Tick Begin時にWorld全体の参加Meshから一度だけ計算し、全Soft Body / Cloth / Contact / Animation Trackingへ共通dt_worldを割り当てる。独立したSolver別Subcyclingは禁止する。Force / Impulse Eventの時刻がSegment内部にある場合もWorld共通Segmentを分割し、全参加Solverを同じ時間だけ進める。すべてのSubstep / Collision / CCD / Render Candidate Validationが成功した後だけGPU Tick Commit Controlを通す。GPU Commit ControlはPhysics State BankとRender Candidate Slotを同時に切り替える。Failure時にWorking State / CandidateをRendererへ公開しない。

基本Pass順序:

1. Logical Tick Begin / Dispatch Plan Gate / Terminal Status Reset。
2. World Substep Schedule Build / Validate。
3. Integration Segment境界のAnimation Target / Animation Target Velocity / Root / Kinematic Pose Sample。
4. External Force / Impulse Event Gather（ForceはSegment内一定、ImpulseはSegment開始時にEvent ID単位で一度だけ適用）。
5. Segment開始状態のSoft Body FEM Force / Kelvin-Voigt Tracking Force Evaluation / Tet Contribution Gather。
6. Soft Body / Cloth Velocity / Position Prediction（dt_segment）。
7. Swept AABB / Broadphase Count / Scan / Fill / Sort / Group / Candidate Dedup / Narrowphase / Contact Patch Build。
8. CCD Conservative Advancement / Minimum TOI Reduction。
9. TOIがない場合: Cloth Stretch / Shear / Bend / Animation Tracking等の非接触内部Constraintを解く。
10. TOIがある場合: Endpoint Contactを確定せず、Dynamicのx_path(tau) / v_path(tau)とKinematicのtau時刻実Trajectory位置・VelocityでTOI prefixを構成し、tauを使って非接触内部Constraintを解く。Endpoint ChordはDeviation Bound付きSwept AABBにのみ使う。
11. Dynamic VertexのContact前Velocityを再構築してv_pre_contactへ保存する。TOIではtau、通常Segmentではdt_segmentを用いる。Fixed / Kinematic VelocityはAnimation Trajectory微分を維持する。
12. Discrete / TOI Contact Position XPBDを解く。Contact位置補正はPositionへだけ適用し、v_pre_contactを変更しない。
13. v_pre_contactとKinematic接触点VelocityからFriction / Restitution Impulseを計算し、Contact ContributionをVertex Gatherで一度適用する。TOI Contactにはimpact_resolvedを設定する。
14. Broadphase Envelope / NonFinite / Capacity / CCD / Physical Quality Gateを検証する。Envelope外は同じSegment時間内でBroadphaseを再構築し、再度Contact解決する。
15. TOIがある場合はImpact後Velocityから残時間の新しいIntegration Segmentへ進む。TOI後に当該Impact結果をVelocity Reconstructionで上書きしない。
16. 次Integration Segment / World Substepへ進み、全Logical Tick完了後にRender Feedback Candidate Prepare（Position / Normal / Tangent）。
17. Render Candidate finite / Binding Validation。
18. Logical Tick Commit Control（全Physics GateとRender Candidateが成功した場合だけPhysics State GenerationとRender Slotを同時にCOMMITTEDへ切替）。
19. Diagnostic Timestamp / Counter / Status Copy + Event Query。

Discrete Contact / TOI Contactのいずれでも、Contact Position XPBDとVelocity Impulseを別Pass / 別Snapshotで扱う。Contact Position correctionを含むPosition差分から速度を再構築する処理は禁止する。

### 15.2 同期・リソースhazard

同じResourceをSRVとUAVへ同時にバインドしない。Pass境界で必要なUAV unbind / SRV bindを行う。読み書きが競合する反復はPing-Pong Bufferを使う。Immediate Contextの発行順序を基本とし、Deferred Contextを使う場合はリソース依存を保つ。Production Runtime v1ではDeferred Contextと複数Submission Threadを使わず、Host RendererとSkinPhysics Runtimeは同一Immediate Contextへ単一Submission Threadから順序付けて発行する。Dispatch間依存はPass境界で解決し、Thread Group間Global Barrierを仮定しない。

Compute Shader thread groupは初期値128 threadsとし、頂点・要素・制約別に64/128/256を実測比較する。Dispatch上限を超える場合は複数Batchに分割する。グループ内同期とDispatch間同期を混同しない。

### 15.3 GPUタイミング

PassごとにD3D11 timestamp query / timestamp disjoint queryを用いる。結果は複数フレーム遅延で取得し、同期待ちを挟まない。CPU時間、GPU時間、待ち時間を別々に記録する。

## 16. 容量・エラー・ステップ状態

シミュレーションインスタンスとGPU Logical Tickは状態を分けて管理する。

World Playback State:
- Ready
- Running
- Paused
- Stopped
- BlockedByCapacity
- BlockedByCCD
- DeviceLost
- Failed

Logical Tick State:
- Pending
- Submitted
- GpuExecuting
- GpuCommitted
- CpuObserved
- Rejected

GpuCommittedはGPU Commit Controlが書いた状態、CpuObservedはEvent Query完了・COMMITTED Status Readback・連続Tick Prefixの3条件をCPUが確認した状態である。CPU observed completed timeはCpuObserved Prefixから算出する。GPU StatusがCOMMITTEDでもEvent Query未完了または先行Tick未完了ならCPU completed timeを進めない。

予測位置と確定位置を分離する。Broadphase/Contact/Contributionのoverflowや非有限状態が検出されたStepはCommitしない。Broadphase envelopeはサブステップ開始位置から予測位置までのSwept AABBを基本とし、設定した最大拘束補正余裕を加える。位置反復後に頂点がこのenvelope外へ移動した場合は、次の反復前にBroadphaseを再構築する。再構築時に容量超過が起きた場合も同じStepをBlock / Retryする。容量を増やして同じStepを再実行するか、Failureとしてユーザーへ通知する。

診断コード:

- InvalidAsset / InvalidTopology
- DegenerateTetrahedron / InvertedTetrahedron
- InvalidMaterialParameter
- SkinWeightTransferError
- RenderBindingError
- BroadphaseOverflow / ContactOverflow / ContributionOverflow
- CCDIterationLimit
- InitialPenetrationLimit
- NonFiniteState
- ContactGeometryUnresolved
- ContactManifoldReductionFailure
- AnimationTimePrecisionUnresolved
- NoForceTarget
- GpuResourceAllocationFailure

警告はWorld ID、Model ID、Mesh ID、Element ID、Logical Tick Sequence、Substep、Pass名、関連カウンタを含む。NonFiniteState発生時は該当TickをCommitしない。GPU StatusのREJECTED / BLOCKED状態は非同期Readbackで回収し、CPU APIはSubmission受付成功とGPU成功を混同しない。

## 17. 初期試験・受け入れ基準案

数値は初期合格基準案。ベンチマーク前に達成済みと扱わない。GPU float32とCPU doubleの比較には [numerical_acceptance_profile_v1.json](numerical_acceptance_profile_v1.json) の正規化スケールと初期Engineering Thresholdを適用する。これらは実測校正前の提案値であり、分析fixture・既存Golden Test・低解像度GPU試験で誤判定がないことを確認する。変更時は理由と比較結果をレビュー記録へ残し、Profile Versionを更新する。

| ID | 試験 | 初期合格基準案 |
|---|---|---|
| T01 | Rest Pose無負荷10秒 | 最大ドリフトがBBox対角の0.1%未満 |
| T02 | Soft Body圧縮・解放 | 体積10%圧縮後の回復誤差5%以内。J分布も保存 |
| T03 | Soft Body剛体回転 | 内部弾性ひずみ誤差0.5%未満 |
| T04 | Soft Body Material Controls | Shape RecoveryとVolume Retentionが個別に作用し、μ/Kの基準値とも独立して調整できる |
| T05 | Cloth Stretch | Compliance増加で伸びが単調増加 |
| T06 | Cloth Shear / Bend | 各Compliance変更で該当変形が単調変化 |
| T07 | VT / EE | 双方の補正、接触後貫通が厚みの25%以下 |
| T08 | Self Collision | 近傍除外以外の貫通が反復後に減少 |
| T09 | CCD | 離散判定で見逃す高速交差をConservative Advancementで検出 |
| T10 | Broadphase | 全探索参照で得た接触ペアを候補が全て包含 |
| T11 | Overflow | 小容量設定で候補を欠落させずBlock / Retry |
| T12 | Binding | Rest Pose誤差最大値がBBox対角の0.1%以下 |
| T13 | Skin Weight | 全Physics Vertexのweight sumが1±1e-4、非負 |
| T14 | Reproducibility | 同一GPU反復で位置RMS差がBBox対角の0.01%以下 |
| T15 | Performance | GTX 1660 SUPERでPass時間、要素数、候補数、GPUメモリを取得 |
| T16 | CCD impact response | TOI後に衝突方向速度が更新され、残時間で再貫通せずImpulseが一度だけ適用される |
| T17 | Contact Patch | 重複接触の統合前後で過拘束が減り、最大貫通が設定閾値内 |
| T18 | Bend Gradient | 解析勾配と有限差分勾配の相対誤差が1e-3以下 |
| T19 | Recovery Controls | Shape RecoveryとVolume Retentionを個別に変更し、各応答への影響を計測 |
| T20 | Animation Inertia | Bone並進・回転で目標速度と物理遅れが発生し、通常再生中に速度が不意にリセットされない |
| T21 | Discontinuity | Teleport / Clip Blend / Resetで指定した速度引継ぎポリシーが適用される |
| T22 | Collision Filtering | デフォルト全ペア有効、個別Mesh Pair除外、Self Collision切替を検証 |
| T23 | Initial Repair Budget | 各頂点の累積修正量が上限を超えず、修正不能時に初期状態がCommitされない |
| T24 | Test Controls / JSON | Play / Stop / Pause / Reset / Step / Time Scale / Gravity / Impulseを保存・再読込後も再現できる |
| T25 | Physics Resolution | target edge length / target tet countの変更が生成規模へ反映され、実測値・キャッシュ識別が保存される |
| T26 | Asset Cache | Cache hitでランタイム再生成せず、入力・生成設定の変更やCache破損を検出する |
| T27 | Render Attribute Preservation | 物理変形後もUV / Color / Material / Indexが元Render Meshと一致する |
| T28 | Visualizer / Run Result | GPU Readbackなしで全Debug Viewを表示し、同一scenario_hashのRunを比較できる |

性能は1体の描画キャラクター（複数のBody / Clothing / Hair Mesh）と、別途配置したStatic / Kinematic Collision-only Colliderを基本シーンとし、Soft Body tet数、Cloth三角形数、接触密度、サブステップ数を変えたスケーリング曲線で評価する。代表規模は2k / 8k / 32k四面体、2k / 8k / 32k Cloth三角形を目安にし、組み合わせ衝突試験も記録する。3～40 FPSは測定対象であり全規模の保証値ではない。

## 18. 実装順序と段階完了条件

1. 共通アセット・glTF/GLB・OBJ Importer
   - 単位、座標、スキニング、トポロジー検査。
2. Physics Mesh生成・四面体品質・skin weight harmonic extension・cache
   - Rest Volume、反転、ウェイト残差、Cache再生成の検査。
3. Render Vertex Binding・GPU Feedback
   - Rest Pose誤差基準を満たす。
4. Test Environment・GPU計測・JSON
   - 固定時間刻み、Pause、Step、Timestamp、JSON再読込。
5. Cloth内部ソルバー
   - Stretch / Shear / Bend単体試験とJacobi収束比較。
6. Soft Body Corotational FEM
   - 剛体回転、圧縮、局所押込み、反転Barrier、CFL試験。
7. Broadphase / Narrowphase / 双方向接触
   - 全組み合わせ、全探索参照、容量超過試験。
8. CCD・初期貫通
   - 高速VT/EE、TOI分割、反復上限試験。
9. Animation Tracking
   - 平行移動、回転、Clip切替、Teleport試験。
10. 物理属性ペイント
    - 保存・再読込・物理メッシュ再生成後の転送試験。
11. 品質・性能最適化
    - Pass別計測と代表規模のスケーリング。

12. GPU Runtime Transaction / Async Scheduler / Render Interop
    - World Global ID、複数Model / Mesh統合、Host-owned Device / Context、GPU Commit / Rollback、Query Ring、Committed Render Binding、Device Lossを接続する。
    - Gate R0～R6を通し、既存T01～T28のRegressionを維持する。

各段階でレビューと試験結果を記録する。前段の完了条件を満たさず次段へ進まない。Phase 12のRuntime境界、API、GPU Commit State Machine、Error / Device Lossは [runtime_integration_design.md](runtime_integration_design.md) を正とする。

## 19. 実装前に残る実測・校正タスク

以下は設計上の未決定事項ではなく、本書で定義した方式・初期値・モデル適用範囲をGolden Fixture / WARP / Physical GPUで検証するタスクである。閾値を満たさない場合は原因を分類し、設計またはProfile Versionを改訂する。測定前にAcceptance Gateを緩和しない。

- Corotational Linear Elasticityの検証領域（Rigid Rotation、Affine Stretch / Shear、J=0.9 / 1.1、J=0.7 / 1.3、局所押込み、反転直前）での精度・安定性。保証領域外の大変形はv1の品質保証範囲としない。必要ならNeo-Hookeanを別候補として比較する。
- Polar / SVD fallbackの直交性残差、det(R)、特異値Floor、異常要素比率、GPU時間をNumerical Profileで検証する。
- μ / K / ρ / E_2D / G_2D / D / tracking stiffness / dampingは単位付きScenario Materialとして入力する。代表値はfixtureごとに固定し、Run Manifestへ記録する。
- Bend解析勾配とcentral finite differenceの相対誤差 <=1e-3、退化Hinge分岐、Jacobi収束を検証する。
- Spatial Hashの64 cells / Primitive、最大24 level、positive cell floor、Morton LBVH Large Primitive Bucket、Hash / BVH間Pair Dedupの完全性と候補数を小規模全探索Fixtureで検証する。
- Contact Patch最大4 Support PointのReduction前後で最大貫通・法線分布・支持範囲を測定し、未縮約Manifold参照と比較する。Patch分割条件もFixtureで検証する。
- CCD eta / safety / tolerance / CA 32 iteration / TOI 2 splitの見逃し・未解決率をMoving VT / EE Fixtureで測定する。False No-HitとUnresolvedの診断を独立して集計する。
- GPU Buffer Capacity Growth、旧Resource Set退役時のPeak Bytes、DXGI Local Memory Budget、Render Candidate Ringを含むVRAMピークをGTX 1660 SUPERで確認する。
- glTFの1 / 4 / 8 Skin Influence、STEP / LINEAR / CUBICSPLINE、複数Skin / Mesh Instance、OBJ Unit Error、未対応Morph / Draco / MeshoptのDiagnostic Fixtureを拡張する。
- numerical_acceptance_profile_v1.jsonの全閾値を既存Golden Fixture / Analysis Fixture / WARPで校正し、低解像度Hardware GPUでFP32 / Double差を測定する。
- Fixed Delta変更・Time Scale・Pose履歴不足・複数WorldのTick Scheduler・in-flight 1 / 2 / 4・Render Lease解放のRuntime統合試験を行う。
- Render Candidate Ringの追加VRAM、Position / Normal / Tangent生成時間、CPU-observed committed-prefix遅延を測定する。
- Device Loss / World Destroy / Query Ring pressure / Capacity retryをHardware D3D11で検証する。

## 20. 参考資料

- Sifakis & Barbic, FEM Simulation of 3D Deformable Solids: https://www.cs.toronto.edu/~jacobson/seminar/sifakis-course-notes-2012.pdf
- Macklin et al., XPBD (2016): https://mmacklin.com/xpbd.pdf
- Macklin et al., Small Steps in Physics Simulation (2019): https://mmacklin.com/smallsteps.pdf
- Müller et al., Position Based Dynamics: https://www.cs.cornell.edu/~utkarshm/cs775/pbd.pdf
- Zhang & Kim, Simple Culling Methods for Continuous Collision Detection of Deforming Triangles: https://graphics.ewha.ac.kr/projects/details/SCCD/
- Microsoft, HLSL InterlockedAdd: https://learn.microsoft.com/en-us/windows/win32/direct3dhlsl/interlockedadd
- Microsoft, Shader Model 5 Intrinsic Functions: https://learn.microsoft.com/en-us/windows/win32/direct3dhlsl/d3d11-graphics-reference-sm5-intrinsics
- Microsoft, IDXGIAdapter3::QueryVideoMemoryInfo: https://learn.microsoft.com/en-us/windows/win32/api/dxgi1_4/nf-dxgi1_4-idxgiadapter3-queryvideomemoryinfo
- Microsoft, DXGI_QUERY_VIDEO_MEMORY_INFO: https://learn.microsoft.com/en-us/windows/win32/api/dxgi1_4/ns-dxgi1_4-dxgi_query_video_memory_info

HLSL Shader Model 5のInterlockedAddはint / uintを対象とする。浮動小数点物理値のAtomic Addに依存せず、Gather方式を用いる。参考実装は式・境界条件の出典として扱い、実装をそのまま移植しない。

## 21. バージョン履歴

- 1.10: Shader Model 5固定幅GPU Packet ABI / Byte Offset、CPU double絶対時刻とGPU Span-local float時刻、Segmentごとのu_segment_start更新、逆再生CUBICSPLINE Tangent選択、Collider Recordとfloat時間Precision Errorを追加。
- 1.9: GPU Animation Trajectory Packetによる任意TOI時刻のBone / Skinning評価、最大2 Track Clip Blendの微分、非応答Contactの有効逆質量判定、複数Contactを結合するProjected Jacobi Impulse、Contact Temporal Quality Limitを追加。
- 1.8: Contact Position correctionとContact前Velocity / Restitution Impulseを分離し、Impact後Velocityを再構築で上書きしないPass順を定義。Animation Target Velocityの唯一の正本、Kinematic CCDの保守的Chord偏差Bound、Cloth Temporal Quality Limit、World absolute Event Timeを追加。
- 1.7: Semi-implicit Eulerの離散CCD pathとTOI prefix状態の一致、TOI時のXPBD Segment時間、Impact後残Segmentの再評価を明文化。
- 1.6: Integration Segment単位のSoft Body / Cloth予測、線形粘性減衰の速度積分、Fixed / Kinematic Velocityの維持、TOI時間変数とSegment Snapshotの整合を明文化。
- 1.5: World共通Substep、Cloth予測・Force / Impulse分配、Contact Patch品質閾値、退化Contact法線決定、Stackless Morton LBVH構築を明文化。
- 1.4: Broadphase初期値・Large Primitive処理、Self Collisionの既定Topology Exclusion、Contact Patchを最大4点のManifold Reductionへ改訂、CCD初期許容値、Render Feedback Candidate / Commit Slotの順序、実測校正タスクの区分を明文化。
- 1.2: 要件定義との整合性レビュー対応。Soft BodyのVolume / Shape Recovery独立調整、物理メッシュ解像度、Asset Cache、Mesh Pair Collision Filter、最大初期修正予算、アニメーション不連続と慣性、Test Environment操作、JSONシナリオ・Run Result、GPUメモリ予算計測、デバッグ可視化、ペイントチャンネルと頂点ウェイト反映規則を明文化。
- 1.1: 批判的レビュー対応。Corotational応力式、μ/Kの独立定義、反転ペナルティ、CFL推定、glTF Skin MatrixとPhysics Mesh skin weight転送、Cloth Shear/Bend、XPBD Jacobiのλ緩和、Contact Patch、TOI時Impulse、階層Spatial Hash、Collision-only Collider、Soft Body内部ペイント転送、overflow時の同一Step再試行を修正・具体化。
- 1.0: 初版。初期アルゴリズム、数式、GPU Pass、データ構造、エラー処理、試験仕様を定義。


## 22. GPU Runtime / Library Integration（v1.8）

本書の数式・既存Solver ABI・T01～T28を維持したまま、Production GPU RuntimeのAPI境界と非同期状態管理を追加する。詳細なPublic API、Host / Runtimeの所有権、World Global ID、Fixed Step Scheduler、GPU Tick State Machine、Commit / Rollback、Render Binding、Device Loss、Error Contractは [runtime_integration_design.md](runtime_integration_design.md) を正とする。

- 配布Targetは既存のSkinPhysics Visual Studio Static Libraryとし、.sln / .vcxprojを正規Build手段にする。CMakeを必須にしない。
- HostがD3D11 Device / Immediate Contextを生成・破棄する。RuntimeはCOM参照を保持してGPU Resourceを管理する。Host RendererとRuntimeは単一Submission ThreadでImmediate Contextを共有する。
- 1 Worldに複数Model Instance / Mesh Componentを登録し、Soft Body / Cloth / Noneを混在させる。Contact KeyはWorld / Model / Mesh / Primitive IDを持つ。
- 既存fixed_step_controllerとsimulation_step_pipelineはCPU Reference / Test Environment用に保持する。非同期GPU Commitの判定やProduction GPU Stateの所有には流用しない。
- GPUがRuntime Physics Stateの唯一の正。Commit済みRender BindingだけをHostへRead-only公開し、通常FrameでPhysics Vertex / Contact / Lambdaを全量Readbackしない。
- GPU StatusはLogical Tick単位でPENDING / SUBMITTED / GPU_COMMITTED / REJECTEDを管理する。CPU completed timeはEvent Query完了、COMMITTED Status Readback、連続Tick Prefixの3条件を満たした範囲だけ進める。
- 最大2件のin-flight Tickはexpected_predecessor_tick_sequence / expected_source_state_generation / expected_source_state_set / dispatch_idを持つImmutable Dispatch Planで連結する。Tick Commit ControlだけがCommitted State Generation / Active State Setを更新し、Rejectでは更新しない。
- 後続Tickのtick_begin_controlと全Budgeted Physics Passは先行TickのCommit Sequence / State Generationを確認する。先行Tickが失敗した場合、後続TickはPhysics StateのRead / Write前にBLOCKED_BY_PREDECESSORとして停止する。先行TickをRetryしてCommitした後、後続Tickを新しいGenerationで再Submissionする。
- get_render_bindingはCPUがEvent Query完了とCOMMITTED Status Readbackを確認した最後のCPU-observed committed State Generationを返す。未確認GPU CommitやWorking BufferをRendererへ公開しない。
- Capacity Overflow / CCD Unresolved / NonFinite / Physical Quality Gate失敗はTickをCommitしない。再試行は同一Tick InputとInitial Snapshotから行い、候補を黙って捨てない。
- Render Feedback Candidateは専用SlotへCommit前に生成・検証し、GPU Commit ControlがPhysics StateとRender Slot参照を同時に切り替える。CPU-observed committed prefixのみをHostへ公開する。Render Candidate RingはWorldごとにmaximum_in_flight_ticks + 1 Slotとし、Lease解放前のSlotを再利用しない。追加Buffer / Copy / Dispatch / Ring VRAM / GPU時間をPerformance Runへ含める。
- Device Loss時は新規Submissionを停止し、Asset / Initial StateからWorldを再構築してTimelineをResetする。毎Frameの全State Readbackで復旧しない。
- 数値・物理品質の初期比較基準はnumerical_acceptance_profile_v1.jsonを使い、同一Solver Algorithm / Update OrderのCPU Double Referenceと比較する。異なるSolver方式同士はAnalytical FixtureとPhysical Invariantで比較する。
- Production Runtimeの実装開始は独立Critical ReviewとGate R0～R6の実装計画承認後とする。採用値とReview Checklistは [design_closure.md](design_closure.md) を正とする。

本改訂は設計追加であり、GPU Production Runtimeの実装・Build・Physical GPU Performanceを意味しない。文書状態はINDEPENDENT REVIEW REQUIREDとする。
