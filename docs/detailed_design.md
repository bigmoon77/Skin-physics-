# Skin Physics 詳細設計書

- 文書バージョン: 1.1
- 状態: 初版レビュー対応・レビュー合格（数値パラメータは段階試験で確定）
- 対応要件: requirements.md 1.1
- 対応基本設計: basic_design.md 1.1
- 前版: 1.0
- 実装対象API: Direct3D 11 / HLSL Shader Model 5
- 言語・ビルド: C++17 / Visual Studio .sln・.vcxproj

## 1. 本書の目的と設計判断

本書は基本設計で残したアルゴリズム、数式、GPUデータ構造、処理順序、数値処理、エラー処理を実装可能な粒度へ具体化する。本書のレビューが合格するまで実装を開始しない。

### 1.1 初期版の方式

| 領域 | 初期採用方式 | 補足・代替 |
|---|---|---|
| Soft Body | 四面体Corotational Linear Elasticity、半陰的Euler | 反転要素はSVD補助処理と体積ペナルティ |
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

内部座標系は右手系Y-upとし、Importerで座標軸・三角形表裏・法線を一度だけ変換する。描画、物理、アニメーションでは同じ内部座標系を用いる。

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
- 三角形インデックス、サブメッシュ、Material slot
- Bone hierarchy、inverse bind matrix、bone weights
- Animation clip、keyframe、補間方式
- 物理方式、物理属性、衝突設定
- 単位・座標系・軸変換情報

Skinning influenceは初期版で最大8本とする。インポート時に上位ウェイトを選択して正規化する。ウェイト合計が0、joint index不正、NaN等は入力エラーとして報告する。

## 3. CPU / GPU責務

### 3.1 CPU側

- アセット読み込み、入力検査、Physics Mesh生成・キャッシュ
- 四面体・三角形の隣接リスト、制約リスト、Graph Coloringの構築
- アニメーションClip評価、Bone行列生成
- GPUリソース生成・破棄、固定時間刻み制御
- JSON入出力、試験環境制御
- 非同期GPU診断結果の回収と容量再設定

CPUは通常フレームのPhysics Vertex位置・速度をReadbackしない。GPU診断結果の回収は非同期とし、通常処理を同期Readbackで停止させない。

### 3.2 GPU側

- Physics MeshのSkinning目標位置と目標速度の生成
- Soft Bodyの変形勾配、局所回転、応力、内部力
- Cloth制約の予測・解決
- Broadphase / Narrowphase / CCD
- 接触位置補正、速度摩擦、反発
- Render Mesh位置・法線・接線更新
- GPUタイムスタンプ計測対象Pass

### 3.3 GPUリソース構成

初期構成はSoAを基本とし、帯域・strideの実測後にAoSへ変更する。

| バッファ | 形式例 | 更新頻度 | 主な用途 |
|---|---|---|---|
| position_read / position_write | SRV/UAV StructuredBuffer<float4> | 反復ごと | 位置Ping-Pong |
| previous_position | StructuredBuffer<float4> | サブステップごと | 速度再構築 |
| velocity | RWStructuredBuffer<float4> | サブステップごと | 速度 |
| inverse_mass | StructuredBuffer<float> | 初期化時 | 逆質量 |
| rest_position | StructuredBuffer<float4> | 初期化時 | Rest Pose |
| animation_target | UAV出力 / SRV入力 StructuredBuffer<float4> | 描画フレーム・サブステップ | Skinning目標 |
| animation_velocity | UAV出力 / SRV入力 StructuredBuffer<float4> | サブステップ | 目標速度 |
| animation_lambda | RWStructuredBuffer<float4> | サブステップ反復 | Cloth追従制約の3軸λ |
| tet_indices | StructuredBuffer<uint4> | 初期化時 | 四面体接続 |
| tet_rest_data | StructuredBuffer<TetRestData> | 初期化時 | Bm、V0、材料ID |
| vertex_tet_offsets / ids | StructuredBuffer<uint> | 初期化時 | Vertex→Tet CSR |
| tet_force_contrib | UAV出力 / SRV入力 StructuredBuffer<float4> | サブステップ | 四面体ごとの4頂点力 |
| cloth_constraints | StructuredBuffer<ConstraintData> | 初期化時 | Stretch / Shear / Bend |
| constraint_lambda | RWStructuredBuffer<float> | 反復ごと | XPBD λ |
| vertex_constraint_offsets / ids | StructuredBuffer<uint> | 初期化時 | Vertex→Constraint CSR |
| cell_records | UAV出力 / SRV入力 StructuredBuffer<CellRecord> | サブステップ | Spatial Hash登録 |
| contact_candidates | UAV出力 / SRV入力 StructuredBuffer<ContactCandidate> | サブステップ | VT / EE候補 |
| contact_data | SRV/UAV StructuredBuffer<ContactData> | 反復ごと | 接触法線・係数・λ |
| vertex_contact_offsets / ids | StructuredBuffer<uint> | サブステップ | Vertex→Contact CSR |
| contact_contrib | UAV出力 / SRV入力 StructuredBuffer<float4> | 反復ごと | 接触補正寄与 |
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
        float4 previous_position;
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

### 5.3 GPU Skinning

glTFのSkin Matrixは次で定義する。

    skin_matrix[j] = inverse(mesh_node_global_matrix)
                   * joint_global_matrix[j]
                   * inverse_bind_matrix[j]

inverseBindMatricesが省略されている場合は全Jointで単位行列を使う。CPUはSimulation Timeに対応するAnimation ClipのPoseを評価してBone行列をGPUへアップロードする。描画フレーム内に複数の物理Stepがある場合も、各Step時刻のPoseを評価または時間補間する。GPUはPhysics MeshのRest Positionとweightから各Stepの目標位置を計算する。前回Step目標位置との差分をそのStepの固定時間刻みで除してAnimation Velocityを得る。

## 6. Soft Body Solver詳細

### 6.1 採用モデルと適用範囲

初期版は四面体Corotational Linear Elasticityを採用する。有限回転を扱うが、ひずみ自体は線形弾性近似である。極端な圧縮・伸長を無制限に正確に表現するモデルではない。大変形試験で品質が不足する場合は、Neo-Hookean等の超弾性モデルを次版候補として比較する。

### 6.2 材料パラメータ

UIで独立調整可能な材料値は、せん断弾性率mu、体積弾性率K、密度rhoとする。muとKはいずれも正値。Lameの第一定数は次で求める。

    lambda_lame = K - 2 * mu / 3

EとPoisson比nuで入力する場合は以下に変換する。

    mu = E / (2 * (1 + nu))
    K = E / (3 * (1 - 2 * nu))

初期UIではnuを0.0～0.45に制限する。E/nuモードとmu/Kモードを混在させず、内部保存値はmu/K/rhoへ統一する。体積とせん断の応答を別々に調整できる。

### 6.3 Corotational応力

変形勾配Fの右極分解をF = R * Sとする。Rはdet(R)=+1を満たす回転、Sは対称ストレッチである。Corotational Linear Elasticityのひずみエネルギーと第一Piola応力は次を基準とする。norm(A)^2はFrobenius norm squared、すなわちtrace(transpose(A)*A)とする。

    epsilon_corot = transpose(R) * F - I
    Psi = mu * norm(F - R)^2
         + 0.5 * lambda_lame * trace(epsilon_corot)^2

    P = 2 * mu * (F - R)
      + lambda_lame * trace(epsilon_corot) * R

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

半陰的Eulerを使う。サブステップ開始位置と速度を保持し、外力・内部力を加えて予測する。

    force_total = force_external + force_internal + force_animation
    velocity_next = velocity + delta_t * inverse_mass * force_total
    velocity_next *= exp(-damping_rate * delta_t)
    position_next = position + delta_t * velocity_next

固定頂点はposition_next = animation_target、velocity_next = animation_velocityとする。Soft Body内部力は各物理サブステップで再評価する。

### 6.7 Animation Tracking力

柔軟頂点の追従は材料弾性と分離したKelvin-Voigt型追従力とする。

    force_animation =
        tracking_weight * (
            tracking_stiffness * (animation_target - position)
          + tracking_damping * (animation_velocity - velocity))

tracking_stiffnessはN/m、tracking_dampingはN*s/m。tracking_weightは0～1。材料のmu/Kと独立して調整できる。過大な値は時間刻み安定条件に影響するため、後述の安定性推定に含める。

### 6.8 明示積分の安定時間刻み

陽的内部力 + 半陰的Eulerは条件付き安定であり、1/240秒にしただけで全メッシュ・全材料の安定性を保証しない。

材料の縦波速度を

    c_p = sqrt((K + 4 * mu / 3) / density)

各四面体の最小高度を

    h_t = 3 * V0 / max_face_area

として、初期サブステップ上限の推定値を次とする。Animation Tracking stiffnessがある場合は、頂点質量m_iと追従剛性k_iからomega_i=sqrt(k_i/m_i)を求め、delta_t_tracking_limit=c_tracking/omega_maxも評価する。c_trackingは初期値0.2として実測検証する。最終上限はFEMとTrackingの小さい方を採用する。

    delta_t_limit = cfl_factor * min_t(h_t / c_p_t)
    cfl_factor = 0.2  // 初期評価値。実測で検証する

これは要素形状・材料・境界条件を含めた厳密な安定保証式ではなく、初期の保守的スクリーニングである。上限超過時は警告し、設定に応じてサブステップを増やす。最大サブステップ数を超えたら明示的にエラーとし、時間刻みを黙って無視しない。

### 6.9 体積・せん断評価

muは主にせん断応答、Kは体積応答を制御する。局所圧縮・膨張試験では要素ごとのJと全体体積を別々に記録する。Corotational Linear Elasticityは大変形時に線形化誤差が増えるため、体積10～30%変化、局所押込み、反転直前の試験を行い、適用範囲を数値で示す。

## 7. Cloth Solver詳細

### 7.1 XPBD共通式

制約関数C(x)、Compliance alpha、固定サブステップdelta_t、制約ごとのlambdaを用いる。

    alpha_tilde = alpha / (delta_t * delta_t)
    delta_lambda_raw =
        (-C(x) - alpha_tilde * lambda_old)
        / (sum_i(inverse_mass_i * dot(gradient_i, gradient_i))
           + alpha_tilde)

Compliance alphaは制約関数の単位に依存する。エディタで扱う材料値は物理的な伸縮弾性率・せん断弾性率・曲げ剛性とし、Physics Mesh構築時にRest Geometryを使って各制約の剛性kへ変換する。各制約のalphaは1/kとする。制約関数の単位、kの単位、alphaの単位をConstraintDataに記録する。

- Stretch距離制約: Cはm、kはN/m、alphaはm/N。
- Shearひずみ制約: Cは無次元、kはJ、alphaは1/J。
- Bend角度制約: Cはrad、kはN*m、alphaは1/(N*m)。

初期版のCloth材料入力は面内引張剛性E_2D (N/m)、面内せん断剛性G_2D (N/m)、曲げ剛性D (N*m)とする。Stretch辺のdual width wは内部辺では隣接2三角形の辺直交高度の平均、境界辺では隣接三角形の高度の半分とし、Rest LengthをL0とするとk_stretch=E_2D*w/L0。Shear三角形はk_shear=G_2D*area。Bend hingeのdual width wは隣接2三角形の辺直交高度の平均とし、共有辺長lを使いk_bend=D*w/lとする。これらは離散化の初期モデルであり、解像度変更時のエネルギー一致を試験する。

分母が閾値以下、全頂点が固定、または値が非有限の場合は補正を出さず診断する。λは各サブステップ開始時に0へ初期化し、同一サブステップの反復中は保持する。初期版ではサブステップ間Warm Startを行わない。

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
    alpha_tilde = alpha_animation / (delta_t * delta_t)

各軸に対してXPBD更新を行い、lambda_animationはfloat3で頂点単位に保持する。lambdaは各サブステップ開始時にリセットし、反復中に維持する。Animation TrackingはStretch / Shear / BendのComplianceと独立して調整する。

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

接触対象はSoft Body境界面とCloth表面である。Soft Bodyの内部四面体面は外部接触プリミティブに登録しない。

VTはメッシュペアごとにA頂点→B三角形、B頂点→A三角形の双方を候補化する。EEは辺ペアを一度だけ正規化キーで生成する。自己衝突では同一・近接トポロジー要素を除外する。候補キーはMesh ID、Primitive ID、接触種別を含み、重複をNarrowphase前に排除する。

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

接触位置補正はdelta_lambda_appliedを使う。0 < omega_contact <= 1とし、初期比較値は0.5 / 0.75 / 1.0。contact Complianceは初期版で0を既定値とするが、完全剛体接触が振動する場合に限り小さいComplianceを比較する。

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

各反復で最近接点と重心座標を再計算する。VTとEEの複数接触が同じ局所接触パッチへ集中すると過拘束になるため、同一メッシュペア・隣接面パッチ内でContact Patchを形成する。初期重複判定は、接触点距離が`cluster_radius = max(0.5 * contact_thickness, 0.02 * local_min_edge_length)`以下、かつ法線内積がcos(10度)以上の場合とする。同一Cluster内では最も深い接触を1つ残し、深さ同値の場合はContact Key順で決定する。離れた接触点は別Constraintとして保持する。閾値は試験設定として変更可能にし、統合前後の最大貫通を計測する。λは同一サブステップの同一接触キーで保持し、接触キーが変わった場合は初期化する。

### 8.4 Edge-Edge

    qA = (1 - s) * a0 + s * a1
    qB = (1 - t) * b0 + t * b1
    C_ee = dot(n, qA - qB) - contact_thickness

    gradient_a0 = +(1 - s) * n
    gradient_a1 = +s * n
    gradient_b0 = -(1 - t) * n
    gradient_b1 = -t * n

有効逆質量は4頂点のinverse_mass * gradient_coefficient^2の総和とする。平行・退化辺では区間最近接点の安定分岐を用い、有限値を保証する。

### 8.5 摩擦・反発

位置XPBD接触と速度Impulseを分離する。位置反復中に摩擦を重ねて適用しない。

Discrete contactでは位置解決後にvelocity = (position_after - previous_position) / delta_tを再構築する。TOIでImpulseを解決したcontactはimpact_resolvedフラグを設定し、同一Step終端のImpulse処理から除外する。TOIを伴わないDiscrete contactについて、接触点の相対速度をv_rel、法線速度をv_n = dot(v_rel,n)とする。接触が接近中の場合のみ、

    j_n = -(1 + restitution) * min(v_n, 0) / effective_inverse_mass

の法線Impulseを計算する。

接線速度v_t = v_rel - v_n*nから摩擦Impulse候補を求め、Coulomb円錐でクランプする。

    j_t_candidate = -v_t / effective_inverse_mass
    j_t = clamp_magnitude(j_t_candidate, friction * j_n)

接触の4頂点へ、接触勾配係数と逆質量でImpulseを配分する。摩擦係数は0以上、反発係数は0～1。初期反発係数は0とする。接触前の相対速度を保存し、同一接触で複数反復するたびに反発を重複適用しない。TOI contactはimpact_resolvedフラグにより同じImpulseをStep終端で二重適用しない。

### 8.1.1 Collision-only Collider

初期版では衝突専用オブジェクトとしてStatic / Kinematic Triangle Meshをサポートする。Colliderは物理頂点の逆質量を0とし、静的形状または固定時間刻みごとに与えられるTransformで移動する形状として扱う。Kinematic Colliderの頂点速度は前Stepと現在Stepの変換後位置から算出し、CCDと接触の相対速度へ含める。接触位置補正・ImpulseによってCollider自体は変形させない。

変形オブジェクトの衝突面は初期版ではSoft Body境界面とCloth Physics Mesh表面を使う。低解像度Collision Proxyを変形メッシュへ割り当て、そのProxy接触を高解像度Physics Meshへ戻す埋め込み応答は初期版の対象外とする。将来追加時はProxy頂点からPhysics MeshへのEmbeddingと、接触勾配のPhysics Vertexへの合成を定義する。

## 9. Broadphase Spatial Hash

### 9.1 多段階セル

単一セル幅では大型プリミティブが過剰なセルへ登録されるため、階層セル幅を用いる。

    cell_width(level) = base_cell_width * 2^level

base_cell_widthは代表PrimitiveのAABB寸法の中央値と最大接触半径の2倍の大きい方を初期候補とし、候補生成数・登録セル数の計測で調整する。各Primitiveは、接触厚みを膨張させたAABBが設定上限max_cells_per_primitive以下のセル数になる最も細かいlevelへ登録する。最大levelでも上限を超える場合はBroadphase Errorとする。全Primitive総当たりへフォールバックしない。

各query Primitiveは自分のlevelから最大levelまでを検索し、各levelでquery AABBと重なる全セルを列挙して整数セル座標を比較する。Primitiveは自身のlevelより細かいlevelへ検索を戻らない。異なるlevelのペアは小さいPrimitive側が大きいPrimitiveのlevelを検索することで検出する。VTは頂点AABB対三角形AABB、EEは辺AABB対辺AABBを検索する。

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

初期化時に一度Broadphase / Narrowphaseを実行し、初期接触を抽出する。位置XPBD接触を設定上限回数まで解き、頂点ごとの総修正量と残存貫通を計測する。各初期接触の制約評価値は、1反復あたり`C_effective = max(C, -0.1 * local_min_edge_length)`として過大な一括補正要求を制限する。この有効制約値をλ更新にも用い、位置だけを後からClampしてλと状態が乖離しないようにする。最大反復後も許容貫通を超える場合は警告またはAsset Load Errorとし、修正結果を自動確定しない。

自己衝突除外:

- 同一三角形・同一辺
- 共有頂点・共有辺
- 隣接面および同一Soft Body四面体近傍
- 設定した近傍リング数以内のトポロジー隣接

近傍除外リングを広げすぎると折り畳み部の接触が欠落する。除外数・除外距離・テスト結果を可視化する。

## 11. Continuous Collision Detection

### 11.1 適用条件

相対移動距離が接触厚みの一定比率を超える候補をCCD対象とする。

    length(relative_displacement) > eta * contact_thickness

etaの初期値は0.5。これは候補の選別値であり、CCD自体の正しさを保証する値ではない。薄い布・高速移動試験で見逃し率を測定し、必要なら対象範囲を広げる。

### 11.2 Swept AABB

各頂点・辺・三角形の前サブステップ位置と予測位置を含むSwept AABBを生成する。接触厚みで膨張させ、Broadphase候補を絞る。

### 11.3 Conservative Advancement

各候補の軌道をサブステップ内の線形補間とする。これは前Step位置と予測位置を結ぶ離散軌道モデルであり、サブステップ中の曲線軌道を厳密に表すものではない。大きな加速度・高剛性時はCFL制約に従いサブステップを細かくし、近似誤差を抑える。秒単位の時刻tauにおける距離をd(tau)、接触厚みをr、相対速度上限をv_boundとする。

VT:

    v_bound = length(path_velocity_vertex)
             + max(length(path_velocity_tri0), length(path_velocity_tri1), length(path_velocity_tri2))

EE:

    v_bound = max(length(path_velocity_edgeA0), length(path_velocity_edgeA1))
             + max(length(path_velocity_edgeB0), length(path_velocity_edgeB1))

保守的な進行量:

    delta_tau = safety * (d(tau) - r) / max(v_bound, epsilon)
    tau_next = tau + delta_tau
    safety = 0.8  // 初期値

d(tau)<=r+distance_toleranceでTOIを確定する。tau_next>delta_t_substepかつ終端距離がrより大きければ当該サブステップでは接触なし。最大反復回数、最小進行時間tau_tolerance、距離許容誤差distance_toleranceを設定する。進行不能または反復上限に到達した場合、未解決CCDとして診断し、候補を黙って破棄しない。

### 11.4 TOI処理

同一サブステップ内の最小TOIをGPU Reductionで求める。TOI時刻は秒単位のtauで扱い、0 <= tau <= delta_t_substepとする。衝突判定用の軌道はサブステップ開始位置と予測位置を結ぶ線形補間とし、path_velocity=(position_predicted-position_start)/delta_t_substep、x(tau)=x_start+tau*path_velocityと定義する。v_boundはこのpath_velocityの速度上限から求める。Conservative Advancementの進行量は次とする。

    delta_tau = safety * (d(tau) - contact_radius) / max(v_bound, epsilon)
    tau_next = tau + delta_tau

接触が確定したら、予測状態をそのまま部分更新しない。サブステップ開始スナップショットへ戻し、tauまでの区間を再積分する。Clothなど位置ベースの系では、この区間の変位から接触前速度を再構築する。Soft Bodyは積分器が保持するtau時点の速度を使う。tau時点で接触位置制約を解き、続けて法線・Coulomb接線Impulseを一度適用して速度を更新し、contactにimpact_resolvedフラグを立てる。その速度から残時間delta_t_substep-tauを再積分し、必要ならBroadphaseを再構築する。Soft Body内部力とCloth拘束は各区間で再評価する。TOI処理は最大分割数内で繰り返す。TOIを含むStepでは全時間の始点・終点差分から速度を再構築してImpulse結果を上書きせず、最後の区間で得た速度を保持する。

1サブステップ内のTOI分割上限は初期値2回。上限を超えた場合は残時間を進めずCCDIterationLimitとしてステップを未確定にし、時間刻みを細分化して再試行する。CCDの解決不能を貫通許容として扱わない。

## 12. Render Vertex Binding

### 12.1 四面体内部

四面体内の描画頂点は4頂点の重心座標で位置を補間する。

    x_render = beta0*x0 + beta1*x1 + beta2*x2 + beta3*x3

Rest Poseでの埋め込み誤差を計測し、最大誤差が基準を超える頂点はBuild Errorまたは明示的な警告とする。

### 12.2 境界外頂点

境界三角形への最近接点q、重心座標、Rest Pose offsetを保存する。rest_offsetはRest Poseのモデル空間ベクトルとして保存し、Rest Pose境界三角形から局所基底T0=[e1_0,e2_0,n0]を作る。変形後は対応三角形からT1=[e1_1,e2_1,n1]を作る。

    rest_offset = x_render_rest - q_rest
    x_render = q_deformed + T1 * inverse(T0) * rest_offset

対応三角形が退化・反転した場合は直前有効基底を使うのではなく、その頂点をBinding Errorとしてマークし、診断する。頂点IDの再検索は毎フレーム行わない。

### 12.3 Cloth Binding

同一トポロジーは直接index対応。異なるトポロジーはRest Pose表面の三角形・重心座標を保存する。距離誤差が閾値超過した頂点を自動的に最近傍頂点へ置き換えない。

### 12.4 法線・接線

Render Mesh三角形ごとに変形後の面法線を求め、Render Vertex単位でGatherして正規化する。Physics VertexのWelding情報をそのまま法線集約に使わない。UV seam / hard edgeは描画属性単位で分離して扱う。

接線は変形後位置とUV微分から再計算し、法線に直交化する。UV退化三角形は安定した直交ベクトルへフォールバックし、診断数を記録する。

## 13. 物理特性・ペイント

材料はMesh単位の基準値、頂点ウェイトは0～1のチャンネル値として保持する。Soft Bodyはmu/K/rho、減衰、Animation Tracking stiffness/dampingを管理する。ClothはStretch/Shear/Bend compliance、面密度、減衰、Animation Trackingを管理する。

Render MeshからPhysics Meshへ属性を転送する場合、Render Vertex Bindingを逆方向に単純平均しない。Soft Body境界Physics VertexはRender Mesh表面三角形から重心座標で補間する。Soft Body内部頂点は境界Physics Vertexのチャンネル値をDirichlet境界条件とするグラフLaplacian調和拡張で伝播する。Cloth頂点は直接対応または表面三角形補間を使う。転送誤差、残差、重みの分散を記録し、曖昧な転送は未確定とする。Physics Mesh再生成時は元Render Meshの安定IDを優先し、トポロジー変更で対応が一意に決まらない場合は自動適用せず警告する。

Paint / Smooth / Average / Eraseを提供する。ブラシ半径はモデル空間長さで扱い、画面ピクセル入力はカメラからモデル空間へ変換する。ペイントデータはJSONまたはアセットサイドカーファイルで保存する。

## 14. 時間管理・JSON・再現性

### 14.1 固定時間刻み

描画Delta Timeを直接ソルバーへ渡さない。Accumulatorで固定時間刻みのStep要求を管理する。フレームあたりの最大Step数を超えた場合は遅延を記録し、シミュレーション時間を黙って飛ばさない。

### 14.2 Step / Pause

Pause中のStep要求は固定時間刻みを1回だけ実行する。描画フレーム更新とは独立させる。複数Step要求は要求数分をキューイングする。GPU容量エラー中はStepを保留する。

### 14.3 JSON

トップレベルにschema_version、asset_id、asset_cache_version、solver_settings、initial_state、animation、forces、collisions、materials、vertex_weights、test_sequenceを持つ。

未知のschema_version、必須項目欠落、NaN/Infinity、負の時間刻み、範囲外パラメータはロードエラー。未知の任意拡張項目は保持して再保存できる。省略可能項目への既定値適用はログへ記録する。

### 14.4 再現性

Gatherの隣接IDを固定順序にし、同一GPU上で浮動小数点加算順序が変わらないようにする。整数Interlockedは容量カウント等に限定し、物理値の加算に使用しない。同一GPU・同一ドライバ・同一Buildで再現性を評価する。GPU間は許容誤差評価とする。

## 15. GPU Pass・同期・Dispatch

### 15.1 サブステップ基本順

1. Animation Target / Animation Velocity Update
2. Physics Vertex Previous State Snapshot
3. Soft Body Tet Deformation Gradient
4. Soft Body Rotation / Stress / Inversion Penalty
5. Soft Body Tet Force Contribution
6. Soft Body Vertex Force Gather
7. Soft Body Velocity / Position Prediction
8. Cloth Velocity / Position Prediction
9. Broadphase Count / Scan / Fill / Sort / Group
10. Candidate Generation / Deduplication
11. 各ソルバー反復
    - Cloth Stretch Graph Color Passes
    - Cloth Animation Tracking Constraint Output / Gather / Apply
    - Cloth Shear Constraint Output / Gather / Apply
    - Cloth Bend Constraint Output / Gather / Apply
    - Contact Narrowphase Refresh
    - Contact XPBD Output / Gather / Apply
    - Contact Patch Reduction / Merge
    - Broadphase Envelope Exceeded Check; 必要時Rebuild
12. TOI検出・TOI分割処理（CCD対象のみ）
13. Velocity Reconstruction
14. Coulomb Friction / Restitution Impulse
15. State Commit
16. Render Position Feedback
17. Render Normal Gather
18. Render Tangent Update
19. Diagnostic Timestamp / Counter

### 15.2 同期・リソースhazard

同じResourceをSRVとUAVへ同時にバインドしない。Pass境界で必要なUAV unbind / SRV bindを行う。読み書きが競合する反復はPing-Pong Bufferを使う。Immediate Contextの発行順序を基本とし、Deferred Contextを使う場合はリソース依存を保つ。

Compute Shader thread groupは初期値128 threadsとし、頂点・要素・制約別に64/128/256を実測比較する。Dispatch上限を超える場合は複数Batchに分割する。グループ内同期とDispatch間同期を混同しない。

### 15.3 GPUタイミング

PassごとにD3D11 timestamp query / timestamp disjoint queryを用いる。結果は複数フレーム遅延で取得し、同期待ちを挟まない。CPU時間、GPU時間、待ち時間を別々に記録する。

## 16. 容量・エラー・ステップ状態

シミュレーションインスタンスは少なくとも以下の状態を持つ。

- Ready
- Running
- Paused
- BlockedByCapacity
- BlockedByCCD
- Failed

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
- GpuResourceAllocationFailure

警告はMesh ID、Element ID、Pass名、関連カウンタを含む。NonFiniteState発生時は該当StepをCommitしない。

## 17. 初期試験・受け入れ基準案

数値は初期合格基準案。ベンチマーク前に達成済みと扱わない。変更時は理由と比較結果をレビュー記録へ残す。

| ID | 試験 | 初期合格基準案 |
|---|---|---|
| T01 | Rest Pose無負荷10秒 | 最大ドリフトがBBox対角の0.1%未満 |
| T02 | Soft Body圧縮・解放 | 体積10%圧縮後の回復誤差5%以内。J分布も保存 |
| T03 | Soft Body剛体回転 | 内部弾性ひずみ誤差0.5%未満 |
| T04 | μ/K独立調整 | μ変更はせん断応答、K変更は体積応答に主に影響 |
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

性能はSoft Body tet数、Cloth三角形数、接触密度、サブステップ数を変えたスケーリング曲線で評価する。代表規模は2k / 8k / 32k四面体、2k / 8k / 32k Cloth三角形を目安にし、組み合わせ衝突試験も記録する。3～40 FPSは測定対象であり全規模の保証値ではない。

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

各段階でレビューと試験結果を記録する。前段の完了条件を満たさず次段へ進まない。

## 19. 残課題・実装前に確定する項目

- Corotational Linear Elasticityの適用可能ひずみ範囲と、Neo-Hookean比較が必要となる境界
- SVD異常要素Passの精度とコスト
- μ/K/ρの代表材料プリセットと安定時間刻み
- Soft Body / Cloth材料プリセットの初期値とUI許容範囲の実測調整
- Bend微分の数値検証・finite difference test
- Spatial Hashのbase cell width、level数、max_cells_per_primitive
- Contact Patchの初期閾値と、Conservative Advancementのsafety / tolerance / 最大分割数の実測調整
- GPUバッファの容量初期値・上限・GTX 1660 SUPER実メモリ予算
- glTF skin influence・AnimationClipのImporterテストセット
- 数値受け入れ基準の実測妥当性

## 20. 参考資料

- Sifakis & Barbic, FEM Simulation of 3D Deformable Solids: https://www.cs.toronto.edu/~jacobson/seminar/sifakis-course-notes-2012.pdf
- Macklin et al., XPBD (2016): https://mmacklin.com/xpbd.pdf
- Macklin et al., Small Steps in Physics Simulation (2019): https://mmacklin.com/smallsteps.pdf
- Müller et al., Position Based Dynamics: https://www.cs.cornell.edu/~utkarshm/cs775/pbd.pdf
- Zhang & Kim, Simple Culling Methods for Continuous Collision Detection of Deforming Triangles: https://graphics.ewha.ac.kr/projects/details/SCCD/
- Microsoft, HLSL InterlockedAdd: https://learn.microsoft.com/en-us/windows/win32/direct3dhlsl/interlockedadd
- Microsoft, Shader Model 5 Intrinsic Functions: https://learn.microsoft.com/en-us/windows/win32/direct3dhlsl/d3d11-graphics-reference-sm5-intrinsics

HLSL Shader Model 5のInterlockedAddはint / uintを対象とする。浮動小数点物理値のAtomic Addに依存せず、Gather方式を用いる。参考実装は式・境界条件の出典として扱い、実装をそのまま移植しない。

## 21. バージョン履歴

- 1.0: 初版。初期アルゴリズム、数式、GPU Pass、データ構造、エラー処理、試験仕様を定義。
- 1.1: 批判的レビュー対応。Corotational応力式、μ/Kの独立定義、反転ペナルティ、CFL推定、glTF Skin MatrixとPhysics Mesh skin weight転送、Cloth Shear/Bend、XPBD Jacobiのλ緩和、Contact Patch、TOI時Impulse、階層Spatial Hash、Collision-only Collider、Soft Body内部ペイント転送、overflow時の同一Step再試行を修正・具体化。
