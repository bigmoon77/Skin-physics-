# Skin Physics 設計閉鎖書（v1.7）

- 状態: **DESIGN CLOSURE CANDIDATE / NOT APPROVED**
- 正本要件: [requirements.md](requirements.md) v1.1
- 基本設計: [basic_design.md](basic_design.md) v1.5
- 詳細設計: [detailed_design.md](detailed_design.md) v1.7
- Runtime API / Device / Async Scheduler: [runtime_integration_design.md](runtime_integration_design.md) v1.4
- Numerical Acceptance: [numerical_acceptance_profile_v1.json](numerical_acceptance_profile_v1.json)
- Test Strategy: [testing_strategy.md](testing_strategy.md)
- Runtime UML: [uml/phase12_gpu_runtime.md](uml/phase12_gpu_runtime.md)

本書は、基本設計・詳細設計・Runtime統合設計に分散していたv1の採用範囲、初期パラメータ、GPU Capacity、時間管理、Render Bindingの寿命を一か所で照合するための決定記録である。数値は初期Engineering Baselineとして固定し、数値受け入れProfileとScenario Manifestに記録する。実機測定前の性能達成・物理品質保証を意味しない。

## 1. v1の対象範囲

### 1.1 対応機能

v1は以下を同一Simulation World内で扱う。

- 複数Model Instance、複数Mesh Component。
- Meshごとのphysics_mode: Soft Body / Cloth / None。
- Soft Body間、Cloth間、Soft Body–Cloth間の相互衝突。
- MeshごとのSelf Collisionと明示的なMesh Pair除外。
- Static / Kinematic Triangle Mesh Collider。
- Skeleton Animation、Clip Blend、Root Transform Sample、Animation Tracking、Teleport / Reset等の不連続処理。
- Render MeshとPhysics Meshのトポロジー分離、Render Vertex Binding、GPU Render Feedback。
- Soft Body / Cloth Material、頂点Physics Attribute、Paint / Smooth / Average / Erase、Heatmap。
- JSON Scenario / Run Result、Play / Pause / Stop / Reset / Step / Time Scale、Gravity / Force / Impulse、Debug View / GPU Pass timing。

1 Runtimeに複数の独立Worldを作成できる。World間の衝突・物理連成はv1対象外とする。各Worldの時間管理・Tick順序・Contact Keyは独立し、D3D11 Immediate ContextへのGPU CommandはRuntime全体で1本のSubmission Threadに直列化する。Worldごとのin-flight上限は個別に管理し、Runtime全体のQuery Ringは全Worldの最大in-flight数合計を収容する。

### 1.2 v1対象外

- Strand Hair専用の伸縮 / 曲げ / ねじりソルバー。
- 動的Rigid Body Solver、Fluid、Soft Body–Rigid Body双方向剛体運動。
- Runtime中のTetrahedralization、Physics Topologyの任意変更。
- Render Mesh以外の独立した低解像度Collision Proxyを使ったEmbedding接触応答。
- FBX Importer、Morph Target / Morph Weight Animation、Draco圧縮、Meshopt圧縮。
- D3D12 / Vulkan / CUDA Backend。
- CPU PhysicsへのRuntime Fallback。
- 毎Frameの全Physics Vertex / Constraint / Contact / Lambda Readback。

Hair Cards / Hair BundlesはCloth Meshとして扱う。未対応Asset Featureは警告だけで黙って無視せず、該当Featureを明示したImport Errorとする。

## 2. ソルバー方式と初期数値

### 2.1 Soft Body

- 四面体Corotational Linear Elasticity、force-based FEM。
- 右極分解 F = R S。通常要素はScaled Newton Polar Iteration、異常要素は3x3 Jacobi SVD補助Pass。
- 応力からTet単位4頂点Contributionを生成し、Vertex→Tet CSRで総和する。浮動小数点Atomic Addは使わない。
- Segment開始時に評価した力を用いる半陰的Euler + 線形粘性減衰の指数積分。Soft Body内部力は各World Integration Segment開始時に再評価する。
- Constitutive modelの主方式をNeo-Hookean FEM XPBDへ置き換えない。

初期値:
- Polar Iteration最大8回。
- Polar Orthogonality Residual目標 <= 1e-5。
- CFL factor = 0.2。
- Animation Trackingの時間刻み制限も評価し、FEM CFLとTracking制限の小さい方を使う。
- Inversion penaltyの係数はScenario Materialで必ず指定し、ゼロ値・未設定を暗黙採用しない。

Corotational Linear Elasticityの保証領域は解析Fixtureで定義する。少なくともRest / Rigid Rotation / Affine Stretch / Shear / J=0.9, 1.1 / J=0.7, 1.3 / Local Indent / Near Inversionを検証する。検証外の極端な変形をv1保証対象としない。モデル限界がAcceptanceを満たさない場合はNeo-Hookean等を別方式として比較し、既存式を黙って置換しない。

### 2.2 Cloth

- Stretch: XPBD Graph Coloring。
- Shear / Bend: XPBD Gather Jacobi。
- Animation Tracking: 頂点単位Vector XPBD Attachment。
- Cloth予測は各World共通Integration Segment開始位置・速度から行い、Force Eventを区間内一定、ImpulseをSegment開始時の一回適用として扱う。Velocity / Positionは詳細設計§6.6の共通Segment積分式を使用する。
- Force / Impulseは共通Mass-weighted Wendland C2 FalloffでPhysics Vertexへ分配する。ForceはN、ImpulseはN*s。Force開始・終了とImpulse event時刻でWorld共通Segmentを分割し、Impulseのみ一度適用する。
- XPBD Complianceは各Constraint Functionの単位と整合する物理剛性から生成する。
- Cloth Stretch / Shear / Bend / Animation Tracking / ContactのXPBD LambdaはPersistent State Bankへ保存しない。各World共通Integration Segment（通常Substep、Force / Impulse境界、TOI分割区間）開始時に0へ初期化し、そのSegment内のSolver反復だけ保持する。Complianceはalpha_tilde=alpha/(dt_segment^2)。Segment間 / Logical Tick間のWarm Startはv1で行わない。Persistent Contact CacheのContact Key / Normal / Patch GenerationだけをState Bankに保持する。

Cloth / Contact Gatherの比較候補は omega={0.5, 0.75, 1.0}。初期比較値は0.5とし、収束・誤差・最大貫通・GPU時間をFixture単位で比較する。Clampや平均化でSolverの物理重みを暗黙に変えない。

Cloth予測式（Dynamic Vertex）:

    z_i = damping_rate_i * dt_segment
    decay_i = exp(-z_i)

    damping_integral_i =
        dt_segment                                      // z_i == 0
        dt_segment * (1 - z_i/2 + z_i^2/6 - z_i^3/24)  // 0 < z_i < 1e-3
        (1 - decay_i) / damping_rate_i                 // z_i >= 1e-3

    v_i_star = decay_i
               * (v_i + inverse_mass_i * J_segment_i)
               + damping_integral_i * inverse_mass_i * F_external_i
    x_i_star = x_i + dt_segment * v_i_star

この式はSegment内一定Forceと線形粘性減衰のVelocity区間解を使い、Positionは半陰的Eulerで更新する。Fixed / Kinematic Vertexは上式を使わず、Segment終端Animation Target / Animation Velocityを維持する。位置制約解決後はDynamic Vertexだけをsegment startから速度再構築し、Fixed / Kinematic Velocityは変更しない。Impact / Friction / Restitution ImpulseはDynamic Vertexへ一度だけ適用する。ForceはF_external_i*dt_segment、ImpulseはJ_segment_iのみに現れ、同じ入力を二重積分しない。

### 2.3 Contact / Impact

- Unilateral XPBD Contact: C(x)>=0、lambda>=0。
- Contact position projectionとVelocity Impulseを分離。
- VTとEEの両方をサポートし、接触する双方のPhysics Vertexへ逆質量と勾配係数で補正。
- Coulomb frictionとrestitutionは速度Impulseとして処理し、同一TOI Eventを二重適用しない。
- Contact Complianceの初期値は0。omega_contact候補は{0.5, 0.75, 1.0}。
- Restitution初期値は0。FrictionはScenario Materialで明示する。

### 2.4 Contact Patch Manifold

- local_min_edge_lengthはVT対象三角形3辺、EE対象2辺のRest Edge Length中の最小正値。候補がない場合はL_ref。
- Surface Patch IDはPhysics Surface Triangleの共有Edge adjacencyからAsset Build時に付与する。EEは隣接FaceのStable Primitive ID最小値を代表Faceとする。
- 初期クラスタ条件はcluster_radius以下、法線内積cos(10度)以上。同一Mesh Pair / Surface Patch IDの組み合わせに限定する。
- Candidateは(Mesh Pair, Patch ID A/B, Contact Key)順に処理するSeed-based deterministic clustering。連鎖的近傍連結で無制限に広げない。複数Seed候補は最短距離、同距離ならSeed Contact Keyで決める。

1 Patchあたり最大4 Support Contactを保持する。選択順は次の通りとする。
1. 最深接触。
2. 最深接触から最も遠い接触。
3. 既存2点の線分から最遠の接触。
4. 既存3点の三角形から最大面積の接触。

距離・面積同値はCanonical Contact Key順で決定する。点 / 線状候補は独立Support Point数だけ保持し、最深1点だけへの縮約は禁止する。

初期Engineering Threshold:
- 縮約Manifoldを規定反復回数で解いた後の最大残存貫通増加 <= 0.05 * contact_thickness。
- 入力Candidate法線から最近傍保持法線への最大角度誤差 <= 5度。
- 平面Support Hull面積の保持率 >= 0.90。Support HullはCandidate / Retained Contact Positionを平均Contact法線の直交2D基底へ射影して算出する。
- 線状PatchはSupport Hull長さ保持率 >= 0.90、点状Patchは独立Support Point数一致。

基準を満たさないPatchは、相互距離が最大のContact Pairを決定的な2つのSeedとし、各Candidateを近いSeedへ割り当てる。同距離はContact Key順。Child Patchへ同じReduction / Quality Testを再帰適用する。分割深度上限は8。上限到達時も基準を満たさない場合はContact Manifold Reduction FailureとしてTickをRejectする。Patch総数 / Contact数はCapacity Check対象。これらは初期値でありRun Manifestへ保存し、Fixture / Hardware試験で校正する。Contact KeyまたはPatch Generationが変わったらlambdaをResetする。

### 2.5 Broadphase / Self Collision

Hierarchical Spatial Hash初期値:
- base_cell_width = max(median positive Primitive AABB max-extent, 2 * max_contact_thickness)。
- max_cells_per_primitive = 64。
- max_hash_levels = 24。
- cell_width(level) = base_cell_width * 2^level。
- cell_width_floor = max(1e-6 m, 1e-3 * L_ref)。
- 64セルを超えるPrimitiveはLarge Primitive Bucketへ登録する。
- Bucketの索引は30-bit Morton Code（各空間軸10-bit）+ Stable Primitive IDの辞書順Keyを用いるGPU Linear BVH。Karras方式で構築し、同Morton CodeもStable Primitive IDで一意化する。
- Node LayoutはAABB min/max、left/right child、parent、escape_index、leaf primitive ID。TraversalはStackless depth-firstとし、逃避先はそのNodeのSubtree直後のDFS Node。Traversal Stackは使用しない。
- Regular Primitiveは自身の膨張AABBでLarge BVHをQueryし、Regular-Large Pairを生成する。各Large PrimitiveもLarge BVHをQueryし、相手Stable IDが自身より大きいPairだけを生成してLarge-Largeの逆順重複を防ぐ。
- Hash / BVH候補はCanonical Pair KeyでUnion / DedupしてNarrowphaseへ渡す。全Large Primitiveまたは全Primitiveの総当たりを実行しない。
- N=0はBVHをSkipしてNode Count=0、N=1は単一Leaf RootとしKarras Internal Node BuildをSkip。N>=2はKarras方式。Node CountはN=0で0、N>=1で2*N-1。escape_indexの終端SentinelはNode Count。
- Node / Pair / Dedup BufferにCount / Capacity Checkを適用し、uint32 / ByteWidth超過はCapacity Error。
- 全Primitive総当たりへのFallbackは禁止。
- Hash一致だけでセル一致を判定せず、levelと元の整数Cell座標を比較する。

Self Collision既定除外:
- 同一Primitive。
- 同じPhysics Vertexを共有するMesh内Primitive Pair（Topological 1-ring）。
- Soft Bodyでは同一Tetに属する境界Face Pair。
- 明示的なMesh Pair Exclusion。

Additional 2-ring / 3-ring除外は既定0。折り畳み部の接触見逃しを避ける。除外Pairと残存ContactをDebug Viewで確認する。

### 2.6 CCD / TOI

- CCD適用候補の初期選別: relative_displacement > eta * contact_thickness、eta=0.5。
- Swept AABBが候補化したPairを、低速という理由だけで候補から落とさない。Initial Contact / Persistent Contact / Initial Penetrationは速度による選別の対象外。
- Conservative Advancement safety=0.8、最大32 Iterations。
- tau_tolerance = max(1e-7 s, 1e-5 * dt_segment)。
- distance_tolerance = max(1e-6 * L_ref, 1e-4 * contact_thickness)。
- tau_toleranceに到達しただけでNo-Hitにしない。距離条件または保守的区間No-Hitを証明できない場合はUnresolved。
- Positive-Time TOI splitは最大2回 / World共通Substep。TOI時刻は全Soft Body / Cloth / Contactを含むWorld全体へ適用する。
- 同時TOI Group tolerance = max(1e-7 s, 1e-5 * dt_segment)。ImpactはContact Key順に一度だけ適用。
- 分割上限超過 / UnresolvedはTickをCommitせず、World共通Substep数を増やしてTick開始SnapshotからRetryする。Retry上限到達時はBlockedByCCD / Failed。

### 2.7 World共通Substep / 外力イベント

v1ではWorld内のSoft Body / Cloth / Contact / Animation Trackingが同一時刻を進む。Solver別Subcyclingは禁止する。Logical Tick開始時に、

    dt_limit_world = min(all FEM CFL limits,
                         all Animation Tracking limits,
                         world_max_substep_delta_time if > 0 else +infinity)
    N_world = max(minimum_substeps, ceil(fixed_delta_time / dt_limit_world))
    dt_world = fixed_delta_time / N_world

を算出する。制限が全て未指定ならN_world=minimum_substeps。minimum_substeps初期値1、maximum_substeps初期値128。world_max_substep_delta_timeはSoft Body / Cloth / Contact / Animation Trackingすべてへ適用する共通dt上限であり、Cloth / Contactの品質要求によりさらに細かい刻みが必要な場合もこのWorld設定で制限する。N_worldが上限を超えた場合はWORK_BUDGET_EXCEEDEDとしてTickをCommitせずReservationを保持する。Cloth / Contact XPBDのComplianceはWorld共通Substep内の各dt_segmentを使う。各Force / Impulse Event境界とTOI時刻でWorld全体をSegment分割し、全参加Solverを同じSegment時間だけ進める。XPBD LambdaはSegmentごとにResetし、Segment内だけ累積する。

Force / Impulseの空間分配:
- Gravityは一様加速度としてPhysics Vertex iへrest_lumped_mass_i * gravityを適用する。
- phi(s)=(1-s)^4(4s+1), s=r/R, 0<=s<1。
- q_i = rest_lumped_mass_i * phi(s_i)、w_i=q_i/sum(q)。各Eventのw_iはそのEventの対象Filter内で独立に総和1へ正規化する。
- r_iはWorld共通Segment開始位置からEvent作用点までの距離。WeightはSegment内で固定する。
- Soft BodyはTet Lumped Mass、ClothはTriangle Lumped Mass。Fixed Vertexにも分配し、逆質量0で拘束反力を表す。
- ForceはN単位の方向付きVectorをF_i=w_i*F_totalとしてSegment全体へ作用。Force start/endでWorld Segmentを分割し、F_i*dt_segmentだけ運動量へ加える。
- ImpulseはN*s単位の方向付きVectorをJ_i=w_i*J_eventとしてSegment開始で一度だけ適用。Event IDでRetry中の二重処理を防ぐ。
- radiusは正の有限値。サポート対象0または重み総和epsilon以下はNO_FORCE_TARGET。

### 2.9 CCD離散軌道とTOI prefixの一致

- §6.6のSemi-implicit Eulerで得たSegment終端Velocityをpath_velocityとし、Dynamic VertexのCCD pathをx_path(tau)=x_segment_start+tau*path_velocityと定義する。
- Positive-Time TOIではこの同一pathのx_path(tau) / v_path(tau)をprefix状態として使う。dt=tauの再Predictionで別軌道を生成しない。
- TOI位置でtauをXPBD Integration Segment時間として内部Constraint / Contact Constraintを解き、Dynamic VertexのVelocityを補正後位置差分から再構築する。Fixed / Kinematic VelocityはAnimation Interface指定値を維持する。
- Impact後の残時間は新しいIntegration Segmentとして開始し、FEM / Animation Tracking / External Forceを新しい開始状態から評価する。
- TOI candidate pathとTOI prefix stateの一致、TOI時のCompliance時間、Impact後の残Segment継続をFixtureで検証する。離散軌道近似の精度範囲はNumerical Acceptance Profileで校正する。

### 2.8 Integration Segment積分・減衰・Kinematic Velocity

- Soft Body / Cloth / Contact / Animation TrackingはWorld共通Integration Segmentを同じdt_segmentで進める。通常SubstepはForce / Impulse Event境界またはTOIで複数Segmentへ分かれ得る。
- Segment内のForceは区間開始時に評価し、次Segment境界まで一定とする。Soft BodyのF_segmentはExternal + FEM Internal + Kelvin-Voigt Animation Tracking、ClothのF_segmentはExternal Forceとする。Cloth Animation TrackingはXPBD Attachment Constraintとして解く。
- ImpulseはSegment開始境界で一度だけ適用し、Forceへ変換しない。Event時間区間は半開区間[t_start, t_end)とし、Logical Tick終端上のEventは次Tick開始へ所属させる。Retry / TOI再積分でEvent IDを二重消費しない。
- Dynamic Vertexの速度更新は線形粘性減衰 dv/dt = inverse_mass * F_segment - damping_rate * v の区間解を使う。decay=exp(-damping_rate*dt_segment)、g=(1-decay)/damping_rate（damping_rate=0の極限はdt_segment）として、v_pred=decay*(v_start+inverse_mass*J_segment)+g*inverse_mass*F_segment、x_pred=x_start+dt_segment*v_predとする。小さいz=damping_rate*dt_segmentでは詳細設計§6.6の級数近似を使用する。速度は区間内一定Force・線形減衰に対する解、位置は半陰的Eulerである。
- Fixed / Kinematic VertexはDynamic積分式を使わず、Segment終端Animation Targetとその時間微分Animation Velocityを維持する。位置差分によるVelocity再構築やContact Impulseで上書きしない。Kinematic Velocityは相対速度・摩擦・反発・CCDに使う。
- XPBD LambdaはSegment開始時にResetし、Segment内の反復でのみ保持する。Impact後のDynamic Velocityを次Segmentへ渡し、Logical Tick全体の始点・終点差分で上書きしない。

## 3. Asset / Material / Animation

### 3.1 Import / Offline Build

v1 Runtime Import:
- OBJ: 呼出元がMeter per Unitを必須指定。
- glTF 2.0 / GLB: glTFのmeter-based coordinateとskin / animation情報を利用。
- ASCII Gmsh MSH 2.2: Soft Body用Physics MeshのOffline中間入力。

fTetWildは外部Offline Tool Adapterとし、Runtimeのリンク依存にしない。Physics Asset Cacheはsource hash、generator identity / version / options、coordinate_system_version、material_layout_version、cache schema versionを照合し、一致しないCacheを採用しない。

FBX / Morph Target / Draco / Meshoptは未対応。Importerは未対応FeatureをErrorとして明示する。

### 3.2 Material / Physics Attribute

材料値は単位付きScenario Dataとして保存する。普遍的なMaterial Presetを勝手に固定しない。
- Soft Body: mu [Pa], K [Pa], rho [kg/m^3], damping [1/s], tracking stiffness [N/m], tracking damping [N*s/m], shape_recovery_scale, volume_retention_scale。
- Cloth: E_2D [N/m], G_2D [N/m], D [N*m], areal density [kg/m^2], damping, tracking properties。
- Contact: thickness [m], friction coefficient, restitution coefficient, optional compliance with declared constraint units.

頂点AttributeはRender Mesh側のStable ID / Weightを基準に保持し、Physics Meshへ既定のBinding/Harmonic Transfer規則で転送する。Paint / Smooth / Average / Eraseの編集を不変なversioned editとして受け付け、Physics側に反映したversionとsource Mesh versionを記録する。再Mesh化後に対応が一意でない場合は自動適用しない。

### 3.3 Animation Input

Animation ClipのSTEP / LINEAR / CUBICSPLINE補間をImporter契約に従って保持する。CPUはBone Hierarchy / Skin Matrixを評価し、GPUはPhysics Mesh Skinning Target / Target Velocityを生成する。Root TransformとKinematic Colliderは時刻付きSampleからTick / Substep境界へ補間する。

Runtime Model Root TransformはTranslation + Rotationのみの剛体Transformとし、ScaleはIdentity固定。非一様・負・ゼロScaleや実行中のRoot Scale変更は拒否する。Static Mesh / Mesh Node ScaleはOffline Build時にGeometry / Rest Data / Mass / BindingへBakeする。Bone Scale Animationは正値かつ特異でない成分のみ許可し、負値・ゼロ・符号反転をImport Errorとする。

必要時刻をbracketできるだけのSample履歴がない場合、Runtimeは直前Poseの延長で速度を捏造しない。対象TickをBLOCKED_BY_INPUTとして保留し、欠落時間範囲をDiagnosticへ返す。Teleport / Reset / Time Rewindは明示Discontinuityとし、PreserveVelocity / ResetVelocity / BlendVelocityのPolicyを適用する。

## 4. Runtime / GPU Resource契約

### 4.1 WorldとSubmission

- 1 Runtimeに複数の独立Worldを持てる。World間のCollision / Physics Couplingは行わない。
- 1 Worldに複数Model Instance / Mesh Component / Colliderを登録できる。
- Worldごとのmaximum_in_flight_ticksの既定値は2、設定可能値は1 / 2 / 4。
- 1 PhysicsSubmissionBatch = 1 Logical Tick。
- 各Worldは2つのPersistent Physics State Bankを持つ。active bankはTick中immutable、inactive bankへ全Physics結果を書き、全Gate成功時にCommit Controlでactive bankとGenerationを切り替える。
- Reject / Blockではactive bank / Generationを維持し、inactive bankを再利用する。Substep / CCD Segment用SnapshotはWorldごとの再利用Scratchであり、3つ目のPersistent State Bankではない。
- Runtime全体のQuery Ringは、全Worldのmaximum_in_flight_ticks合計以上のSlotを持つ。
- Immutable Dispatch Planはpredecessor tick sequence、source state generation / set、dispatch identityを持つ。後続Tickは先行Tick Commit Gateを通過しなければPhysics Stateへアクセスしない。

### 4.2 Fixed Time

- Fixed Delta Time初期値=1/60 s。Test Environmentで正のFinite値へ変更可能。
- Time ScaleはFrame elapsedをAccumulatorへ加える倍率のみ。Fixed Delta Timeは変更しない。
- advanceで供給された時間はrequested_simulation_timeへ累積する。single_stepで受理したFixed Deltaもrequested_simulation_timeへ加算する。未割当時間はAccumulatorが保持し、TickをSubmissionする際にFixed DeltaをTick Reservationへ一度だけ移す。
- cancelled_requested_timeはStop / Resetで取消した未Reservation Step Requestの累積時間。requested_simulation_timeから減算せず、effective_requested_time = requested_simulation_time - cancelled_requested_timeとしてBacklogを評価する。
- Tick ReservationはPose / Kinematic Input、Render Candidate Slot、Query Slotが揃ったGPU Submission直前に作成する。作成時にAccumulatorまたはStep Request Queueから固定Delta Timeを移し、Tick Sequence / Input Snapshot / time_sourceを不変化する。CPU ObservationはAccumulatorを変更しない。
- BLOCKED_BY_INPUT / Render BackpressureがSubmission前に発生した場合はReservationを作らず、時間をAccumulatorまたはStep Request Queueに保持する。
- Capacity / CCD / PredecessorによるBlock / Retryでは同一Reservation / Tick Sequenceを維持する。Retry時に時間をAccumulatorへ再加算したり、二重に消費したりしない。
- Fixed Delta Timeの変更はPaused、in-flight=0、Accumulatorのwhole ticks=0、pending Step request=0でのみ許可する。fractional remainderは秒単位で保持し、新しいdtに対してwhole tick数を再評価する。
- fractional remainderは秒単位の実時間として保持し、新しいdtでwhole tick数を再計算する。
- 変更時はCloth / Contact / Animation XPBD LambdaとTick ScratchをResetする。旧dtのwhole tickを新dtで再解釈しない。
- Step要求はPaused / Stopped中にのみ受理する。受理時にrequested_simulation_timeへ1 Fixed Delta Timeを加算するが、wall-clock Accumulatorは変更せず、独立Step Queueへ1 Fixed Tickを追加する。Pause / Stop中はStep QueueをAccumulatorより優先し、Pose / Render Slot / Query Slotが揃うまでStep Requestとして保持して、GPU Submission直前にTick Reservationへ移す。Accumulatorのwhole tick / fractional remainderをStepのために消費しない。
- Pauseはwall-clock Accumulatorを保持する。Stopは未Reservation Step Requestを取消してcancelled_requested_timeへ計上するが、wall-clock AccumulatorとPhysics Stateは保持する。ResetはAccumulator全体、Step Queue、Reservation / Sequence、Solver Cache、Timeline CountersをInitial Stateへ戻す。Tick Reservation済みのStepは取消し・二重要求を行わず通常のCommit / Retryに従う。
- 最大Step / Substep / Iteration予算超過時に時間をdrop / clampしない。

### 4.3 Capacity Growth / Memory

各可変長Resourceはcurrent_capacity、required_count、overflow flag、element stride、resource group ID、producer/consumer identityを持つ。

Capacity Growth:
- required_countをCPUへ小規模非同期Readback。
- 拡張Capacity = next_power_of_two(max(required_count, ceil(current_capacity * 1.5)))。
- 初回0容量はnext_power_of_two(required_count)。
- uint32 / D3D11 ByteWidth / Dispatch Limitを超える場合はGrowth対象外のCapacity Error。
- Resource Group全体を再確保できるMemory Budgetを先に検査し、必要な新Resource Setを確保してから切替。
- 旧Resource Setは全in-flight Tick / Query / Render Leaseから切り離されるまで破棄しない。
- Capacity Growth中は新規Submissionを止め、先行状態が確定した後、同一Snapshot / InputからRetryする。後続Tickは新State Generationに基づいて再発行する。
- Capacity不足 / Memory不足を理由にContact / Candidate / Contributionを切り捨てない。

Memory budget:
- Runtime-owned Resource Bytes。
- DXGI Local Memory CurrentUsage / Budget。
- Host Reserved GPU Bytes。
- Library Limit = min(runtime_desc.maximum_gpu_resource_bytes, max(0, dxgi_budget - host_reserved_gpu_bytes))。
- Render Candidate Ring / Rollback Snapshot / Query / Stagingを含むすべてのLibrary Resourceを計上する。
- Capacity縮小はv1では行わない。診断でCapacityとPeak Bytesを残す。

### 4.4 Render Candidate / Binding Lease

Render Candidate SlotはTick Reservationごとに確保する。Slotがなければ新規Reservation / Submissionを停止し、未割当時間をAccumulatorへ、発行済み時間をReservationへ保持する。

各TickはSolver全Substep後に専用Render Candidate SlotへPosition / Normal / Tangentを生成し、Binding / Geometry / NonFinite Validationを実行する。すべてのPhysics GateとRender Candidate Validationが通過した場合だけ、GPU Commit ControlがCommitted Physics State Set / GenerationとRender Slot ID / Generationを同一Controlで切り替える。Reject時は両方を維持し、Working State / Candidate SlotをHostへ公開しない。

Render Candidate Ring:
- Slot数 = world_desc.maximum_in_flight_ticks + 1。
- 既定in-flight 2の場合は3 Slots。CPU-observed直前Committed Slot 1つと、最大2件のin-flight / 未観測Candidateを保持する。
- Query / Status未回収、in-flight Tick、Host Leaseが存在するSlotは上書きしない。
- get_render_bindingの最初の成功呼出しでWorld単位のLeaseを取得し、返すlease_generationを全Mesh共通にする。
- release_render_bindings(world, lease_generation)は現在Leaseと一致する世代だけを解放する。古い / 二重ReleaseはSTALE_HANDLE。
- Hostは全MeshのDrawコマンドをImmediate Contextへ発行し、公開SRVをVS / HS / DS / GS / PS / CS全StageからUnbindする。Deferred Contextを使った場合は該当Command ListをImmediate ContextへExecuteCommandListしてからReleaseする。
- Release後はSRV Pointerと未実行Command List内の参照を使用しない。COM AddRefはCOM Objectの寿命だけを延ばし、Slot内容のLeaseは延長しない。
- Render Lease未解放で次Submissionを要求した場合はBUSY / render_binding_busyでBackpressureし、Accumulator / Reservation時間を保持する。
- Copy / Publish Pass / Render Candidate Validation / Ring Slot VRAM・GPU時間をPerformance Runへ含める。

この方式はCPU-observed commitとRendererのBuffer内容を一致させるためのv1契約であり、GPUでCommit済みでもCPUが観測していない新しいRender Slotを直接公開しない。

### 4.5 Model / World / Runtime Resource Lifecycle

- destroy_model / destroy_colliderは対象Worldのin-flight Tickが0件、対象Resourceを参照するTick Reservationが存在せず、Host Render Leaseが存在しない場合だけ成功する。BLOCKED_BY_INPUT / Render Backpressure中の未Submission要求はModel / Collider Snapshotを固定していないため、Destroy後の次回Scheduler評価で現在のWorld構成を使う。条件不足はBUSY。成功するまでHandle / Resourceは有効。
- reset_worldはin-flight Tick = 0、Host Leaseなしでのみ成功する。Pending Reservation / Accumulator / Step Queue / Contact Cache / Lambda / Diagnostic / requested / cancelled / committed / observed timeline countersをResetし、保存済みInitial Stateへ戻る。
- destroy_worldはPaused / Stopped、in-flight Query / Tick = 0、Host Lease = 0、Pending Reservation / Pending Step Request = 0、未割当Accumulator = 0でのみ成功する。条件不足はBUSY。残時間の破棄はStop / Resetによる明示操作とし、whole tick / fractional remainderの取消しをDiagnosticへ残す。
- GPU Command / Queryが参照したResourceはCompletion QueryまでDeferred Release Listで保持する。
- drain()は明示的な非Frame API。新規Submissionを停止し、既にSubmission済みGPU Tick / Queryの完了とDeferred Releaseを待つ。未割当Accumulator whole tick、未処理Step Request、Host Leaseがある場合はBUSYを返す。Host Leaseを強制Releaseしない。
- shutdown()は全WorldがDestroy済み、Query Ring Idle、Deferred Release List空、Host Lease=0の時だけCOM Resourceを解放してOK。条件不足ならBUSYを返し、Runtime状態を壊さない。Runtime Destructor前にshutdown成功が必要。
- 通常Frame PathにFlush / 同期Waitを入れない。明示drainのみGPU Query完了待ちを許可する。

## 5. Validation / Acceptance

### 5.1 Canonical numerical profile

[numerical_acceptance_profile_v1.json](numerical_acceptance_profile_v1.json)をGPU float32 / CPU doubleの比較、Local / Whole-step誤差、物理Invariantの初期機械可読Profileとする。値は初期Engineering Thresholdであり、既存Golden / Analytical Fixture / WARPで誤判定がないことを確認し、Hardware GPU低解像度試験で校正する。変更時は理由と差分を記録し、Profile Versionを更新し、関連Suiteを再実行する。

軌跡比較はCPUとGPUの式、アルゴリズム、更新順、反復順、Substep境界が同一のときだけ行う。異なるSolver Algorithmは解析Fixture、物理Invariant、収束・エネルギー応答で比較する。

### 5.2 Existing requirements and tests

- 要件定義書§12 T01～T28を維持する。
- Runtime Integration Gate R0～R6をPhase 12のProduction Gateとする。
- Windows CI / D3D11 WARPはShader / ABI / Correctness検証。WARP時間をPhysical GPU性能として扱わない。
- GTX 1660 SUPER 6GBで2k / 8k / 32k Tetと2k / 8k / 32k Cloth、および混在・接触密度のスケーリングを計測する。
- 3～40 FPSはWorkload別の開発評価レンジ。全Assetでの保証値ではない。
- 性能を理由にContact / Candidate / Iteration / Substep / Collision Pair / Mesh Quality / Simulation Timeを黙って削減しない。

### 5.3 次段階へ進むためのReview Checklist

- [ ] Requirements v1.1の全必須項目がBasic / Detailed / Runtime / Test Gateへ追跡されている。
- [ ] Soft Body / Cloth / Contact / Animation Tracking / TOIがWorld共通Substep / Segmentで同じ時刻を進む。
- [ ] Cloth予測・Force / Impulse分配の式とイベント境界が確定している。
- [ ] Tick ReservationがAccumulator、Retry、GPU Commit、CPU Observation間で二重消費・時間欠落を起こさない。
- [ ] Render LeaseはGeneration付きReleaseで stale / double releaseを拒否する。
- [ ] Model / Collider / World / RuntimeのDestroy / Reset / Drain / ShutdownにBUSY条件がある。
- [ ] BVHのKey順、Node Layout、Stackless traversal、Capacity failureが定義済み。
- [ ] Contact Patch品質閾値と退化法線のUnresolved経路が数値・状態として確定している。
- [ ] Runtime Root TransformのScale制約とAsset Build Bake規則が確定している。
- [ ] Soft Body Corotational FEMとCloth XPBDを混同していない。
- [ ] Multi-World / Multi-Model / Multi-Mesh / Self Collision / Mixed CollisionのID・Filter・Ownershipが一貫している。
- [ ] Contact Patchが複数Support Pointを保持し、縮約前後の品質を比較可能。
- [ ] Broadphase / Large Primitive / Pair Dedupの完全性とCapacity failure pathが定義済み。
- [ ] CCD No-Hit / TOI / Unresolved / Initial Contactの区別がある。
- [ ] Fixed Delta / Time Scale / Pose History / Step / Pause / Resetの時刻所有者が一意。
- [ ] GPU CommitとRender Candidate Slotの切替が同一Commit Controlで行われる。
- [ ] Render Binding LeaseのSlot数、Unbind、Release、再利用条件が定義済み。
- [ ] Capacity Growth、Memory Budget、Rollback / Retry、Predecessor Blockの状態遷移が一貫している。
- [ ] Numerical Profileの正本と変更管理が明確。
- [ ] 実装開始後の各GateにPersistent Fixture、Expected Result、Executable Assertion、Failure Caseがある。

**次の工程は独立Senior Engineer Critical Reviewとする。** 重大矛盾0件、未接続Commit / Error path 0件を確認した後にProduction Runtime実装へ進む。本書と設計書の追加だけでレビューPASS、Build成功、GPU性能達成とは扱わない。


## 6. 要件追跡マトリクス（requirements.md v1.1）

| 要件節 | 主な要求 | 設計上の反映先 | 検証先 |
|---|---|---|---|
| §1 目的 | 同一キャラクターの肉体・衣服変形、研究・可視化 | basic_design §1-2、detailed_design §6-13、design_closure §1-5 | T01-T28、R0-R6 |
| §2 環境・GPU制約 | Windows / C++17 / D3D11 / SM5 / GPU physics / no normal-frame readback / no CPU fallback | basic_design §17、detailed_design §3 / §15-16、runtime_integration §3-9 | R0 / R1-R6、No hidden CPU fallback |
| §3 用語・対象 | Render / Physics Mesh分離、Soft Body / Cloth / None、Hair Cards、Multi-Mesh | basic_design §3-4、detailed_design §2 / §4-5、runtime_integration §2-3 | P01-P03、R0 / R3 / R5 |
| §4 物理方式 | Tetrahedral Soft Body、Cloth Stretch / Shear / Bend、混在 | detailed_design §6-7、design_closure §2 | T01-T06、R1-R3 |
| §5 Import / Convert | Multiple Importers、common asset、independent topology、cache、diagnostics | detailed_design §2 / §4-5、runtime_integration §3、design_closure §3.1 | P01-P03、T12 / T13 / T25 / T26 |
| §6 Animation | Skeleton pose / velocity、inertia、tracking、discontinuity、render attribute preservation | detailed_design §5 / §6.7 / §12、runtime_integration §5.4 / §9、design_closure §3.3-4.4 | T20 / T21 / T27、R5 |
| §7 Collision | Mixed collision, self collision, bilateral response, initial repair, CCD, material, filters, no brute-force, overflow safety | detailed_design §8-11、design_closure §2.3-2.6 | T07-T11 / T16 / T17 / T22 / T23、R3 / R4 |
| §8 Physics attributes / Paint | per-Mesh base values, per-vertex channels, Paint / Smooth / Average / Erase / transfer / persistence | detailed_design §13、runtime_integration §3.3-3.4、design_closure §3.2 | P10, T04 / T06 / T13 / T19 / T27 |
| §9 Test Environment | Play / Stop / Pause / Reset / Step / Time Scale / fixed dt / gravity / force / impulse / visualizer / GPU timing | detailed_design §14 / §17、runtime_integration §5 / §7 / §10 | T15 / T24 / T28、R0 / R5 / R6 |
| §10 JSON | readable versioned Scenario / Run Result / compatibility / deterministic seed | detailed_design §14.3-14.6、runtime_integration §3.4 / §4、existing scenario schema | T24 / T28、R0 |
| §11 Performance / Quality | GTX 1660 SUPER 6GB, 3-40 FPS evaluation range, memory budget, correctness vs performance | basic_design §12 / §17、detailed_design §17 / §19、runtime_integration §8 / §10、design_closure §5 | T15、R6 |
| §12 Acceptance | Soft Body / Cloth / mixed / self collision / CCD / animation / render binding / overflow / reproducibility / GPU stats | detailed_design §17、testing_strategy R0-R6 | T01-T28、R0-R6 |
| §13 Out of scope | no Runtime external physics lib, no CPU physics, no all-model import, no Strand Hair, no cross-GPU bit exact | runtime_integration §2 / §12、design_closure §1.2 | Import negative tests, No CPU fallback, R0-R6 |
| §14 Process | requirements -> design -> independent review -> implementation; changes versioned | README, basic_design, detailed_design version history, design_closure §5.3 | Review Checklist / CI evidence |
| §15 History | source requirement version history | requirements.md itself; design docs record corresponding version | Review metadata |

**追跡上の注意:** 要件§5のImporter形式は「複数形式」という抽象要件を満たすためv1具体形式をOBJ / glTF 2.0 / GLBへ固定した。FBX / Morph / Draco / Meshoptはv1未対応として明示Errorにする。§13の未確定事項は基本・詳細設計で具体化済みであり、残る数値は設計未決定ではなく測定・校正タスクである。
