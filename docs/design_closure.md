# Skin Physics 設計閉鎖書（v1.4）

- 状態: **DESIGN CLOSURE CANDIDATE / NOT APPROVED**
- 正本要件: [requirements.md](requirements.md) v1.1
- 基本設計: [basic_design.md](basic_design.md) v1.3
- 詳細設計: [detailed_design.md](detailed_design.md) v1.4
- Runtime API / Device / Async Scheduler: [runtime_integration_design.md](runtime_integration_design.md)
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
- Semi-implicit Euler。Soft Body内部力はInternal Substepごとに再評価する。
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
- XPBD Complianceは各Constraint Functionの単位と整合する物理剛性から生成する。
- Lambdaは各Substep開始時に0へ初期化し、同一Substepの反復中は保持する。初期v1はSubstep間Warm Startを行わない。

Cloth / Contact Gatherの比較候補は omega={0.5, 0.75, 1.0}。初期比較値は0.5とし、収束・誤差・最大貫通・GPU時間をFixture単位で比較する。Clampや平均化でSolverの物理重みを暗黙に変えない。

### 2.3 Contact / Impact

- Unilateral XPBD Contact: C(x)>=0、lambda>=0。
- Contact position projectionとVelocity Impulseを分離。
- VTとEEの両方をサポートし、接触する双方のPhysics Vertexへ逆質量と勾配係数で補正。
- Coulomb frictionとrestitutionは速度Impulseとして処理し、同一TOI Eventを二重適用しない。
- Contact Complianceの初期値は0。omega_contact候補は{0.5, 0.75, 1.0}。
- Restitution初期値は0。FrictionはScenario Materialで明示する。

### 2.4 Contact Patch Manifold

局所Patchの初期クラスタ判定:
- cluster_radius = max(0.5 * contact_thickness, 0.02 * local_min_edge_length)
- 法線内積 >= cos(10度)
- 同一Mesh Pairかつ同じ連結Surface Patchの範囲だけで統合する。

1 Patchあたり最大4 Support Contactを保持する。選択順は次の通りとする。
1. 最深接触。
2. 最深接触から最も遠い接触。
3. 既存2点の線分から最も遠い接触。
4. 既存3点の三角形から最大面積を形成する接触。

距離・面積同値はCanonical Contact Key順で決定する。候補集合が点 / 線状の場合、独立なSupport Point数だけ保持する。最深1点だけへ縮約する方式は禁止する。縮約前後の最大貫通、法線分布、Support Regionを比較し、接触品質が悪化する場合はPatchを分割する。Contact KeyまたはPatch Generationが変わったらlambdaをResetする。

### 2.5 Broadphase / Self Collision

Hierarchical Spatial Hash初期値:
- base_cell_width = max(median positive Primitive AABB max-extent, 2 * max_contact_thickness)。
- max_cells_per_primitive = 64。
- max_hash_levels = 24。
- cell_width(level) = base_cell_width * 2^level。
- cell_width_floor = max(1e-6 m, 1e-3 * L_ref)。
- 64セルを超えるPrimitiveはLarge Primitive Bucketへ登録する。BucketはMorton-sort GPU Binary BVHで索引し、通常PrimitiveのAABB QueryとLarge-Large BVH Self-TraversalでPairを生成する。Count / Capacity / Dedupを適用し、全Primitive総当たりへ移行しない。
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
- Conservative Advancement safety=0.8。
- CA最大32 Iterations。
- tau_tolerance = max(1e-7 s, 1e-5 * delta_t_substep)。
- distance_tolerance = max(1e-6 * L_ref, 1e-4 * contact_thickness)。L_refは対象Physics Meshのmedian positive rest-edge length。
- tau_toleranceに到達しただけでNo-Hitにしない。距離条件または保守的区間No-Hitを証明できない場合はUnresolved。
- Positive-Time TOI splitは最大2回 / Substep。
- 同時TOI Group tolerance = max(1e-7 s, 1e-5 * delta_t_substep)。ImpactはContact Key順に一度だけ適用。
- 分割上限超過 / UnresolvedはTickをCommitせず、Internal Substepを増やしてTick開始SnapshotからRetryする。Retry上限到達時はBlockedByCCD / Failed。

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
- Fixed Delta Timeの変更はPaused、in-flight=0、pending whole ticks=0、pending Step request=0でのみ許可する。
- fractional remainderは秒単位の実時間として保持し、新しいdtでwhole tick数を再計算する。
- 変更時はCloth / Contact / Animation XPBD LambdaとTick ScratchをResetする。旧dtのwhole tickを新dtで再解釈しない。
- Step要求はPause / Stop中に1 Fixed Tickだけ要求する。未実行Step要求はQueueで管理し、Stop / Resetの明示Policyに従う。
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

Render Candidate SlotはTickごとにReservationする。SlotがなければTickをSubmissionせずAccumulatorに時間を保持する。

各TickはSolver全Substep後に専用Render Candidate SlotへPosition / Normal / Tangentを生成し、Binding / Geometry / NonFinite Validationを実行する。すべてのPhysics GateとRender Candidate Validationが通過した場合だけ、GPU Commit ControlがCommitted Physics State Set / GenerationとRender Slot ID / Generationを同一Controlで切り替える。Reject時は両方を維持し、Working State / Candidate SlotをHostへ公開しない。

Render Candidate Ring:
- Slot数 = world_desc.maximum_in_flight_ticks + 1。
- 既定in-flight 2の場合は3 Slots。うち1つはCPU-observedの直前Committed Slot、最大2つはin-flight Candidateとして使う。
- Query / Status未回収、in-flight Tick、Host Leaseが存在するSlotは上書きしない。
- get_render_bindingはEvent Query完了、COMMITTED Status Readback、連続Tick Prefixを確認した最新CPU-observed Render Slotだけを返す。
- WorldあたりHost Render Leaseは同時に1つ。各MeshのBindingは同じLease Generationに属する。
- HostはDraw後、Position / Normal / Tangent SRVをVS / HS / DS / GS / PS / CS全StageからUnbindし、release_render_bindings(world)を呼ぶ。
- Release後は該当Pointerを使わない。COM AddRefはCOM objectの生存期間を延ばすだけで、Slot内容のLeaseは延長しない。
- Render Lease未解放で次Submissionを要求した場合はBUSY / render_binding_busyでBackpressureし、Accumulator時間を保持する。
- Copy / Publish Pass / Render Candidate Validation / Ring Slot VRAM・GPU時間をPerformance Runへ含める。

この方式はCPU-observed commitとRendererのBuffer内容を一致させるためのv1契約であり、GPUでCommit済みでもCPUが観測していない新しいRender Slotを直接公開しない。

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
