# Skin Physics GPU Runtime / Library Integration Design

    #include <cstdint>
    #include <string>
    #include <d3d11.h>
    #include "skin_physics/asset/types.hpp"

- 文書バージョン: 1.0
- 状態: DESIGN REVISION / READY FOR INDEPENDENT REVIEW（承認前）
- 対応要件: requirements.md 1.1
- 対応基本設計: basic_design.md 1.2
- 対応詳細設計: detailed_design.md 1.3
- 実装基点: feature/phase1-importer
- 対象ビルド: SkinPhysics.sln / SkinPhysics.vcxproj（x64、C++17、D3D11）

## 1. 目的と設計上の位置付け

本書は、既存のSkin Physics設計を実際のGPUランタイムへ接続するため、Library境界、Simulation World、非同期GPU Step、状態Commit / Rollback、Render Binding、資源所有権、エラー処理を定義する。

Skin PhysicsはSoft BodyとClothを同一Simulation Worldで扱うC++静的ライブラリである。Soft Bodyの四面体Corotational FEM、ClothのXPBD、双方向VT / EE接触、自己衝突、アニメーション追従を統合する。物理計算とRender FeedbackはGPU上で実行する。

この設計は既存のSoft Body単体デモ向け仕様をそのまま移植しない。Skin Physicsの要件と既存設計を維持しながら、再利用可能なGPU Runtime境界と検証可能なStep transactionを追加する。

**本書の状態は独立レビュー待ちである。** API例とPass接続は実装契約案であり、Production Runtimeが完成済みであることを示さない。

## 2. v1の機能範囲

### 2.1 World構成

1つのSimulation Worldに以下を登録できる。

- 1つ以上のCharacter Model Instance。
- 各Model Instance内の複数Render Mesh / Physics Mesh。
- Mesh単位のphysics_mode: none / soft_body / cloth。
- Soft BodyとClothの混在。
- Static / Kinematic Triangle Mesh Collider。
- Mesh単位のSelf Collision設定とMesh Pair除外設定。

同一Model内のBody、Shirt、Pants、Hair Cards / Hair Bundles等は共通Worldで接触できる。Soft BodyはPhysics Mesh境界面、ClothはPhysics表面三角形を接触対象とする。Soft Body内部Tet面は外部接触プリミティブへ登録しない。

v1では独立した動的Rigid Body、Strand Hair専用線状モデル、GPU上での四面体化、GPU上での任意Topology編集、変形Collision ProxyからPhysics Meshへの汎用応答写像を対象外とする。

### 2.2 物理方式の境界

| 対象 | v1の基準方式 | GPU状態 |
|---|---|---|
| Soft Body | Tetrahedral Corotational Linear Elasticity + semi-implicit Euler | Position / Velocity / Element diagnostics |
| Cloth Stretch | XPBD distance constraint + Graph Coloring | Position / Lambda |
| Cloth Shear | XPBD strain constraint + Gather Jacobi | Position / Lambda |
| Cloth Bend | XPBD signed dihedral constraint + Gather Jacobi | Position / Lambda |
| Animation Tracking | Soft BodyはKelvin-Voigt型追従力、ClothはXPBD attachment | Target / Velocity / Lambda |
| Contact | 双方向VT / EEのXPBD unilateral constraint | Contact / Lambda |
| Friction / Restitution | 接触後のVelocity impulse | Velocity / Impact state |

FEMの応力・内部力とXPBDのLagrange multiplierを同じ状態変数・同じ更新式として扱わない。Soft BodyのCorotationalモデルはv1の比較対象基準として保持し、大変形条件で適用範囲を検証する。Neo-Hookean FEMは将来比較候補であり、既存Corotational実装を置き換える前提にしない。

### 2.3 性能目標の解釈

主評価GPUはGTX 1660 SUPER 6GB。要件上の3～40 FPSは開発時の評価範囲であり、全Asset規模に対する固定保証値ではない。SoftBodyDemoの200,000粒子 / 40 FPS必須、約1,000,000 Tet、1/120秒固定はSkin Physicsの要件へ転用しない。

v1のPerformance Profileは2k / 8k / 32k Tet、2k / 8k / 32k Cloth Trianglesと、接触密度・Mesh Pair数・Substep数・Solver Iterationsを組み合わせて評価する。GPUメモリはOS / Rendererの利用分とDXGI Budgetを考慮し、6GB全量をLibraryが使える前提にしない。3～40 FPSの記録と品質指標を必ず同時に保存する。

## 3. ビルド・Library境界

### 3.1 配布形態

既存のVisual Studio Target `SkinPhysics` をv1の静的Library（.lib）として提供する。ビルドの正規手段は既存の `SkinPhysics.sln` / `SkinPhysics.vcxproj` とし、CMakeを必須化しない。

| コンポーネント | 責務 | 実行場所 |
|---|---|---|
| SkinPhysics Public API | Runtime / World / Model / ColliderのHandle、状態、エラー | Hostから呼び出す |
| Asset Importer / Builder | OBJ / glTF、Physics Mesh生成、Skin Weight、Cache | CPU |
| Simulation Manager | Fixed Step、World更新、GPU Submission、Query / Status | CPU制御 |
| GPU Runtime | FEM、Cloth XPBD、Broadphase、Narrowphase、CCD、Contact、Render Feedback | D3D11 Compute Shader |
| Animation Interface | Clip評価、Bone Pose、目標位置 / 速度 | CPU評価 + GPU Skinning |
| Test / Validation | CPU Reference、解析Fixture、WARP、GPU Profiler | 開発・CI |

fTetWild等の四面体化器はOffline Asset Buildの任意ツールであり、SkinPhysics Runtimeのリンク必須依存にしない。外部物理ライブラリをRuntimeのソルバーとして使用しない。CPU Referenceは検証専用で、GPU障害時の物理Fallbackに使わない。

### 3.2 D3D11 Device / Context所有権

Host ApplicationがHardware D3D11 DeviceとImmediate Contextを生成する。SkinPhysics Runtimeは初期化時に受け取ったInterfaceをCOM AddRefして保持し、GPU Buffer / SRV / UAV / Shader / Queryを生成する。Device / Contextの最終破棄責任はHostに残す。

- Feature Level 11_0以上、Compute Shader 5.0を確認する。
- RuntimeとRendererは同一Device / Immediate Contextを共有する。
- Immediate Contextへの呼び出しは初期化時に登録した単一Submission Threadへ直列化する。
- Deferred Context、同一Contextへの複数Thread同時呼び出し、独立したPhysics / Render GPU Queueを仮定しない。
- RuntimeはGPU呼び出し前にDevice Removed状態を検出し、Device喪失後の新規Dispatchを拒否する。
- RuntimeはHostのGraphics Stateを暗黙復元しない。Host RendererはRuntimeが公開するRender Bindingを次回Compute使用前にUnbindする。

### 3.3 Public APIの基本形

以下はPublic APIの責務を固定する概念ヘッダーであり、実装済み宣言ではない。

    namespace skin_physics::runtime
    {
        enum class status_code : uint32_t
        {
            ok,
            not_ready,
            invalid_argument,
            invalid_state,
            busy,
            stale_handle,
            unsupported_feature,
            asset_invalid,
            capacity_exceeded,
            out_of_memory,
            device_removed,
            gpu_execution_failed,
            step_rejected
        };

        enum class playback_state : uint8_t
        {
            stopped,
            playing,
            paused
        };

        struct status
        {
            status_code code;
            int32_t native_code;
            std::string diagnostic;
        };

        struct world_handle { uint32_t index; uint32_t generation; };
        struct model_handle { uint32_t index; uint32_t generation; };
        struct collider_handle { uint32_t index; uint32_t generation; };

        struct runtime_desc;
        struct world_desc;
        struct model_desc;
        struct collider_desc;
        struct animation_playback_desc;
        struct transform;
        struct force_desc;
        struct impulse_desc;
        struct advance_report;
        struct world_status;
        struct render_binding;

        class physics_runtime
        {
        public:
            status initialize(
                ID3D11Device* device,
                ID3D11DeviceContext* immediate_context,
                const runtime_desc& desc);

            status create_world(
                const world_desc& desc,
                world_handle* out_world);

            status add_model(
                world_handle world,
                const model_desc& desc,
                model_handle* out_model);

            status create_static_collider(
                world_handle world,
                const collider_desc& desc,
                collider_handle* out_collider);

            status create_kinematic_collider(
                world_handle world,
                const collider_desc& desc,
                collider_handle* out_collider);

            status destroy_model(
                world_handle world,
                model_handle model);

            status destroy_collider(
                world_handle world,
                collider_handle collider);

            status set_playback_state(
                world_handle world,
                playback_state state);

            status set_fixed_delta_time(
                world_handle world,
                double fixed_delta_time);

            status set_time_scale(
                world_handle world,
                double time_scale);

            status set_gravity(
                world_handle world,
                const asset::vec3& gravity);

            status enqueue_force(
                world_handle world,
                const force_desc& force);

            status enqueue_impulse(
                world_handle world,
                const impulse_desc& impulse);

            status set_animation_playback(
                world_handle world,
                model_handle model,
                const animation_playback_desc& desc);

            status set_kinematic_pose_sample(
                world_handle world,
                collider_handle collider,
                double sample_time,
                const transform& pose);

            status advance(
                world_handle world,
                double frame_delta_time,
                advance_report* out_report);

            status poll(
                world_handle world,
                world_status* out_status);

            status single_step(
                world_handle world,
                advance_report* out_report);

            status get_render_binding(
                world_handle world,
                model_handle model,
                uint32_t mesh_index,
                render_binding* out_binding);

            status release_render_bindings(world_handle world);
            status reset_world(world_handle world);
            status destroy_world(world_handle world);
            void shutdown() noexcept;
        };
    }

Handleはindex + generationで識別し、Destroy後のHandle再利用を拒否する。APIからGPU内部Buffer、Solver Scratch、Writable UAVを公開しない。Statusのdiagnosticは呼び出し結果が保持するstd::stringとし、Runtime内部の一時文字列ポインタを返さない。Public APIはC++静的Library向けであり、異なるCompiler / CRT間の安定ABIは保証しない。Asset / DescriptorはVersioned Schemaを使い、ロード時に既存Asset Cacheのhash・coordinate_system_version・material_layout_version・generator identityを検証する。

### 3.4 WorldとMesh Instanceの配置

Worldは複数Model InstanceとColliderを管理する。各Model Instanceは複数Mesh Componentを持ち、各Mesh Componentにphysics_mode、Collision Filter、Material、Attribute Channelを割り当てる。

Runtime内部ではMesh単位の局所Indexを保持したまま、World単位のGlobal Vertex / Primitive IDとBuffer Rangeへ変換する。

- Global Vertex IDはWorld内で一意。
- Mesh ID / Model ID / Primitive IDは安定した識別子としてContact KeyとDiagnosticへ含める。
- Soft Body Tet Index、Cloth Triangle Index、Boundary Face Indexは同一配列へ混同して格納しない。
- 異なるMesh間の接触応答はWorld Global Vertex Rangeへ戻す。
- 同一Mesh内のSelf CollisionとMesh Pair除外はBroadphase前のFilterで適用する。
- Render Mesh / Physics Mesh / Collision-only Meshの頂点数・Index・属性を相互に流用しない。

## 4. CPU / GPUの責務

### 4.1 CPU側

- Import / Asset Validation / Physics Mesh Build / Cacheの生成・ロード。
- Animation Clip評価、Bone Hierarchy評価、Bone Matrix生成。
- Simulation World / Mesh / Colliderの管理と設定更新。
- Fixed Step Accumulator、Step Submission、GPU Query / Statusの非同期Poll。
- GPU Resource生成・拡張・破棄、Asset / Scenario / Run Result JSON。
- 小規模Diagnostic / Counter / Error StatusのReadback。
- Device Lost検出とWorld再初期化。

CPUは通常フレームでPhysics Vertex位置・速度、Tet Lambda、Contact List全体を同期Readbackしない。CPU Referenceは同じアルゴリズム・同じ更新順序の検証専用とし、Runtimeの物理計算を肩代わりしない。

### 4.2 GPU側

- Physics Mesh Skinning Target / Animation Velocity生成。
- Soft BodyのF、Polar Rotation、Stress、Tet Force Contribution。
- Tet→Vertex Force GatherとSoft Body Integration。
- Cloth Stretch / Shear / Bend / Animation Tracking XPBD。
- Swept Broadphase、Spatial Hash、Candidate Pair生成・重複排除。
- VT / EE Narrowphase、Contact Patch、双方向Contact XPBD。
- CCD / TOI Reduction / TOI Substep Split / Impact / Friction / Restitution。
- 状態検証・Tick Commit / Reject。
- Render Vertex Position / Normal / Tangent Feedback。
- GPU Timestamp対象の全Physics / Render Feedback Pass。

### 4.3 CPU Referenceと既存同期Pipelineの位置付け

現在の `fixed_step_controller` は同期Callbackの成功をStep完了として扱うCPU側制御器である。GPU Dispatch受付とGPU Logical Tick Commitは別イベントなので、このクラスをProduction GPU Runtimeの完了判定に直接使わない。既存の `simulation_step_pipeline` はCPU上のWorking Copyを使う同期Transactionであり、CPU Reference / Test Harnessとして保持する。Production GPU RuntimeはGPU Commit Status、D3D11 Event Query、非同期Status Readbackを扱う専用Schedulerを実装する。

`d3d11_compute_graph` は順序付きPassのBinding / Dispatch / Unbindを行うExecutorとして扱う。依存関係を自動推論したり、GPU Physics Worldの状態遷移やTick Commitを代行したりしない。Pass順序・Resource ABI・Dispatch Argument・Commit制御はSimulation Manager / Solverが明示する。

## 5. 時間管理とGPU Tick Scheduler

### 5.1 時間モデル

- Fixed Delta TimeはTest Environment設定で変更可能。初期値は既存 `fixed_step_settings` の1/60秒を維持する。
- Time ScaleはFrame elapsedをAccumulatorへ加える倍率とする。物理のFixed Delta Timeそのものを変化させない。
- Soft Bodyの内部力積分にはCFL / Animation Tracking安定条件から求めたInternal Substepを使う。
- Cloth XPBD / Contact XPBDのComplianceには実際に解決するSubstep Delta Timeを使う。
- Fixed Delta Timeを変更する場合、WorldをPausedにし、in-flight Tickが0件で、未処理のwhole Tick / pending Step要求がないことを要求する。条件を満たさない場合はBUSYを返す。条件成立後、Accumulator fractional remainderの処理（Resetまたは明示的に保持可能なfractional timeのみ保持）を適用し、XPBD LambdaとStep内ScratchをResetしてから新しい時間刻みへ移行する。旧Fixed Delta Timeのwhole Tickを新しい時間刻みで実行しない。
- 最大Step / Substep予算に達してもAccumulator内の未処理時間を黙って破棄しない。

### 5.2 Fixed Stepと非同期GPU Stepの分離

CPU側のFixed Step制御はWall Clockから必要Logical Tick数を算出する。GPU Runtimeは各Logical TickをGPU CommandとしてSubmissionし、GPU上でCommit / Rejectを決定する。

管理する時刻は次の3つに分ける。

1. requested_simulation_time: Time Scale適用後、HostがRuntimeへ供給した時間。
2. gpu_committed_simulation_time: GPUのTick Commit Controlが連続成功したLogical Tickの時間。
3. cpu_observed_completed_time: 対応Event Query完了とCOMMITTED Status ReadbackをCPUが確認した連続Tick Prefixの時間。

`advance`が成功を返すことは、GPU TickがCOMMITTEDになったことを意味しない。Host側のUIは必要に応じてsubmitted / committed / observedを分けて表示する。

### 5.3 Submission / Backlog

- D3D11 Immediate Contextは単一の順序付きCommand Streamである。PhysicsとRenderが独立GPU Queueで同時に動く前提を置かない。
- 1 PhysicsSubmissionBatchは1 Logical Tickを含む。
- 初期の上限はin-flight logical tick 2件とし、Status Query Ringはその上限以上のSlotを持つ。上限は設定で変更できるが、未回収Slotを上書きしない。
- 各TickのDispatch Planはexpected_predecessor_tick_sequence、expected_source_state_generation、expected_source_state_set、dispatch_idを持つ不変Recordとする。
- Tick NのCommit Controlは成功時だけcommitted_tick_sequenceとcommitted_state_generation / active_state_setを更新する。Reject / BlockではCommitted Stateの参照先を変更しない。
- Tick N+1以降の先行Submissionは、tick_begin_controlでexpected predecessor sequenceとstate generationが一致することを検証する。すべてのBudgeted Physics Passは最初に同じTick Plan / scheduler gateを確認し、前提不一致・先行Tick未Commit・terminal failure時はPhysics Bufferの読み込み・書き込み・Atomicを行わず終了する。
- 先行TickがRejectされた場合、既にSubmissionされた後続TickはBLOCKED_BY_PREDECESSORとなり、物理状態を変更しない。先行TickのRetryがGPU上でCommitされた後、CPUは後続TickのPlanを新しいCommitted State Generationに合わせて再発行する。
- Planのsource / destination State SetはSubmission時点の予測Commit順序に基づいて固定する。GPU Commitが成立しない場合の後続Planは必ずgateで停止する。GPUが成功した場合のみ計画されたState Set遷移が有効になる。
- 上限到達中は新しいPhysics Dispatchを発行せず、Fixed Step Accumulatorに時間を残す。
- RenderをSkip / ThrottleしてPhysics Submissionを優先することは許可する。
- Step数、Substep数、Iteration数、Contact候補、時間を性能維持のために黙って減らさない。
- Accumulatorが長時間増加する場合はsimulation_backlogとして警告・計測する。Skin Physicsの要件にない100ms Failureや40FPS必須条件を追加しない。
- Pauseは新規Logical Tick生成を止め、in-flight GPU Tickは安全に完了させる。Pause時間をactive elapsedへ加算しない。
- StepはPause / Stop中に固定時間刻みを正確に1回だけ要求する。GPU Capacity / CCD Block中は実行待ちとする。

初期in-flight上限2は資源・Query管理を単純化する初期値であり、2件が最適性能を保証する値ではない。1 / 2 / 4件の比較とQuery Ringの遅延試験で確定する。

### 5.4 Animation / Kinematic Inputの時間整合

Animation ClipはRuntimeがSimulation Timeを基準にCPU評価し、各Logical TickおよびInternal SubstepでBone Pose / Skin Matrixを供給する。Render FrameのPoseを複数Physics Tickに無条件複製しない。CUBICSPLINE / LINEAR / STEPはImporterで定義された補間方式を維持する。

外部からBone Poseを直接供給するHostでは、時刻付きPose Sampleを登録し、各Tick時刻を補間できるだけの履歴を保持する。Pose履歴不足時は直前Poseを不明な速度で延長せず、Diagnosticへ記録する。

Kinematic Colliderは時刻付きTransform SampleからLogical Tick / Substep境界の姿勢を補間する。Teleportは明示Eventとし、Contact / CCD Cache、前回位置、速度を規定ポリシーで再初期化する。Collider自身は位置補正・Impulseで変形しない。

## 6. GPU Tick Transaction / Commit / Rollback

### 6.1 State Bufferの分離

少なくとも次を分ける。

- committed_state_set: 最後にCommit済みのPosition / Velocity / Solver Persistent Stateと、そのstate_generation / active_state_set。
- working_state_set: 現在解いているLogical Tick / Substepの作業状態。Commit前にCommitted Stateを上書きしない。
- tick_start_snapshot: Capacity / CCD / NonFinite failureから再実行するLogical Tick開始状態。Position / Velocityだけでなく、そのTickのRetryに必要なPersistent Lambda / Contact / Animation Historyも対象とする。
- tick_dispatch_plan: expected_predecessor_tick_sequence、expected_source_state_generation、expected_source_state_set、dispatch_id / tokenを持ち、ReservationからExecution / Commitまでimmutableとする。
- solver_lambda / contact_lambda / impact_state: 各Solverの更新規則に従うWorking / Persistent State。Rollback時はtick_start_snapshotと対応するLambda / Contact Cacheを復元する。
- render_position / render_normal / render_tangent: Commit後の描画公開用状態。
- simulation_status / tick_sequence / state_generation / diagnostic_counters: GPU Commit / Reject / Error Control。

GPU Solver ScratchをHost Render Bindingへ直接公開しない。Commit済み状態と描画用BufferはCopyまたはPublish Passで同期する。Committed StateのResource Set切替はGPU Commit Controlだけが行う。Retry / Rejectではactive_state_setとstate_generationを維持する。後続Tickは前TickのCommit結果とGenerationを検証してから実行する。

### 6.2 GPU Commit Control

Logical Tickは次の条件を全て満たした場合だけCOMMITTEDとする。

- すべてのSubstepが正常に完了。
- Broadphase / Candidate / Contact / Contributionの全Capacityが有効。
- CCDが全必須候補でResolvedまたはNo Hitに確定し、Unresolvedが0。
- Position / Velocity / Stress / Lambda / TOI等にNaN / Infがない。
- Tet Inversion / Volume / Contact Residual / Initial Repair等のScenario Gateを通過。
- Render Feedback用の対応付けとGeometry更新に致命的エラーがない。
- GPU DiagnosticのTerminal Failureが0。

判定はGPU側のReduce / Commit Controlで行う。CPU Readbackの完了をTick Commitの条件にしない。CPUのcompleted_timeはQuery完了後の非同期Status Readbackを確認してから進める。

### 6.3 Rollback / Retry

Capacity Overflow、CCD Unresolved、NonFinite、物理品質Gate失敗時は、現在のLogical TickをREJECTED / BLOCKEDにする。作業状態をCommitしない。GPU上のtick_start_snapshotから再実行する。

- Capacity Overflow: Count / Scan / Capacity Checkで必要数を記録し、Fill / Consumerの範囲外アクセスを禁止する。Event Query完了後にCPUが小規模Required Capacityを回収し、Submission ThreadでBufferを再確保する。入力と初期状態を変更せず同じTickを再実行する。
- CCD Unresolved: Internal Substep数またはCCD分割上限を上げて同じTickを再実行する。No Hitへ置き換えない。
- NonFinite / Invalid Asset / ABI mismatch: 自動的な低品質再試行をしない。Failureを報告する。
- Retry上限到達: TickをCommitせず、Simulation WorldをBlockedByCCD / BlockedByCapacity / Failedへ遷移する。

CPU Physics Fallback、Contact候補のDrop、時間のDrop / Clampは禁止する。GPU全状態をCPUへ戻してRollbackする方式にしない。

### 6.4 GPU / CPU完了時刻の整合

各Tickの最後にGPU Status BufferをStagingへCopyし、同じImmediate Context Command StreamへEvent Queryを発行する。QueryのGetDataはDO_NOT_FLUSHでnon-blocking Pollし、Query完了後にStaging BufferをDO_NOT_WAITでMapする。未完了なら前回Statusを維持する。

CPU completed_timeは、(1) Event Query完了、(2) Status=COMMITTED、(3) 先行Tickがすべて連続COMMITTED、を満たす最後のTickまで進める。Query完了順がTick Sequence順と異なっても、CPU observed completed prefixは連続したCOMMITTED Tickだけから構成する。Event QueryだけでTick成功とみなさない。通常FrameではParticle / Contact / Lambda全体をReadbackしない。

### 6.5 複数in-flight Tickの依存関係

Immediate ContextのCommand Streamは順序付きだが、CPUは最大2件のPhysicsSubmissionBatchをGPU完了前に発行できる。後続Tickの安全性はCPUのQuery Pollタイミングに依存させない。

- BatchはTick Begin時にImmutable Dispatch Planを検査する。
- Tick NがGPU Commit Controlで成功した場合だけ、Committed Sequence / State Generation / Active State Setを更新する。
- Tick N+1の先行Submissionは、NのCommit Controlより後に同一Command Streamへ発行する。Nが成功していれば計画済みGenerationが一致し、NのCommit済み状態を入力として処理できる。
- NがReject / Blockされた場合、N+1のTick Begin GateはGeneration / predecessor mismatchを検出し、Physics StateのRead / Write前に後続処理を抑止する。後続TickはBLOCKED_BY_PREDECESSORとなる。
- CPUがNの非同期Statusを確認し、NをRetryしてGPU Commitした後、後続Tickを新しいState Generation / predecessor IDで再Submissionする。
- どのBudgeted Physics Passもtick plan gateを迂回できない。Scheduler Control / Diagnostic PassだけがGate対象外である。
- Query RingはStatus CopyとEvent Queryの両方をTick IDで対応付け、遅延・順序逆転・Slot再利用を検出する。

これにより、先行Tickの失敗後に後続Tickが未確定Working Stateを入力としてCommitする経路を禁止する。実際のState Set数、GPU Copy / Ping-Pong構成、Shader Binding TableはPhase 12実装前にResource ABI Reviewで確定し、Rollback SnapshotとRender Publishを含むVRAM / GPU時間を測定する。

## 7. GPU Pass / Dataflow Integration

### 7.1 World Logical Tick

1. CPU Animation Evaluation / Kinematic Transform Sampleの取得。
2. GPU Animation Target / Velocity Skinning。
3. Logical Tick開始SnapshotとTerminal Status初期化。
4. 各Internal Substepについて次を処理。
   - Previous State Snapshot。
   - Soft Body Tet Deformation Gradient / Polar Rotation / Stress / Inversion Penalty。
   - Tet Force Contribution。
   - Vertex→Tet CSR Force Gather（float Atomic Add禁止）。
   - Soft Body external / animation forceとSemi-implicit Euler Prediction。
   - Cloth Prediction。
   - Swept AABB / Broadphase Count / Scan / Capacity Check / Fill。
   - Spatial Hash Sort / Group / Pair Generation / Dedup。
   - VT / EE Narrowphase、Contact Patch Reduction。
   - CCD候補CA / GPU Minimum TOI Reduction / 必要なTOI分割。
   - Cloth Stretch / Shear / Bend / Animation TrackingとContact Position XPBD。
   - Velocity Reconstruction / Impact / Friction / Restitution。
   - Broadphase Envelopeと残存Penetrationの再検査。Envelope外ならRebuild。
   - Substep状態の確認。
5. 全Substep成功後にRender Position / Normal / Tangent Feedback。
6. GPU Logical Tick Commit Control。
7. GPU Status / Diagnostic Copy、Timestamp / Event Query。

Soft Body内部力はInternal Substepごとに再評価する。Soft BodyのFEM ForceをXPBD Contact Lambdaへ混ぜない。Contactは双方のPhysics Vertexへ質量・勾配係数に基づいて補正する。

### 7.2 CCD / TOI

既存のPoint-Triangle CCDは初期Fixtureとして維持する。Production Runtimeでは変形Mesh間の高速接触を満たすため、VTとEEの双方に対してSwept BroadphaseとConservative Advancementを接続する。Triangle-Triangleの交差候補はVT / EEイベントへ分解して扱い、Primitive Pair / Contact Keyの重複を除去する。

TOIは秒単位で管理する。Positive-Time TOIが得られた場合はSubstep開始Snapshotへ戻し、TOIまで再積分し、接触Position / Velocity応答を一度だけ適用して残時間を再積分する。TOI後の速度をStep終端のVelocity Reconstructionで上書きしない。複数接触は同時TOI許容幅でグループ化し、同一Eventを二重Impulseしない。Unresolved、進行不能、Iteration上限は未解決のままCommitしない。

### 7.3 D3D11 Compute Pass Executor

既存Compute Pass Graphは明示Pass順を実行するExecutorとして活用する。Production Solverは次を保証する。

- PassごとのSRV / UAV / Constant Buffer Binding Table。
- 同一ResourceのSRV / UAV同時Bind禁止。
- CS UAVは8 Slot以下。
- Dispatch Thread Group間のGlobal Barrierを仮定しない。依存段階は別Dispatchとする。
- GPU CountからConsumer Dispatchを決める場合はGPU生成Indirect Argumentを使い、CPU ReadbackでGroup Countを求めない。
- DispatchIndirect ArgsのUAV書き込み後はUAV UnbindしてからDispatchIndirectする。
- Thread Group SizeはShader Reflection由来のnumthreadsとDispatch Argument Builderで一致確認する。
- Group Size 64 / 128 / 256はPhase Benchmarkで比較し、未検証の固定値を前提にしない。
- GPU Pass GraphへDependency / Resource Lifetime情報を追加する場合も、Pass順を暗黙推論せず明示仕様を保つ。

### 7.4 Force Gather / XPBD Gather

Soft Body Tet ForceはTet単位で4頂点のContributionを出力し、Vertex→Tet CSRを走査するVertex単位Gatherで総和する。寄与数で割って平均化しない。

Cloth XPBDは全Constraintが同一position snapshotを参照し、ConstraintごとにLambdaと適用寄与を出力する。Vertex→Constraint CSRでGatherする。Jacobi緩和はProjected Lambda Deltaに適用し、その同一DeltaでLambda / Positionを更新する。Coloringは同色内のVertex重複がないことをAsset Build時に検証する。

## 8. Capacity / Memory / Error Contract

### 8.1 Capacity

可変長GPU出力はCount → Prefix Sum → Capacity Check → Fillを使用する。候補、Cell Record、Contact、Contribution、TOI、Impact Event等に次を持つ。

- current_capacity
- required_count
- overflow_flag
- producer / consumer dispatch identity
- logical_tick_sequence
- validation / error code

Count / Scanで必要数がCapacityを超える場合Fillを停止する。範囲外UAV Write、無言の候補切捨て、上限越えCountのwraparoundを禁止する。GPU uint32 Countの加算はOverflow検知を行い、UINT_MAXへwrapした値を有効Countとして使わない。

### 8.2 Memory Budget

Library所有Buffer / TextureのByte数とDXGI Local Memory CurrentUsage / Budgetを別々に計測する。使用可能BudgetはOS / Renderer等の他利用者を含む実測値から設定し、6GB全量を占有しない。Capacity GrowthはMemory Budgetを超えない範囲で行い、必要容量を確保できなければWorldをBlockedByCapacityにする。

### 8.3 Error分類

| Error | Runtime動作 |
|---|---|
| InvalidAsset / InvalidTopology | Asset Load拒否 |
| DegenerateTetrahedron / InvertedTetrahedron | Asset / Step Validationに基づき警告またはReject |
| InvalidMaterialParameter | 設定変更拒否 |
| SkinWeightTransferError / RenderBindingError | 該当MeshのLoad / FeedbackをReject |
| BroadphaseOverflow / ContactOverflow / ContributionOverflow | TickをRollbackし、Capacity調整後に再実行 |
| CCDIterationLimit / CCDUnresolved | TickをRollbackし、Substep / CCD設定変更後に再実行 |
| InitialPenetrationLimit | Initial StateをCommitしない |
| NonFiniteState | TickをReject。自動Fallbackしない |
| GpuResourceAllocationFailure | Runtime / WorldをFailedまたはBlockedByCapacityへ |
| DeviceRemoved | 新規Dispatch停止。Asset / Initial Stateから再初期化が必要 |
| WorkBudgetExceeded | TickをRejectし、原因カウンタを報告 |

警告・エラーはModel ID、Mesh ID、Primitive ID、Logical Tick、Substep、Pass、Counterを含む。Diagnosticsは小規模かつ非同期で回収する。

## 9. D3D11 Render Resource Interop

### 9.1 HostとRuntimeのResource所有権

| Resource | Owner | Host利用 |
|---|---|---|
| ID3D11Device / Immediate Context | Host | RuntimeへBorrowed Binding |
| Physics Vertex / Tet / Constraint / Contact / Scratch | SkinPhysics | 非公開 |
| Timestamp / Event Query / Status Staging Ring | SkinPhysics | 非公開 |
| Committed Render Position SRV | SkinPhysics | Read-only Shader Resource |
| Committed Render Normal / Tangent SRV | SkinPhysics | Read-only Shader Resource |
| Immutable Render Index / Material / UV / Color | HostまたはAsset | Host Rendererが管理 |

### 9.2 Render Binding

Render BindingはPosition / Normal / TangentのSRV、Format、Stride、Vertex Count、Index Count、Resource Generationを返す。Borrowed COM Pointerとし、長期保持する場合だけHostがAddRefする。HostはSRVをVS / PS等へBindできるが、UAVとして書き込まない。

get_render_bindingはCPUがEvent Query完了とCOMMITTED Status Readbackを確認した、最後のCPU-observed committed state generationを返す。GPU上でCOMMITTEDになっていてもCPU-observed prefixへまだ入っていないTickの状態は返さない。このためRenderは1 Query Poll周期以上遅れる場合があるが、未確認Working Stateや寿命未確定Resourceを公開しない。

Host Rendererは描画完了後、次のRuntime Compute Dispatch前に公開SRVを全Graphics StageからUnbindする。Runtimeは公開Render Bufferへ直接Solver書き込みを行わず、Commit後のPublish Pass / Copyで更新する。Publish CostとVRAMはPerformance Runへ含める。

### 9.3 Device Loss

Device Removed / Reset時は新規Submissionを停止し、WorldをDEVICE_LOSTへ遷移させる。GPUが唯一の物理Stateであり、毎Frame全状態Readbackをしないため、Device喪失直前の任意Stateを完全復元する保証はv1に設けない。Asset CacheとInitial Stateから再生成してSimulation TimeをResetする。継続復旧が必要な場合は明示Checkpoint機能を別設計する。

## 10. Test / Validation Gates

既存T01～T28を維持し、CPU Reference / Analytical / WARPの各レイヤーを次の順で接続する。

### Gate R0 — Runtime Boundary / Async Scheduler

- Host-owned Device / Immediate Contextと単一Submission Thread。
- API Handle Generation、World / Model / Collider lifecycle。
- GPU SubmissionとGPU Commitの区別。
- Event Query + asynchronous COMMITTED Status Readback + contiguous Tick Prefix。
- Accumulator backlog、Pause / Stop / Step / Reset、Time Scale。
- GPU Capacity Block / Retry、同じInitial Snapshotからの再実行。

### Gate R1 — Soft Body GPU Integration

- Corotational Tet Evaluation、Rotation / SVD fallback、Stress / Force Contribution。
- Vertex→Tet CSR Force Gather、Mass Integration、CFL / Internal Substep。
- CPU Double OracleとのLocal / Whole-Step比較。
- Tet Rest / Rigid Rotation / Affine Compression / Shear / Inversion fixture。

### Gate R2 — Cloth GPU Iteration

- Stretch Coloring、Shear / Bend Gather Jacobi。
- 同一Snapshot、Projected Lambda Delta、Lambda / Position一致。
- Iteration / Substep Lambda lifecycleとCompliance actual-dt。
- CPU Referenceとの位置・速度・Lambda比較。

### Gate R3 — Mixed Collision Runtime

- GPU Spatial Hash Build / Rebuild / Pair Dedup。
- 全有効Mesh PairとSelf Collision。
- VT / EE Narrowphase、双方補正、Contact Patch。
- Contact Candidate集合が小規模全探索の確定候補を全て含む。
- Capacity OverflowのRollback / Retry。

### Gate R4 — CCD / Initial Repair

- Moving VT / EE Conservative Advancement。
- 同時TOI、Positive-Time TOI、初期接触、退化・進行不能。
- TOI impulse一度だけ適用、残時間再積分。
- Initial Repair per-vertex budgetと未修復Error。

### Gate R5 — Animation / Render Integration

- Multi-key glTF Clip / Skeleton / Mesh Node Matrix。
- Substepped target position / velocity。
- Animation Discontinuity / Teleport Policy。
- GPU Render FeedbackとUV / Color / Material / Index維持。
- Render Binding Unbind / Resource Generation / Device Loss。

### Gate R6 — Physical GPU Performance / Memory

- GTX 1660 SUPER上で2k / 8k / 32k Tet、2k / 8k / 32k Clothの組合せを計測。
- Pass別GPU時間、CPU Submission時間、Frame時間、GPU Memory、Candidate / Contact / CCD / Retry数を別々に保存。
- WARP時間を物理GPU性能値として使用しない。
- 3～40 FPSは開発評価値として報告し、未指定Workloadの固定合否値に変えない。
- 性能を理由に品質・Collision Pair・Step Time・Iterationを黙って減らさない。

## 11. 未決定事項と実装前の確認

以下は実装前レビューまたは該当PhaseのFixture測定で最終値を確定する。

1. Public API具体型、Asset / Runtime DescriptorのABIとDiagnostic文字列Lifetime。
2. Fixed Stepのin-flight tick上限（1 / 2 / 4）とQuery Ring容量の最終値。
3. Kinematic Transform Sample履歴の保持時間・不足時Diagnostic。
4. GPU Tick Snapshot / Working Bufferの具体的Byte数とRollback方式。
5. Production BroadphaseのGPU Build / Sort / Pair DedupのBuffer ABI。
6. VT / EE CCDのSDFやTriangle distance近似精度、同時TOI tolerance。
7. FEM / Cloth / Contactの代表Material PresetとCFL / Substep上限。
8. Render Publish BufferのCopy / Compute方式と実測コスト。
9. Device Loss時Checkpointの将来要件。
10. Physical GPU上のNumerical / Performance Baseline。

この残項目は設計境界や失敗時契約を未定義にするものではない。アルゴリズム・API・ライフサイクルは本書で固定し、残る値はGate R0-R6で測定・校正する。

## 12. SoftBodyDemo設計からの採否

| SoftBodyDemo側の設計要素 | Skin Physicsへの扱い | 理由 |
|---|---|---|
| GPUをRuntime Physics Stateの唯一の正とする | 採用 | GPU物理・CPU Readback制約に適合 |
| GPU Commit Status / Event Query / 非同期Status Readback | 採用 | Submissionと成功Commitを区別するため |
| Rollback Snapshot / Capacity Retry / No Silent Drop | 採用 | 既存Broadphase Overflow契約を実装可能にする |
| CPU Double Oracle / Analytical Fixture / WARP分離 | 採用 | 既存Phase 1-11のGolden Test方針を強化 |
| D3D11 Immediate Context単一Command Stream / Dispatch境界 | 採用 | API制約と一致 |
| D3D11 Static Library、Host-owned Device / Context | 採用 | SkinPhysicsの再利用性に適合 |
| Neo-Hookean FEM XPBD + Tet Lambda Recompute | v1主方式には不採用 | 既存Corotational force-based FEMと状態・数式が異なる。モデル比較の将来候補とする |
| 1 Soft Body / 1 Static + 1 Kinematic Collider | 不採用 | Soft Body / Cloth / Self Collision / Mesh Pairの混在要件を満たさない |
| 200k Dynamic Particles / 40 FPS必須 | 不採用 | requirements.mdは3～40 FPSを開発目安とし、固定Workload合否を指定していない |
| 固定1/120秒 | 不採用 | Skin PhysicsはFixed Delta TimeをTest Environmentで設定可能 |
| SDF-only Collision Runtime | 不採用 | Skin PhysicsはVT / EEとTriangle Mesh Colliderを基準とする |

## 13. 実装開始条件

以下を満たした時点でGPU Runtime実装に進める。

- 基本設計1.2 / 詳細設計1.3 / 本書1.0の独立Critical Reviewで重大な設計矛盾が0件。
- Gate R0-R6の責務、入力、出力、失敗状態、検証Fixtureが追跡可能。
- Corotational FEM / XPBD Cloth / Contactの式とCPU Double Oracleが一致している。
- Runtime APIとDevice / Context / Render Bindingの所有権が確定。
- Async GPU Tick Schedulerの状態遷移とRollback / RetryがUML・テストケースで定義済み。

**本書は独立レビューへ提出可能な設計改訂案であり、レビュー合格・実装完了・GTX 1660 SUPER性能達成を意味しない。**
