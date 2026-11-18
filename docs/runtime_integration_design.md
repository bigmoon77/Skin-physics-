# Skin Physics GPU Runtime / Library Integration Design

    #include <cstdint>
    #include <string>
    #include <d3d11.h>
    #include "skin_physics/asset/types.hpp"

- 文書バージョン: 1.9
- 状態: DESIGN CLOSURE CANDIDATE / INDEPENDENT CRITICAL REVIEW REQUIRED
- 対応要件: requirements.md 1.1
- 対応基本設計: basic_design.md 1.10
- 対応詳細設計: detailed_design.md 1.12
- 前版: 1.8
- 実装基点: feature/phase1-importer
- 対象ビルド: SkinPhysics.sln / SkinPhysics.vcxproj（x64、C++17、D3D11）

## 1. 目的と設計上の位置付け

本書は、既存のSkin Physics設計を実際のGPUランタイムへ接続するため、Library境界、Simulation World、非同期GPU Step、状態Commit / Rollback、Render Binding、資源所有権、エラー処理を定義する。

Skin PhysicsはSoft BodyとClothを同一Simulation Worldで扱うC++静的ライブラリである。Soft Bodyの四面体Corotational FEM、ClothのXPBD、双方向VT / EE接触、自己衝突、アニメーション追従を統合する。物理計算とRender FeedbackはGPU上で実行する。

この設計は既存のSoft Body単体デモ向け仕様をそのまま移植しない。Skin Physicsの要件と既存設計を維持しながら、再利用可能なGPU Runtime境界と検証可能なStep transactionを追加する。

**本書の状態は独立レビュー待ちである。** API例とPass接続は実装契約案であり、Production Runtimeが完成済みであることを示さない。

## 2. v1の機能範囲

### 2.1 World構成

Runtimeは複数の独立Simulation Worldを作成できる。World間のCollision / Physics Couplingは行わず、各Worldは独立したSimulation Time、Model / Mesh ID、Contact Key、Tick Sequenceを持つ。D3D11 Device / Immediate ContextとGPU Submission ThreadはRuntime全体で共有する。Query Ringは全Worldのin-flight上限合計以上を収容する。

1つのSimulation Worldに以下を登録できる。

- 1つ以上のCharacter Model Instance。
- 各Model Instance内の複数Render Mesh / Physics Mesh。
- Mesh単位のphysics_mode: none / soft_body / cloth。
- Soft BodyとClothの混在。
- Static / Kinematic Triangle Mesh Collider。
- Mesh単位のSelf Collision設定とMesh Pair除外設定。

同一Model内のBody、Shirt、Pants、Hair Cards / Hair Bundles等は共通Worldで接触できる。Soft BodyはPhysics Mesh境界面、ClothはPhysics表面三角形を接触対象とする。Soft Body内部Tet面は外部接触プリミティブへ登録しない。World共通Substep Scheduleを採用し、v1ではSolver別Subcyclingを行わない。Force / Impulse Event境界とTOIでSubstepをIntegration Segmentへ分割し、Soft Body / Cloth / Contact / Animation Trackingは各Segmentを同じdtで進める。Segment開始境界でImpulseを一度だけ適用し、Segment終了後のDynamic Velocity再構築とFixed / Kinematic Animation Velocityの維持はdetailed_design.md §6.6 / §7.1.1 / §7.1.2に従う。

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
- SkinPhysics RuntimeはDeferred Contextを使用しない。同一Immediate Contextへの複数Thread同時呼び出し、独立したPhysics / Render GPU Queueを仮定しない。
- Host RendererがDeferred ContextでRender BindingのSRVを参照する場合、該当Command ListをImmediate ContextへExecuteCommandListした後にだけLeaseをReleaseする。Release後に未実行Command Listから当該SRVを参照することは禁止する。v1の最小構成ではRender BindingをImmediate Context上でDrawする。
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
            submission_thread_violation,
            blocked_by_input,
            blocked_by_capacity,
            blocked_by_ccd,
            blocked_by_predecessor,
            render_binding_busy,
            stale_handle,
            unsupported_feature,
            asset_invalid,
            capacity_exceeded,
            out_of_memory,
            device_removed,
            gpu_execution_failed,
            step_rejected,
            work_budget_exceeded,
            contact_geometry_unresolved,
            contact_manifold_reduction_failure,
            contact_impulse_unresolved,
            animation_time_precision_unresolved,
            no_force_target
        };

        enum class animation_discontinuity_policy : uint8_t
        {
            preserve_velocity,
            reset_velocity,
            blend_velocity
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
        struct animation_track_desc
        {
            std::uint32_t clip_index = 0;
            double time = 0.0;             // Clip Local Time [s]
            double speed = 1.0;             // d(Clip Time) / d(World Time)
            bool loop = false;
        };

        struct animation_playback_desc
        {
            animation_track_desc track;
            animation_discontinuity_policy discontinuity_policy =
                animation_discontinuity_policy::preserve_velocity;
            float w_event = 0.0f;           // BlendVelocity event weight [0,1]
        };

        struct animation_blend_desc
        {
            animation_track_desc source_track;
            animation_track_desc target_track;
            double blend_start_time = 0.0;  // Absolute World Simulation Time [s]
            double blend_end_time = 1.0;    // Absolute World Simulation Time [s]
            float weight_start = 0.0f;
            float weight_end = 1.0f;
            float tangent_start = 0.0f;     // d(weight) / d(World Time) [1/s]
            float tangent_end = 0.0f;       // d(weight) / d(World Time) [1/s]
            animation_discontinuity_policy discontinuity_policy =
                animation_discontinuity_policy::preserve_velocity;
            float w_event = 0.0f;           // BlendVelocity event weight [0,1]
        };

        struct transform;
        struct force_desc;
        struct impulse_desc;
        struct mesh_physics_parameters;
        struct attribute_edit_desc;
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

            status set_animation_blend(
                world_handle world,
                model_handle model,
                const animation_blend_desc& desc);

            status set_model_transform_sample(
                world_handle world,
                model_handle model,
                double sample_time,
                const transform& world_transform,
                bool is_teleport);

            status set_mesh_physics_parameters(
                world_handle world,
                model_handle model,
                uint32_t mesh_index,
                const mesh_physics_parameters& parameters);

            status apply_attribute_edit(
                world_handle world,
                model_handle model,
                uint32_t mesh_index,
                const attribute_edit_desc& edit);

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

            status release_render_bindings(
                world_handle world,
                uint64_t lease_generation);

            status reset_world(world_handle world);
            status destroy_world(world_handle world);
            status drain();
            status_code shutdown() noexcept;
        };
    }

Resource Lifecycleの破棄契約は次の通りとする。destroy_model / destroy_colliderは対象Worldのin-flight Tickが0件、対象Resourceを参照するTick Reservationが存在せず、Host Render Leaseが存在しない場合のみ成功する。BLOCKED_BY_INPUT / Render Backpressureの未Submission要求はModel / Collider Snapshotを固定していないため、Destroy後の次回Scheduler評価で現在のWorld構成を使う。該当条件を満たさなければBUSYを返し、HandleとResourceを維持する。reset_worldはin-flight Tick = 0、Host Leaseなしでのみ成功し、Accumulator / Pending Reservation / Step Queue / Contact Cache / Lambda / Diagnostic / requested / cancelled / committed / observed timeline countersをInitial Stateへ戻す。destroy_worldはPaused / Stoppedで、in-flight Tick / Query未回収 / Host Lease / Pending Reservation / Pending Step Request / 未割当Accumulator時間（fractional remainderを含む）が全て0のときのみ成功し、条件不足ではBUSYを返す。destroy操作の成功後、GPU QueryやCommand Streamが参照するResourceはそのCompletion Queryが完了するまでDeferred Release Listに保持する。

drain()は明示的な非Frame APIである。新規Tick Submissionを停止し、既にSubmission済みのGPU Query / TickのCompletionとDeferred Releaseを待つ。未割当Accumulatorのwhole Tick、未処理Step RequestまたはHost Render Leaseがある場合はBUSYを返す。DrainはHost Leaseを強制Releaseせず、HostにUnbind / Releaseを要求する。shutdown()は全WorldがDestroy済み、Query RingがIdle、Deferred Release Listが空、Host Leaseが0のときだけOKを返してCOM参照・Library Resourceを解放する。それ以外はBUSYを返し、状態を破壊しない。Runtime Destructorはshutdown成功後に呼び出す契約とする。通常Frame PathにFlush / 同期Waitを入れない。明示drain()のみGPU Event Queryの完了待ちを許可する。

Handleはindex + generationで識別し、Destroy後のHandle再利用を拒否する。APIからGPU内部Buffer、Solver Scratch、Writable UAVを公開しない。Statusのdiagnosticは呼び出し結果が保持するstd::stringとし、Runtime内部の一時文字列ポインタを返さない。Public APIはC++静的Library向けであり、異なるCompiler / CRT間の安定ABIは保証しない。Asset / DescriptorはVersioned Schemaを使い、ロード時に既存Asset Cacheのhash・coordinate_system_version・material_layout_version・generator identityを検証する。v1のRuntime ImporterはOBJ（呼出元が明示するunit scale必須）とglTF 2.0 / GLBを対象とする。ASCII Gmsh MSH 2.2はSoft Body Physics MeshのOffline中間入力として扱う。FBX、Morph Target / Morph Weight Animation、Draco圧縮、Meshopt圧縮はv1で未対応とし、Import時に明示Errorを返す。

### 3.4 Descriptor / StatusのField Contract

v1のPublic Descriptorは次の意味を持つ。
#### Review対象DescriptorのC++17 Field Width / Validation

この節で追加したPublic Fieldの型は次を固定する。Runtime / World Descriptor本体の実装はこのField Widthを使用し、暗黙のsize_t / long / pointer-sized整数をPublic設定値へ使わない。

| Field | C++17 Type | Validation |
|---|---|---|
| maximum_integration_segments_per_tick | std::uint32_t | 1以上。初期値512 |
| contact_max_substep_delta_time | double | finite、0以上。0は無効 |
| contact_impulse_iterations | std::uint32_t | 1～16。初期値8 |
| contact_impulse_relaxation | float | finite、0より大きく1以下。初期値0.5 |
| contact_impulse_residual_tolerance | float | finite、0より大きい。初期値1e-4 |
| animation_track_desc::time / speed | double | finite。speed=0はPose停止、負値は逆再生 |
| animation_track_desc::clip_index | std::uint32_t | Asset内Clip範囲内 |
| animation_blend_desc::blend_start_time / blend_end_time | double | finite、end > start |
| animation_blend_desc::weight_start / weight_end / tangent_start / tangent_end / w_event | float | finite。Weight / w_eventは0～1。Tangentは1/s |

animation_track_desc / animation_playback_desc / animation_blend_descは標準レイアウトを維持し、独自Pointer / std::string / std::vectorを含めない。RuntimeはAPI呼出し中にDescriptorを検証・コピーし、呼出し元のメモリを後続Dispatchまで保持しない。Public C++静的LibraryのABIは同一Compiler / CRTの組合せを対象とし、異なるCompiler / CRT間のBinary ABI互換性は保証しない。


| Descriptor | Field | 意味・制約 |
|---|---|---|
| runtime_desc | shader_package_root | Shader Bytecode / manifestの絶対またはPackage-relative root。Working Directory依存は禁止 |
| runtime_desc | maximum_gpu_resource_bytes | SkinPhysics全体のLibrary-owned Resource総量上限。0は明示上限なしとして扱い、DXGI Budget / 他利用者 / Host Reservedから自動算出 |
| runtime_desc | host_reserved_gpu_bytes | Renderer / OS / 他Subsystem向けにHostが確保するLocal Memory余裕。Libraryはこの領域を使用しない |
| runtime_desc | query_ring_capacity | すべての作成済みWorldのmaximum_in_flight_ticks合計以上。World追加時に不足すればRingを拡張し、未回収slotを上書きしない |
| runtime_desc | enable_debug_layer_validation | Debug / validation buildでD3D11 hazard診断を有効化 |
| world_desc | fixed_delta_time | 正の有限値。初期値1/60 s。Test Environmentから変更可能 |
| world_desc | time_scale | 0以上の有限値。Frame elapsedへ適用 |
| world_desc | gravity | SI単位 m/s²。初期値(0,-9.81,0) |
| world_desc | maximum_in_flight_ticks | 1 / 2 / 4を比較可能。出荷初期値2、上限変更はquery / dependency gateを含む検証が必要 |
| world_desc | minimum_substeps / maximum_substeps | World共通Substep数の下限 / 上限。初期値1 / 128。超過時はWORK_BUDGET_EXCEEDEDとしてTickをCommitしない |
| world_desc | maximum_integration_segments_per_tick | uint32_t。Animation Span / Force / Impulse / TOI境界後のLogical Tick内Integration Segment総数上限。初期値512、許容値1以上。超過時はWORK_BUDGET_EXCEEDEDとしてReservationを保持 |
| world_desc | world_max_substep_delta_time | World共通Substepの任意上限。0は未指定。全Solverへ適用する共通上限 |
| world_desc | contact_max_substep_delta_time | double [s]。初期値1/120 s、有限値かつ0以上。0は明示無効（Substep式では+infinity） |
| world_desc | contact_impulse_iterations | uint32_t。Coupled Projected JacobiのIteration数。初期値8、許容範囲1～16 |
| world_desc | contact_impulse_relaxation | float。Contact Impulse Jacobi Relaxation。初期値0.5、有限値かつ0より大きく1以下 |
| world_desc | contact_impulse_residual_tolerance | float。Contact Impulse正規化Residual上限。初期値1e-4、正の有限値 |
| world_desc | enable_cloth_temporal_quality_limit | Cloth拘束振動数ベースの時間品質上限を有効化。初期値true。XPBDの安定保証とは扱わない |
| world_desc | cloth_temporal_quality_factor | dt_limit_cloth_quality = cloth_temporal_quality_factor / omega_cloth_max。初期値0.2、正の有限値 |
| world_desc | render_binding_slots (derived) | maximum_in_flight_ticks + 1。Runtimeが作成し、Hostは変更できない |
| world_desc | maximum_steps_per_advance | CPU submission budget。未処理tickはAccumulatorに保持 |
| world_desc | memory_budget_bytes | World単位のResource総量上限。0はRuntime全体上限を継承。正値はWorld所有Resourceの合計へ適用 |
| model_desc | source_model | validated asset::model。mesh physics_mode、nodes、materials、skin、animationを含む |
| model_desc | mesh_physics_assets | Soft Body用physics_meshとmesh index / cache identityの対応。Cloth / Noneはphysics_modeに従う |
| model_desc | render_bindings | mesh indexごとのvalidated render_vertex_binding array。Rest Pose誤差上限を満たす |
| model_desc | world_transform | 初期World Transform。既存asset::mat4のcolumn-major規約に従う。Runtime RootはTranslation + Rotationのみ。ScaleはIdentity固定 |
| asset node transform | static scale | Offline BuildでGeometry / Rest Data / Mass / BindingへBake。負・ゼロScaleは禁止 |
| bone animation | scale channel | 正値かつ特異でないScaleのみ許可。負値・ゼロ・符号反転はImport Error |
| model transform sample | sample_time / world_transform / is_teleport | 時刻付きRoot Transform履歴。Clip連続移動とTeleportを区別し、Tick / Substep時刻へ補間 |
| mesh_physics_parameters | material channels | physics_modeに有効な単位付きSoft Body / Cloth Material。無効チャンネルの入力は拒否 |
| attribute_edit_desc | operation / channel / brush / target / strength | Paint / Smooth / Average / Erase。Render Mesh側の編集値をCPUで更新し、Runtimeへ不変のversioned editとして反映 |
| collider_desc | collision_mesh | Collision-only / Render Meshの三角形データ。static / kinematicで共通のTopology入力 |
| collider_desc | initial_transform | 初期World Transform |
| collider_desc | thickness / friction / restitution | SI単位・係数範囲は詳細設計のCollision Material定義に従う |
| collider_desc | collision_filter | Layer / Mask / Mesh Pair除外と一貫するFilter |
| animation_track_desc | clip_index / time / speed / loop | uint32_t / double / double / bool。Clip Local Time / Playback Speed / Loop。speed=0は停止、負値は逆再生 |
| animation_playback_desc | discontinuity_policy / w_event | enum class / float。PreserveVelocity / ResetVelocity / BlendVelocity。BlendVelocityはDescriptorの0～1 w_eventでDynamic Velocityを補間 |
| animation_blend_desc | source_track / target_track | 最大2本のanimation_track_desc。Additive / 3本以上はv1 Unsupported |
| animation_blend_desc | blend_start_time / blend_end_time / weight_start / weight_end / tangent_start / tangent_end | double / float。World Simulation Time上のCubic Hermite Blend Weight Curve。時間は絶対秒、tangentは1/s。区間外は端点Weightを保持し、Curve全域で0<=w<=1を必須とする |
| animation_blend_desc | discontinuity_policy / w_event | 明示Discontinuity時のPolicy。BlendVelocityはv_after=(1-w_event)*v_before+w_event*v_target(t_event+)。w_eventは0～1 |
| force_desc | event_id / target_filter | Retryを含む同一Eventの重複適用を防ぐStable IDと対象World / Model / Mesh範囲 |
| force_desc | position / direction / magnitude / radius / duration / start_time | ForceはN、半径m、継続時間s。start_timeはWorld Simulation Time原点からの絶対秒。end_timeはstart_time + durationの導出値でDescriptorに重複保持しない。正のRadius必須。Mass-weighted Falloffで分配 |
| impulse_desc | event_id / target_filter | Retryを含む同一Eventの重複適用を防ぐStable IDと対象World / Model / Mesh範囲 |
| impulse_desc | position / direction / impulse_magnitude / radius / event_time | ImpulseはN·s。event_timeはWorld Simulation Time原点からの絶対秒。瞬間イベントとして一度だけ適用 |
| advance_report | submitted_tick_count / pending_tick_count / in_flight_tick_count | 今回の受付・未送信・GPU未完了を分離 |
| advance_report | render_binding_backpressure / capacity_blocked | Render Slot leaseまたはGPU Capacityに起因するSubmission停止理由 |
| advance_report | first_required_input_tick / required_input_tick_count | Kinematic Pose履歴不足時に必要な時刻範囲 |
| world_status | submitted_sequence / gpu_committed_sequence / cpu_observed_sequence | GPU CommitとCPU確認済みprefixを別々に保持 |
| world_status | playback_state / tick_state / terminal_error / simulation_backlog | 状態、最終エラー、遅延・未処理時間を報告 |
| world_status | requested_simulation_time / cancelled_requested_time / effective_requested_time / reserved_tick_time / gpu_committed_simulation_time / cpu_observed_completed_time | 要求時間、未Reservation取消時間、有効要求時間、Tick Reservation時間、GPU Commit時間、CPU確認時間を別々に報告 |
| render_binding | position_srv / normal_srv / tangent_srv | Commit済みPhysics Stateから生成したRead-only SRV |
| render_binding | format / stride / vertex_count / index_count | Host RendererがDrawを構成するためのLayout |
| render_binding | resource_generation / device_generation / committed_tick_sequence | Stale Binding検出と描画状態の追跡 |
| render_binding | render_slot_id / render_slot_generation / lease_generation | Runtime-owned Render Candidate Ring内のSlot識別とHost lease検証。Releaseには同じlease_generationを必須指定 |

Animation Playback APIの状態遷移は次の通りとする。set_animation_playbackは単一Trackを有効にし、存在するClip Blendを次のTick Reservation境界で解除する。set_animation_blendはSource / Targetの2 TrackとBlend Weight Curveを同じ境界で一括置換する。Blend期間外はweight_start / weight_endを保持し、区間内だけCubic Hermiteを評価する。API呼出しの途中でGPU Packetを変更しない。Clip / Weight / Root / Collider Sampleの更新はTick Reservation開始前に採用され、そのTickのPacket Versionへ固定される。Span RecordはTrack / Channel / Blend / Collider Buffer Rangeを参照し、Blend Source / Target Track IndexはSpan-relative Track Record Index。Discontinuity Policyは明示されたTeleport / Reset / Time Rewind / Clip Switch Eventにだけ適用する。通常BlendはTrajectory微分へw_dotを含め、Velocity Resetしない。

`model_desc`内のAsset参照はadd_model呼び出し中に検証・Uploadし、Runtimeが必要なCPU Cache / GPU Resourceを所有する。APIは呼び出し元の一時vectorやspanを後続GPU Dispatchまで保持しない。Assetデータのコピーを避ける場合は、明示的なshared immutable asset handleとLifetime契約を別途追加する。

Model / ColliderのCreate / Destroy、Model Transform Sample、Mesh Physics Parameter変更、Attribute EditはTick Reservation作成前のWorld状態へ適用する。GPU Submission直前に入力・構造・Material Snapshotを固定し、既にReservation / Submission済みのTickは変更しない。BLOCKED_BY_INPUT / Render Backpressure中の要求はSnapshotを持たないため、Create / Destroy後の再評価で最新World構成を使う。Paint編集はRender MeshのStable Vertex IDを基準にversioned editとして保持し、既存Physics Mesh Bindingを使ってPhysics Vertexへ転送する。Topology再生成後に対応が一意でない場合は適用を保留して警告する。

`set_fixed_delta_time`はWorldがPaused、in-flight Tick = 0、pending whole Tick = 0、queued single-step request = 0のときだけ成功する。条件を満たさなければBUSYを返す。fractional remainderは秒単位の実時間として保持し、新しいFixed Delta Timeに対してAccumulatorのwhole tick数を再計算する。旧Fixed Delta Timeのwhole tickは存在しないため、時間を黙って破棄・換算しない。変更時はCloth / Contact / Animation XPBD LambdaとTick ScratchをResetする。

`get_render_binding`の最初の成功呼出しはWorldのHost Render Leaseを取得し、以降の各Mesh取得は同じlease_generationを返す。Releaseは取得時に返されたuint64 lease_generationを必須とし、現在Leaseと一致しない世代はSTALE_HANDLEとして拒否する。GPU Commit済みでもCPUがEvent Query / Status Readbackを確認していないTickのSlotは公開しない。Worldごとに同時に有効なHost Render Leaseは1つ。Hostは全MeshのDrawコマンド（Deferred Contextを使った場合はExecuteCommandListも）をImmediate Contextへ発行し、Position / Normal / Tangent SRVをVS / HS / DS / GS / PS / CS全StageからUnbindしてからrelease_render_bindings(world, lease_generation)を呼ぶ。Release後はSRV PointerもCommand List内の参照も使用しない。COM AddRefはCOM Objectの寿命だけを延長し、Slot内容のLeaseは延長しない。

### 3.5 WorldとMesh Instanceの配置

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

- Animation Trajectory PacketからSpan-local normalized timeとSegment-relative tauを使ってBone Local / Global Transform、Skin Matrix / 微分、Physics Vertex Target / Velocityを評価する。Absolute World TimeはCPU doubleで保持しGPU floatへCastしない。
- Span全域のConservative Speed / Acceleration / Chord Deviation Boundを利用してSwept AABB / CAを実行する。
- Soft BodyのF、Polar Rotation、Stress、Tet Force Contribution。
- Tet→Vertex Force GatherとSoft Body Integration。
- Cloth Stretch / Shear / Bend / Animation Tracking XPBD。
- Swept Broadphase、Spatial Hash、Candidate Pair生成・重複排除。
- VT / EE Narrowphase、Contact Patch、双方向Contact XPBD。
- Coupled Projected Jacobi Contact Impulse、TOI Impact / Friction / Restitution。
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
- Fixed Delta Timeを変更する場合、WorldをPausedにし、in-flight Tickが0件で、未処理のwhole Tick / pending Step要求がないことを要求する。条件を満たさない場合はBUSYを返す。条件成立後、fractional remainderは秒単位の実時間として保持してAccumulatorのwhole tick数を再計算し、Cloth / Contact / Animation XPBD LambdaとStep内ScratchをResetしてから新しい時間刻みに移行する。旧Fixed Delta Timeのwhole Tickを新しい時間刻みで実行しない。
- 最大Step / Substep予算に達してもAccumulator内の未処理時間を黙って破棄しない。

### 5.2 Fixed Stepと非同期GPU Stepの分離

CPU側のFixed Step制御はWall Clockから必要Logical Tick数を算出する。GPU Runtimeは各Logical TickをGPU CommandとしてSubmissionし、GPU上でCommit / Rejectを決定する。

管理する時刻は次の3つに分ける。

1. requested_simulation_time: Time Scale適用後のadvance供給時間と、受理したsingle_step要求時間を合算した累積要求時間。
2. gpu_committed_simulation_time: GPU Commit Controlが連続成功したLogical Tickの累積時間。
3. cpu_observed_completed_time: Event Query完了とCOMMITTED Status ReadbackをCPUが確認した連続Tick Prefixの累積時間。

requested_simulation_timeは受理した全要求の累積値であり、Pause / Stop / Resetによる未Reservation Step Requestの取消しでも減算しない。取消した未Reservation Step時間はcancelled_requested_timeへ加算し、effective_requested_time = requested_simulation_time - cancelled_requested_timeとして表示・Backlog評価に使う。Commit済み・Reservation済みStepは取消対象にしない。

各Logical Tickには固定Delta Timeの時間区間、Tick Sequence、Input Snapshot、時間ソース（Accumulator / single_step）を持つTick Reservationを割り当てる。Submission時にAccumulatorからdtを消費する場合、そのdtはAccumulatorからTick Reservationへ移動したものとして記録し、in-flight / blocked / retry中の予約時間として保持する。GPU Commit成功時だけReservationをCommittedへ移す。CPU Observationは表示・Status更新だけを行い、AccumulatorやSimulation Timeを加減算しない。

Capacity / CCD / PredecessorによるSubmission後の一時Blockは、同じTick SequenceとReservationを保持してRetryする。RetryでAccumulatorへ再加算したり、新しいTick Sequenceを発行したりしない。BLOCKED_BY_INPUT / Render BackpressureがSubmission前に発生した場合はTick Reservationを作らず、時間をAccumulatorまたはStep Request Queueに保持する。必要なPose履歴・Render Candidate Slot・Query Slotが揃い、GPU Submission直前にだけReservationを作成する。回復不能なFailureでは発行済みReservationを診断情報として残し、Reset / Stopの明示操作で未Reservation要求を破棄する。

`advance`が成功を返すことは、GPU TickがCOMMITTEDになったことを意味しない。Host側のUIはrequested / reserved / GPU committed / CPU observedを分けて表示する。

### 5.3 Submission / Backlog

- Play中のadvanceはmax(frame_delta_time, 0) * time_scaleだけrequested_simulation_time / unassigned_accumulator_secondsを増やす。Paused / Stopped中はGPU QueryのPollだけを行い、Wall TimeをAccumulatorへ追加しない。
- Tick時間はAccumulatorからTick Reservationへ移動する。各Tick Reservationは一意のSequenceと固定区間を持ち、GPU Commit / Retry / Rejectの結果を記録する。Accumulatorは未割当時間だけを保持し、reserved_tick_timeはin-flight / blocked Tickだけを集計する。
- SchedulerはAccumulatorにfixed_delta_time以上があり、必要Pose / Kinematic Sample、暫定Animation Packet / Event Boundary Tableが揃い、in-flight / Query / Render Candidate Slotが確保できる場合にTickをSubmission対象とする。これらの前提条件が揃う前はReservationを作らず、時間はAccumulatorに残す。全Resource Slotが確保できたSubmission直前に一意のTick Sequence / Input Snapshot / Animation Packet Versionを固定し、Accumulatorからfixed_delta_timeを一度だけTick Reservationへ移す。GPU Commit時に再度差し引かない。
- Capacity / CCD / predecessorの一時RejectはReservationを同じSequenceで保持してRetryする。Tick N+1がPredecessor Gateで停止した場合はN+1のReservationを破棄せず、NがReplay / Commitした後に新しいSource Generationで再Submissionする。
- 回復不能なFailureでWorldがFailedになった場合、当該Reservationおよび後続Pending Reservationは診断用に保持する。ResetはAccumulator、Reservation、Step Queueを明示的に破棄してInitial Stateへ戻す。
- single_stepはPaused / Stopped中のみ受け付ける。受理時にrequested_simulation_timeへfixed_delta_timeを加算し、wall-clock Accumulatorを変更せず、独立したStep Request Queueに1 Tickを追加する。Pause / Stop中はStep Request Queueをwall-clock Accumulatorより優先し、Accumulatorにwhole tick / fractional remainderが残っていてもStep Requestを先に処理してAccumulatorは変更しない。Pose / Render Slot / Query Slotが揃うまではStep Requestのまま保持し、GPU Submission直前にtime_source=single_stepのTick Reservationへ移す。Commit時にGPU committed timeを1 Fixed Delta進める。
- Pauseはwall-clock Accumulatorを保持し、in-flight Tickだけを完了させる。Stopは時間進行を停止し、未Reservation Step Requestだけを取消してcancelled_requested_timeへ加算する。Stopはwall-clock AccumulatorとPhysics Stateを保持する。ResetはInitial Stateへ戻し、Accumulator全体、Step Queue、Reservation / Sequence、Solver Cache、Timeline Counterを初期化する。既にTick ReservationとなったStepは通常のin-flight / Retry契約に従い、取消し・二重要求を行わない。
- D3D11 Immediate Contextは単一の順序付きCommand Streamである。PhysicsとRenderが独立GPU Queueで同時に動く前提を置かない。
- 1 PhysicsSubmissionBatchは1 Logical Tickを含む。
- 各Worldの初期上限はin-flight logical tick 2件、設定可能値は1 / 2 / 4件とする。Status Query RingはRuntime内の全Worldの上限合計以上のSlotを持ち、World追加時に不足すれば拡張する。未回収Slotを上書きしない。
- 各TickのDispatch Planはexpected_predecessor_tick_sequence、expected_source_state_generation、expected_source_state_set、dispatch_idを持つ不変Recordとする。
- Tick NのCommit Controlは成功時だけcommitted_tick_sequenceとcommitted_state_generation / active_state_setを更新する。Reject / BlockではCommitted Stateの参照先を変更しない。
- Tick N+1以降の先行Submissionは、tick_begin_controlでexpected predecessor sequenceとstate generationが一致することを検証する。すべてのBudgeted Physics Passは最初に同じTick Plan / scheduler gateを確認し、前提不一致・先行Tick未Commit・terminal failure時はPhysics Bufferの読み込み・書き込み・Atomicを行わず終了する。
- 先行TickがRejectされた場合、既にSubmissionされた後続TickはBLOCKED_BY_PREDECESSORとなり、物理状態を変更しない。先行TickのRetryがGPU上でCommitされた後、CPUは後続TickのPlanを新しいCommitted State Generationに合わせて再発行する。
- Planのsource / destination State SetはSubmission時点の予測Commit順序に基づいて固定する。GPU Commitが成立しない場合の後続Planは必ずgateで停止する。GPUが成功した場合のみ計画されたState Set遷移が有効になる。
- in-flight / Query / Render Lease上限に到達した場合、新規Reservation / Submissionを停止する。Submission前の時間はAccumulator / Step Request Queue、発行済み時間はTick Reservationへ残す。maximum_integration_segments_per_tick超過はQuery / Render Slot確保およびTick Reservation作成後のPreflight Blockとして扱い、同じTick Sequence / Snapshot / Packet Version / Reservation時間を保持する。設定上限を変更して再試行する場合も新しいTickを発行しない。
- RenderをSkip / ThrottleしてPhysics Submissionを優先することは許可する。
- Step数、Substep数、Iteration数、Contact候補、時間を性能維持のために黙って減らさない。
- Accumulatorが長時間増加する場合はsimulation_backlogとして警告・計測する。Skin Physicsの要件にない100ms Failureや40FPS必須条件を追加しない。
- Pauseは新規wall-clock時間供給とAccumulator由来の新規Reservationを止め、in-flight GPU Tickは安全に完了させる。Pause時間をactive elapsedへ加算しない。未割当Accumulatorは保持する。
- StopはPause状態へ移行し、未Reservation Step Requestを取消してrequested / cancelled時間を記録する。wall-clock AccumulatorとPhysics Stateは保持する。ResetだけがInitial State・Accumulator・Timeline Counterを初期化する。
- StepはPause / Stop中に固定時間刻みを正確に1回だけ要求する。Pause / Stop中のStep Requestはwall-clock Accumulatorより優先する。GPU Capacity / CCD / Input / Render Backpressure中は実行待ちとする。

初期in-flight上限2はWorldごとの資源・Query管理を単純化する初期値であり、2件が最適性能を保証する値ではない。1 / 2 / 4件と複数Worldの混在Submissionを比較し、Runtime全体のQuery Ring遅延・Memory・公平性を測定する。

### 5.4 Animation / Kinematic Inputの時間整合

CPU Animation EvaluatorはLogical Tick候補の有効なClip Key / Loop / Blend Weight / Root Transform Sample / Kinematic Collider Transform Sample境界を抽出し、Span単位の暫定Animation Trajectory Packetと境界表を生成する。PacketはRender / Query Slot確保後のTick Reservation作成時にImmutable Versionとして確定する。各Spanは1つの補間区間だけを含み、GPUが任意のtauで評価できるBone Transform係数・Blend Weight係数・Root Transform係数・解析微分を含む。set_kinematic_pose_sampleで登録された各ColliderはStable Collider ID / Span Sample Pair / Translation Linear係数 / Shortest-Arc Slerp Quaternion / Linear・Angular Velocity / Maximum Radius / Conservative Boundsを持つCollider Trajectory Recordへ変換する。入力Packet / Collider RecordはTick ReservationとともにVersion固定し、Retryで再利用する。

絶対World Simulation Time、Tick Reservation時刻、Clip Local Time、Span境界はCPU上でdouble管理する。GPU Shader Model 5へ渡すPacketはdoubleを含まず、uint32 / floatのみの固定幅Recordとする。GPUは絶対World TimeをfloatへCastしない。Immutable Packetとは別のWorld Working Cursor Bufferが保持するu_segment_startと、Segment開始からの相対秒tau [s]から、u_tau = clamp(cursor.u_segment_start + tau * span.inverse_span_duration, 0, 1)を算出する。TOIで残Segmentを始める場合はGPU上でCursor Recordをu_tauへ更新し、Packet係数 / Recordは書き換えない。CursorはSegment Snapshotへ含め、TOI retime / Retry時に同じCursorへRollbackする。CPU Readback / 再Uploadを挟まない。

Immutable GPU Packet ABIはdetailed_design.md §5.3のanimation_span_gpu_record 80 bytes、animation_track_gpu_record 32 bytes、animation_channel_gpu_record 224 bytes、animation_blend_gpu_record 48 bytes、kinematic_collider_gpu_record 128 bytesを正本とする。Mutable World Cursor ABIはanimation_span_cursor_gpu_record 16 bytes。Cursorは別UAV / Working Bufferであり、Immutable Packetには含めない。Recordは4-byte packing / alignment、uint32 / float / float4のみ。Span / Channel / Blend / Colliderの全Field Byte Offsetも§5.3のABI表で固定する。CPU Absolute Timeはdouble、GPUはSpan-local normalized time / Segment-relative tauのみfloatで扱う。C++ sizeof / alignof / offsetof Static Assertion、HLSL StructuredBuffer stride、WARP ABI ReadbackをR5 Gateの必須検証にする。Boundをfloatへ変換するときは外向き丸めを行う。floatへ変換したSpanでu_tauの単調進行または境界判定が保証できない場合はSpanを細分化し、最大深度8で解決しなければBLOCKED_BY_INPUT / CCDUnresolvedとしてCommitしない。


- STEP / LINEAR / CUBICSPLINEの係数化、Quaternion Slerp / normalized Hermite微分、最大2 Track Blend、Hierarchy / Skinning微分はdetailed_design.md §5.3を正とする。
- Kinematic Colliderは連続する時刻付きTransform SampleをTranslation Linear + Shortest-Arc Slerpで評価する。CAの任意tauでTransform / Contact Point Velocityを算出し、Speed / Sagitta Boundを同じPacket Versionで提供する。必要なSample Pairがbracketできない場合はBLOCKED_BY_INPUT。
- CPUは係数区間から保守的なSpeed / Acceleration Boundを生成し、GPU Trajectory Packetと同一Versionで渡す。BoundsはInterval Arithmetic / 外向き丸めを使い、実軌道の上限を過小評価しない。CPUのdouble絶対時刻をGPU floatへ直接渡さず、GPUではSpan内uとSegment相対tauだけを使用する。
- Animation Span境界はWorld Integration Segment境界へ追加する。TOIは単一Span内で発生するため、CAは同じimmutable Packetを使って任意tauの姿勢・速度を評価する。通常Substep / Force / Impulse / TOIを含む全Segmentの進行後にWorld Working Cursorのu_segment_startを更新し、TOI prefixではtau、その他はdt_segmentを使う。次SegmentがSpan境界へ到達した場合はspan_record_indexを次Spanへ切り替え、u_segment_start=0とする。正の時間進行でCursorが増加しない場合はANIMATION_TIME_PRECISION_UNRESOLVEDとしてCommitしない。
- Clip Blendは最大2 Track、非Additiveとする。Blend Weightの時間微分項を必ず含める。
- Missing Pose / Key / Bound、正規化不能Quaternion、非有限微分はBLOCKED_BY_INPUTまたはInvalid Animation Inputとし、前回Poseや速度の流用・有限差分代用を行わない。

外部からBone Pose / Kinematic Transformを直接供給するHostでは、時刻付きSampleとTrajectory Derivative Sampleを登録する。Input Sampleの履歴はTick全体とTOI再試行・Span分割をbracketできるだけ保持する。Kinematic Colliderの回転接触点Velocityはv_linear + omega cross radiusを使う。Teleport / Reset / Time Rewindは明示Discontinuity Eventとし、Policyに従ってContact / CCD CacheとVelocity履歴を再初期化する。

### 5.5 World共通Substep Schedule

詳細設計§6.8 / §7.1.1 / §7.1.2のWorld共通Substep / Integration SegmentをRuntime Schedulerの唯一の時間分割規則とする。Logical Tick Beginで参加する全Soft BodyのFEM CFL / Animation Tracking制限、全Cloth ConstraintのTemporal Quality Limit、Contactの時間上限（contact_max_substep_delta_time > 0の場合だけ適用）を集約し、World Descriptorのminimum_substeps / maximum_substeps / world_max_substep_delta_timeを適用してN_worldを確定する。contact_max_substep_delta_time=0は+infinityとして扱う。World Substep / Clip Key / Loop / Blend Weight / Root Transform / Force / Impulse境界を統合したSegment数をGPU Physics Upload前にCountし、maximum_integration_segments_per_tick（初期値512）を超える場合はWORK_BUDGET_EXCEEDEDとしてReservationを保持する。Keyまたは異なるEvent時刻の境界を結合・省略しない。TOIで新たな残Segmentを作成する際も実行済みSegment数へ加算し、上限を超えたら残SegmentをDispatchせずTick全体を未Commitにする。全Soft Body / Cloth / Contact / Animation Trackingは同じdt_world=fixed_delta_time/N_worldを使用する。ClothのTemporal Quality LimitとContact Temporal Quality LimitはXPBDの安定保証ではなく品質上限として扱う。各SubstepはAnimation Span / Force / Impulse / TOI境界でIntegration Segmentへ分割する。Segment内のForceは開始時に評価し、Impulseは開始境界で一度だけ適用する。Mesh追加・Material変更・Fixed Delta変更は次Tick開始境界で反映し、そのTickのschedule計算から再評価する。N_worldがmaximum_substepsを超えたらWORK_BUDGET_EXCEEDEDでCommitせずReservationを保持する。

## 6. GPU Tick Transaction / Commit / Rollback

### 6.1 State Bufferの分離

少なくとも次を分ける。

- state_bank[2]: Worldごとに2つの完全なPersistent State Bankを持つ。各BankはPosition / Velocity / Soft Body dynamic state / Animation target history / Persistent Contact Cache（Contact Key / Normal / Patch Generation）を保持する。XPBD Lambda / Impact EventはPersistent State Bankへ含めない。
- active_state_bank: GPU Commit済みのSource Bank。Tick全体を通してRead-onlyであり、Retry / Reject時の完全なLogical Tick開始状態を兼ねる。
- inactive_state_bank: 次のTickがWriteするDestination / Working Bank。Physics Stateの変更はこのBank内だけで行う。
- trajectory_cursor_working: WorldごとのMutable UAV Buffer。animation_span_cursor_gpu_recordを保持し、Immutable Packet Version / Span Record Index / normalized uを参照する。Tick開始時に初期化し、TOI / Segment進行はGPUだけで更新する。Persistent Packet Recordは変更しない。
- segment_start_snapshot: 現在処理中のIntegration Segment開始状態を保存するWorldごとの再利用Scratch。Physics Position / Velocity、Animation Input Version、trajectory_cursor_workingのCursor Record、Impulse Event Cursor / 適用済みEvent ID状態を含む。TOI retime / Event Segment Retry時は同じCursorへ戻してinactive_state_bankを復元する。1本のWorld Command Stream内で順序実行するため、同一Worldで同時使用するSnapshotは1つ。
- tick_dispatch_plan: expected_predecessor_tick_sequence、expected_source_state_generation、expected_source_state_set、dispatch_id / tokenを持ち、ReservationからExecution / Commitまでimmutableとする。
- segment_solver_scratch: Cloth Stretch / Shear / Bend / Animation Tracking / ContactのXPBD Lambda、projected correction、Impact Resolved Eventを保持する。各World共通Integration Segment開始時にLambdaを0へ初期化し、同Segment内のSolver反復中だけ保持する。Segment間 / Logical Tick間のLambda Warm Startはv1で行わない。Persistent Contact Key / Normal / Patch GenerationだけをState Bankへ保持する。
- render_candidate_slots: Tickごとに確保するPosition / Normal / Tangent candidateとslot_generation。GPU Commit ControlだけがPhysics committed stateとRender slot referenceを同時に切り替える。
- host_render_lease: CPU-observed committed Render slotをHostへ貸し出したlease_generation。lease解放までslotを再利用しない。
- simulation_status / tick_sequence / state_generation / diagnostic_counters: GPU Commit / Reject / Error Control。

GPU Solver ScratchをHost Render Bindingへ直接公開しない。WorldのPersistent Physics State Bankは2つに固定する。Tick Beginでactive_state_bankをImmutable Sourceとし、inactive_state_bankへ予測・Solver結果を書き込む。すべてのSubstep、CCD、Contact、Render Candidate Validationが成功した場合だけGPU Commit Controlがactive_state_bankとstate_generationを切り替える。Reject / Blockではactive_state_bankとstate_generationを維持し、inactive_state_bankをRetry / 次Tick用に再利用する。Substep / CCD Segment用snapshotは同一Command Stream内で順序利用する再利用Scratchであり、3つ目のPersistent State Bankではない。Render Candidate SlotはPhysics State Bankと独立したRing Resourceとする。後続Tickは前TickのCommit結果とGenerationを検証してから実行する。

### 6.2 GPU Commit Control

Logical Tickは次の条件を全て満たした場合だけCOMMITTEDとする。

- すべてのSubstepが正常に完了。
- Broadphase / Candidate / Contact / Contributionの全Capacityが有効。
- CCDが全必須候補でResolvedまたはNo Hitに確定し、Unresolvedが0。
- Position / Velocity / Stress / Lambda / TOI等にNaN / Infがない。
- Tet Inversion / Volume / Contact Residual / Initial Repair等のScenario Gateを通過。
- Render Feedback Candidateが専用Slotへ生成され、Binding / Geometry ValidationとNonFinite検査を通過している。
- GPU DiagnosticのTerminal Failureが0。

判定はGPU側のReduce / Commit Controlで行う。CPU Readbackの完了をTick Commitの条件にしない。CPUのcompleted_timeはQuery完了後の非同期Status Readbackを確認してから進める。

### 6.3 Rollback / Retry

Capacity Overflow、CCD Unresolved、物理品質Gate失敗時は、現在のLogical TickをREJECTED / BLOCKEDにする。GPU Commit前のactive_state_bankは変更されていないため、Retryは同じactive source bankと同じTick Inputからinactive destination bankを再構築する。NonFinite / Invalid Asset / ABI mismatchは自動Retryしない。Substep / CCD Segmentの再計算はsegment_start_snapshotからinactive destination bankを復元する。

- Capacity Overflow: Count / Scan / Capacity Checkで必要数を記録し、Fill / Consumerの範囲外アクセスを禁止する。Capacity Growth後のRetryはactive source bankを不変のまま読み、inactive destination bankを先頭から再構築する。Event Query / Status Readback後にCPUがRequired Capacityを回収し、該当Resource GroupのCapacityを next_power_of_two(max(required_count, ceil(current_capacity * 1.5))) へ拡張する。0容量からの初回確保はnext_power_of_two(required_count)とする。uint32 / D3D11 ByteWidth / Dispatch上限を超える要求は成長で解決できないCapacity Errorとする。Resource Group全体を新規確保できるMemory Budgetを先に確認し、割当・Copy / Rebuildが完了してから新Resource Setへ切り替える。旧Resource Setは参照中の全Query / Tick / Render Leaseが解放されるまで退役させない。Capacity Growth中は新規Tick Submissionを停止し、影響する全in-flight TickのQuery完了と状態判定をnon-blockingで待つ。後続Tickは先行状態が確定するまでBLOCKED_BY_PREDECESSORを維持し、先行Tickを同一Snapshot / Inputで再実行してCommitした後、新Generationに基づいて再Submissionする。最大GPU Resource Budgetを超える場合はBlockedByCapacityを維持し、候補を捨てない。
- CCD Unresolved: Internal Substep数またはCCD分割上限を上げて、同じactive source bank / Tick Inputからinactive destination bankを再構築する。TOI retime / Segment再計算は該当segment_start_snapshotから復元する。No Hitへ置き換えない。
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

これにより、先行Tickの失敗後に後続Tickが未確定Working Stateを入力としてCommitする経路を禁止する。各Worldは2つのPersistent State Bankと1つの再利用Integration Segment Snapshotを持ち、GPU Commit ControlだけがBankを切り替える。WorldをまたぐTickは同じImmediate Context Command Streamへ順序付けるが、異なるWorldのCommitted Stateは相互参照しない。Resource ABI ReviewではState Bank / SnapshotのBuffer Stride、SRV / UAV Binding、Lifetime、Render Candidate Ringを含むVRAM / GPU時間を検証する。

## 7. GPU Pass / Dataflow Integration

### 7.1 World Logical Tick

1. CPU Animation EvaluationでLogical Tick候補のAnimation Trajectory Span Packet / Conservative Bounds、Kinematic Transform Sample、World Substep / Animation / Force / Impulseの暫定Boundary Tableを生成する。Target PoseとTrajectory Derivativeは同じSimulation Timeで評価する。この段階ではAccumulatorから時間を移動しない。
2. Render Candidate Slot / Query Slot / in-flight枠を確保する。いずれか不足する場合はPacket / Boundary Tableを破棄可能なPending Inputとして扱い、Accumulator / Step Request Queueに時間を保持する。Slot確保成功後にTick Sequence / Input Snapshot / Animation Packet Versionを固定し、Accumulatorからfixed_delta_timeを一度だけTick Reservationへ移す。
3. 確定済みBoundary TableからWorld Substep / Animation / Force / Impulse境界を統合した予定Segment数をCountする。maximum_integration_segments_per_tickを超える場合はWORK_BUDGET_EXCEEDEDとして同じTick Reservation / Sequence / Snapshot / Packet Versionを保持し、GPU Physics Upload / Dispatchを行わない。異なる時刻のBoundaryを結合・省略して上限内へ丸めない。設定更新後のRetryは同じReservationを使用する。
4. GPUへAnimation Trajectory Packet / Collider RecordをUploadし、Span Coefficient / Bound / Input Version / ABI StrideをValidate。GPU SkinningでAnimation Target / Target Velocityを生成する。
5. Logical Tick開始SnapshotとTerminal Status初期化。
6. 各World Integration Segmentについて次を処理する。
   - Segment開始Snapshot / XPBD Lambda / Contact Lambda / Impact Scratch Reset。
   - Animation Span Packet / Bound Versionを選択し、Segment開始・終了時刻のAnimation Target / Animation Velocity / Root / Kinematic Poseを同じGPU Trajectory Evaluatorで取得。
   - Force EventのSegment内有効状態を確定し、Impulse EventをSegment開始でEvent ID単位に一度だけ適用。
   - Soft Body Tet Deformation Gradient / Polar Rotation / Stress / Inversion Penalty / Segment開始状態のTet Force Contribution。
   - Vertex→Tet CSR Force Gather（float Atomic Add禁止）。
   - Soft Body external / internal / animation forceとSegment積分によるVelocity / Position Prediction。
   - Cloth external force / impulseとSegment積分によるVelocity / Position Prediction。
   - Swept AABB / Broadphase Count / Scan / Capacity Check / Fill / Sort / Pair Dedup。
   - VT / EE Narrowphase、Contact Patch Reduction、CCD Conservative Advancement / Minimum TOI Reduction。
   - TOIなし: Cloth Stretch / Shear / Bend / Animation Tracking等の非接触内部Constraintを解く。
   - TOIあり: Endpoint Contactを確定せず、Dynamicの離散pathとAnimation Trajectory Packetが評価するtau時刻のKinematic実TrajectoryでTOI prefixを構成し、非接触内部Constraintをtau時間で解く。TOI分割後に作成する残SegmentをSegment数へ加算し、maximum_integration_segments_per_tickを超える場合は残SegmentをDispatchせずTickをWORK_BUDGET_EXCEEDEDで未Commitにする。
   - Contact Position Constraint前にDynamic VertexのContact前Velocityを保存する。通常Segmentはdt_segment、TOI prefixはtauで速度再構築する。Fixed / Kinematic VelocityはAnimation Trajectory微分を維持する。
   - Discrete / TOI Contact Position XPBDを解く。Contact位置補正はPositionだけを変更し、保存済みContact前Velocityへ反映しない。
   - 保存したContact前VelocityをCoupled Projected Jacobiへ渡す。Contact Contributionを反復Gatherし、Normal / Coulomb Cone Residualが許容範囲へ収束した場合のみv_finalを確定する。W_ii <= epsilon_inverse_massのContactはNON_RESPONSIVE_CONTACTとしてPosition / ImpulseをSkipする。TOI Contactは成功時にimpact_resolvedを設定する。
   - Broadphase Envelope / NonFinite / Capacity / CCD / Physical Quality Gateを検証する。Envelope外は同じSegment内で再Buildし、Contact解決を再実行する。
   - TOIありの場合、Impact後Velocityから残時間の新しいIntegration Segmentへ進む。Impact後Velocityを同じSegmentの速度再構築で上書きしない。
6. 全Integration Segment / World Substep成功後にRender Feedback Candidate Prepare（Tick Reservation済みslotへ書き込む）。
7. Render Candidate Validation。
8. GPU Logical Tick Commit ControlでPhysics State Set / GenerationとRender Candidate Slot / Generationを同時にcommit。
9. GPU Status / Diagnostic Copy、Timestamp / Event Query。Statusはcommitted render slot ID / generationを記録する。

Soft Body内部力は各World Integration Segment開始状態で再評価する。Soft BodyのFEM ForceをXPBD Contact Lambdaへ混ぜない。Contact Position projectionとVelocity Impulseは別処理とし、Contact Position補正がRestitutionへ疑似Velocityとして混入しないようにする。

### 7.2 CCD / TOI

既存のPoint-Triangle CCDは初期Fixtureとして維持する。Production Runtimeでは変形Mesh間の高速接触を満たすため、VTとEEの双方に対してSwept BroadphaseとConservative Advancementを接続する。Triangle-Triangleの交差候補はVT / EEイベントへ分解して扱い、Primitive Pair / Contact Keyの重複を除去する。

TOIは現在Integration Segment開始からの秒単位で管理する。Dynamic Vertex CCD pathはdetailed_design.md §11.4のSemi-implicit Euler離散pathを使用し、TOI prefixでdt=tauの再Predictionを行わない。CAの各tauでAnimation Trajectory Evaluate PassがBone / Skinning Transformを評価し、その同一出力からDistance / Contact Velocityを得る。Kinematic任意時刻評価は5.4のAnimation Trajectory Packetを使用し、係数・BoundsのVersionをTick Reservationへ固定する。Kinematic Vertex / ColliderのTOI geometryはtau時刻のAnimation / Rigid Transform Trajectoryを直接評価し、Endpoint Chordは保守的Deviation Boundで膨張したSwept AABB構築に限って使う。Conservative Advancementの距離も実Trajectory位置で評価し、v_boundにkinematic_path_speed_boundを含める。有限個の中間サンプルだけでDeviation / Speed Boundを保証しない。CUBICSPLINE TranslationはBezier Control Hull、Bone Hierarchy / Skinningは区間演算または保守的導関数Bound、Slerp Rigid Colliderは回転角・最大半径のSagitta Boundを用いる。Deviation / Speed Boundを取得できない場合はSegmentを最大深度8まで分割し、それでも証明できなければCCDUnresolvedとする。

Kinematic接触相対Velocityはtau時刻のAnimation Target Trajectory微分、またはv_linear + omega cross radiusを使用し、Chord Path Velocityへ置換しない。TOI prefixでは非接触内部Constraintを先に解き、Contact前Dynamic Velocityを再構築して保存する。次にContact Position XPBDを解き、Contact位置補正後のVelocity再構築は禁止する。保存済みContact前VelocityをCoupled Projected Jacobi Impulse Solverへ渡し、収束Residualを検証する。W_ii <= epsilon_inverse_massの非応答ContactはImpulse / Position correctionをSkipし、全ImpulseがResolvedの場合だけImpact後状態から残Segmentを進める。

複数接触は同時TOI許容幅でグループ化し、同一Eventを二重Impulseしない。Unresolved、進行不能、Iteration上限は未解決のままCommitしない。

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

Library所有Buffer / TextureのByte数、DXGI Local Memory CurrentUsage / Budget、Host Reserved Bytesを別々に計測する。DXGI CurrentUsageにはSkinPhysics自身の現在割当も含まれるため、他利用者の概算使用量を

    dxgi_other_usage = max(0, dxgi_current_usage - library_owned_current_bytes)

として分離する。Host Reserved BytesはCurrentUsageに含まれていない追加予約量として定義する。

runtime_desc.maximum_gpu_resource_bytesが正値の場合はLibrary-owned Resourceの設定上限、0の場合は設定上限なし（UINT64_MAX相当）とする。DXGIと他利用者を考慮したLibrary総量の上限は、

    dxgi_library_ceiling =
        max(0, dxgi_budget - dxgi_other_usage - host_reserved_gpu_bytes)

    configured_library_limit =
        maximum_gpu_resource_bytes > 0
            ? maximum_gpu_resource_bytes
            : UINT64_MAX

    library_limit = min(configured_library_limit, dxgi_library_ceiling)

とする。これによりmaximum_gpu_resource_bytes=0でもDXGI Budgetを上限とする自動管理が有効になる。

Capacity Growth時は、

    runtime_growth_headroom = max(0, library_limit - library_owned_current_bytes)

を求める。Worldのmemory_budget_bytesが正値ならWorld Resource総量上限、0ならRuntime上限を継承する。World単位のGrowth HeadroomとRuntime Growth Headroomの小さい方を許可量とする。Resource Group全体の追加Byte数が許可量を超える場合はAllocation前に拒否し、WorldをBlockedByCapacityへ移す。既存Resource Setを解放してから確保する危険な入替えは行わず、新Resource Group確保後にAtomicな参照切替を行う。旧Resource Setは全Tick / Query / Render Leaseが解放されるまで保持する。

Peak計測にはPhysics State Banks / Segment Snapshot / Contact・Candidate / Render Candidate Ring / Query / Stagingを含む全Library Resourceを計上する。Capacityは通常運用中に縮小しない。

### 8.3 Error分類

| Error | Runtime動作 |
|---|---|
| InvalidAsset / InvalidTopology | Asset Load拒否 |
| DegenerateTetrahedron / InvertedTetrahedron | Asset / Step Validationに基づき警告またはReject |
| InvalidMaterialParameter | 設定変更拒否 |
| SkinWeightTransferError / RenderBindingError | 該当MeshのLoad / FeedbackをReject |
| SubmissionThreadViolation | 初期化時に登録したSubmission Thread以外からImmediate Context / GPU Resourceを操作しようとしたAPI CallをReject |
| BlockedByInput | Animation / Kinematic Pose Sample履歴が必要なTick / Substep時刻をbracketできず、入力不足範囲の登録を待つ |
| RenderBindingBusy | Host Render Leaseが未解放、またはRender Candidate Slot不足により新規Submissionを停止 |
| BroadphaseOverflow / ContactOverflow / ContributionOverflow | TickをRollbackし、Capacity調整後に再実行 |
| ContactImpulseUnresolved | Coupled Projected Jacobiが最大IterationでResidualへ収束しない。Tick全体をRollback / Blockし、Iteration / Solver設定を検証してRetry |
| AnimationTimePrecisionUnresolved | GPU Span-local normalized timeが正のSegment進行を表現できない。Span細分化深度8後も解決しない場合にTick全体をBlock / Reject |
| NonResponsiveContact | 有効逆質量が閾値以下の正常な非応答Contact。Position / ImpulseをSkipし、Diagnostic Counterへ記録 |
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

Render BindingはPosition / Normal / TangentのSRV、Format、Stride、Vertex Count、Index Count、Resource Generation、Device Generation、Committed Tick Sequence、Render Slot ID / Generation、Lease Generationを返す。SRVはBorrowed COM Pointerであり、HostのAddRefはCOM Objectの生存期間だけを延ばす。Render Slotの内容はlease_generationの有効期間だけ保証する。WorldごとにHost Render Leaseは同時に1つだけ許可し、全Mesh Bindingを同じLeaseへ紐付ける。

get_render_bindingはCPUがEvent Query完了、COMMITTED Status Readback、連続Tick Prefixの3条件を確認した最後のTickに対応するRender Slotを返す。GPU上でCOMMITTEDになっていてもCPU-observed prefixへまだ入っていないTickのSlotは返さない。最初の成功呼出しがWorldごとのHost Leaseを取得し、すべてのMesh Bindingは同じlease_generationに属する。release_render_bindings(world, lease_generation)は現在のGenerationと一致する場合だけ成功し、古い / 二重ReleaseはSTALE_HANDLEを返す。

Host Rendererは全MeshのDraw commandをImmediate Contextへ発行し、公開SRVをVS / HS / DS / GS / PS / CSの全Graphics / Compute StageからUnbindしてから、release_render_bindings(world, lease_generation)を呼ぶ。Deferred Contextを使用するHostは、Leaseに属するSRVを参照する全Command ListをImmediate ContextへExecuteCommandListした後にReleaseしなければならない。未実行Command Listを残したままReleaseすることは禁止する。Release後はSRV PointerとCommand List内の参照を使用しない。RuntimeはLease解放を確認するまで当該Render Slotを再利用しない。Render Candidate RingのSlot数はmaximum_in_flight_ticks + 1とする（既定in-flight 2なら3 Slot）。これによりCPU-observedな直前Committed Slot 1つと最大2件のin-flight Candidateを保持できる。HostがLeaseをreleaseせず次のSubmissionを要求した場合、RuntimeはBUSY / render_binding_busyで追加Reservation / Submissionを止め、時間をAccumulator / Step Request Queueに保持する。Render Candidate生成・検証・Ring Memory / Publish CostをPerformance Runへ含める。

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
- 有効逆質量0のNonResponsiveContact GuardとWorld-wide Coupled Projected Jacobi Impulse。
- Contact Candidate集合が小規模全探索の確定候補を全て含む。Contact Impulse Residual / Coulomb Cone Acceptanceも満たす。
- Capacity Overflow / ContactImpulseUnresolvedのRollback / Retry。

### Gate R4 — CCD / Initial Repair

- Moving VT / EE Conservative Advancement。
- GPU Animation Trajectory Packetを使う任意tauのKinematic Position / Velocity評価、保守的Span Bounds。
- 同時TOI、Positive-Time TOI、初期接触、退化・進行不能。
- TOI Coupled Impulse一度だけCommit、残時間再積分。
- Initial Repair per-vertex budgetと未修復Error。

### Gate R5 — Animation / Render Integration

- Multi-key glTF Clip / Skeleton / Mesh Node Matrix。
- GPU Animation Span / Track / Channel / Blend Packet、任意tauのSkinning Position / Velocity / Bounds。
- Fixed SM5 Packet ABI / Span-local normalized cursor monotonicity / float precision gate (Span 80 B / Track 32 B / Channel 224 B / Blend 48 B / Collider 128 B)。
- Mutable World Cursor Buffer 16 B / Snapshot rollback / immutable Packet mutation count 0 gate。
- 2 Track Clip Blend / w_dot / BlendVelocity Discontinuity。
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

- 基本設計1.10 / 詳細設計1.12 / 本書1.9の独立Critical Reviewで重大な設計矛盾が0件。
- Gate R0-R6の責務、入力、出力、失敗状態、検証Fixtureが追跡可能。
- Corotational FEM / XPBD Cloth / Contactの式とCPU Double Oracleが一致している。
- Runtime APIとDevice / Context / Render Bindingの所有権が確定。
- Async GPU Tick Schedulerの状態遷移とRollback / RetryがUML・テストケースで定義済み。

**本書は独立レビューへ提出可能な設計改訂案であり、レビュー合格・実装完了・GTX 1660 SUPER性能達成を意味しない。**


## 14. バージョン履歴

- 1.9: Immutable Trajectory PacketとMutable per-World Span Cursorを分離し、Cursor UAV / Segment Snapshot Rollback / R5 ABI Isolation Gateを追加。
- 1.8: Span / Track / Channel / Blend / Colliderの固定Byte Offset / Flags / Node / Inverse Bind参照、Blend Curve時刻写像、Collider Angular Axis / Velocity、StructuredBuffer ABI検証を確定。
- 1.7: Shader Model 5固定幅GPU Packet ABI / Byte Offset、CPU double絶対時刻とGPU Span-local float時刻、各Segmentでのnormalized cursor更新、任意tauのBone / Collider評価、Kinematic Collider Record、Segment Preflight Reservation順序を明文化。GPU ABI Readback / Time Precision Gateを追加。
- 1.6: GPU Animation Trajectory Packet / Spanと任意tau評価、最大2 Track Blend / BlendVelocity、Contact有効逆質量Guard、Coupled Projected Jacobi Impulse、Contact Temporal Quality LimitをRuntime Descriptor / Gateへ反映。
- 1.5: Contact Position補正とContact前Velocity / Impulse処理の分離、Animation Trajectory微分の正本化、Kinematic CCDの保守的Chord Bound、GPU Memory Budgetの0 sentinel修正、Cloth Temporal Quality Limitを追加。
- 1.4: Semi-implicit Eulerの離散CCD pathとTOI prefix状態の一致、TOI時間を使うXPBD Compliance、Impact後残Segmentの再評価規則を明文化。
- 1.3: Force / Impulse EventとTOIによるIntegration Segment分割、Soft Body / ClothのSegment積分同期、Kinematic Velocity所有規則をRuntime Scheduler / TOI契約へ反映。
- 1.2: World共通Substep契約、AccumulatorからTick Reservationへの時間移管、Lease Generation付きRelease、Deferred Context Command Listの解放順序、Model / Collider / World / RuntimeのDestroy・Drain・Shutdown条件を明文化。
- 1.1: 複数WorldのQuery Ring集約、Model Transform / Material / Paint API、Pose履歴不足時のBLOCKED_BY_INPUT、Fixed Delta fractional remainder、Capacity Growth / Memory Budget、Render Candidate Ring / Host Lease、Physics State Bank 2枚とSubstep Snapshotの分離を明文化。
- 1.0: Runtime / Library境界、Public API、D3D11 Device / Context所有権、Fixed Step Scheduler、GPU Commit / Rollback、Render Binding、Device Loss、R0-R6 Gateの初版。
