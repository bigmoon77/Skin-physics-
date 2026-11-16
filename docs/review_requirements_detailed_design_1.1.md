# 要件定義 v1.1 / 詳細設計 v1.2 整合性レビュー

- 対象要件: docs/requirements.md 1.1
- 対象詳細設計: docs/detailed_design.md 1.2
- 要件定義 blob: f47eaa0bd497bb3a043ce422b5c9aa8ed7fcaa91
- 詳細設計 blob: 4b334a08378bb3d583c3937814c6dec9f7481c38
- 判定: **合格**
- 直接矛盾: **0件**
- 初版詳細設計の記述不足: v1.2で対応済み

## 1. 総評

要件定義1.1と詳細設計1.2を機能範囲、実行環境、CPU / GPU責務、物理機能、モデル入力、アニメーション、衝突、物理特性編集、試験環境、JSON、性能・品質目標の単位で照合した。

**両文書の間に、同時に成立しない仕様や明示的な論理矛盾は見つからなかった。**

一方、詳細設計1.1には、要件が要求する操作・設定を実装へ結び付ける具体的な記述が不足していた。これは要件と反対の設計をしていたという意味の矛盾ではないが、設計のままでは要求を満たしたことを確認できない。以下の項目を詳細設計1.2へ追加して対応した。

## 2. 要件トレーサビリティ

| 要件定義 | 要求 | 詳細設計の対応 | 判定 |
|---|---|---|---|
| 1 | キャラクターのSoft Body / Clothを扱うC++ライブラリ | 全体構成、Soft Body / Cloth Solver、Render Feedback | 合格 |
| 2 | Windows / C++17 / DX11 / HLSL SM5 / Visual Studio | 詳細設計ヘッダー、単位・実行環境、GPU Pass | 合格 |
| 2 | CMakeを必須にしない | Visual Studio .sln / .vcxprojを正規ビルド手段として明記 | 合格 |
| 2 | ランタイム外部物理ライブラリなし | ランタイム物理式を内製し、四面体化器はオフライン工程へ限定 | 合格 |
| 2 | GPU物理、通常時GPU頂点Readbackなし、GPU描画反映 | CPU / GPU責務、GPU Feedback、非同期診断 | 合格 |
| 3 | Physics Mesh / Render Mesh / Collision Proxy | Physics Mesh生成、Render Binding、Collider設計 | 合格 |
| 3 | Soft Body / Cloth / Hair Cards・Bundles、Strand Hair対象外 | 初期方式表とHair対応方針 | 合格 |
| 3 | Meshごとに物理方式を選び、物理なしも許可 | physics_mode = none / soft_body / cloth | 合格 |
| 4.1 | 四面体Soft Body、圧縮・膨張・へこみ・復元 | Corotational FEM、反転ペナルティ、時間積分、試験項目 | 合格 |
| 4.1 | 物理用四面体メッシュ生成と解像度調整 | target_tet_edge_length / target_tet_count / max_tet_count | 合格 |
| 4.1 | 体積保持と元形状復元を独立調整 | volume_retention_scale / shape_recovery_scale、独立エネルギー項 | 合格 |
| 4.2 | ClothのStretch / Shear / Bend / Damping | XPBD各制約、Dampingチャンネル | 合格 |
| 4.2–4.3 | Soft Body / Cloth併用、相互衝突 | 双方向VT / EEと共通接触制約 | 合格 |
| 5 | 複数形式Importer、共通内部表現 | glTF 2.0 / GLB、OBJ、共通Mesh情報 | 合格 |
| 5 | Vertex Group / Bone Weight / 物理属性 | 共通アセットのVertex Group、Skin Weight、Attribute Channel | 合格 |
| 5 | Physics MeshとRender Meshのトポロジー差 | 四面体 / 表面Embedding、境界Binding | 合格 |
| 5 | 生成物キャッシュの保存・ロード | Asset Cacheの内容、Cache Key、無効化条件 | 合格 |
| 5 | 入力不正・生成失敗・対応誤差を報告 | Diagnostic Code、Mesh / Element / Pass識別情報 | 合格 |
| 6 | アニメーションと物理の併用、目標速度 | GPU Skinning、Animation Velocity、Tracking | 合格 |
| 6 | 骨格移動・回転による慣性 | ワールド空間での物理積分と移動目標への追従力 | 合格 |
| 6 | 通常再生時の速度リセット防止 | 通常再生 / Clip Blendでは履歴を維持 | 合格 |
| 6 | Teleport / Reset / Clip切替時の速度引継ぎ | Discontinuity Policy: preserve_velocity / reset_velocity | 合格 |
| 6 | UV / Material等の維持、法線・接線更新 | Render Feedbackで更新対象と維持属性を明記 | 合格 |
| 7 | Soft Body / Cloth全組み合わせと自己衝突 | 双方向VT / EE、self_collision_enabled | 合格 |
| 7 | 双方への接触応答 | 双方頂点の質量・勾配に基づく補正 | 合格 |
| 7 | 初期貫通の最大修正量 | max_initial_repair_distanceと補正予算 | 合格 |
| 7 | CCDまたは同等の高速衝突対策 | Swept AABB、Conservative Advancement、TOI応答 | 合格 |
| 7 | 接触厚み / 摩擦 / 反発 | Contact Data、Coulomb摩擦、速度Impulse | 合格 |
| 7 | Mesh組み合わせ衝突デフォルト有効、個別除外 | Collision Matrix、Mesh Pair除外リスト | 合格 |
| 7 | 全頂点・全三角形総当たりを主方式にしない | Spatial Hash、多段階セル、VT / EE候補生成 | 合格 |
| 7 | Candidate Overflowを黙って破棄しない | Count / Scan / Capacity Check、Block / Retry | 合格 |
| 8 | Mesh基準値とVertex Weight | 基準材料値と0～1チャンネル値 | 合格 |
| 8 | Stretch / Shear / Bend / Volume / Damping / Tracking | Soft Body / Cloth別チャンネル表と制約への反映式 | 合格 |
| 8.1 | Paint / Smooth / Average / Erase | Paint操作、strength、target_value、Heatmap | 合格 |
| 8.1 | Render MeshからPhysics Meshへの値転送 | Surface補間、Soft Body内部調和拡張 | 合格 |
| 8.1 | Physics Mesh再生成後の編集値再利用 | 安定ID・幾何対応、曖昧時の警告 | 合格 |
| 9 | Play / Stop / Pause / Reset / Step | Test Environment操作、固定Step定義 | 合格 |
| 9 | Time Scale / Fixed Delta / Gravity / Impulse | Test Environment設定 | 合格 |
| 9 | 各種物理・衝突・速度・体積等の可視化 | GPU Debug View | 合格 |
| 9 | Pass時間、候補数、接触数、変位、誤差、メモリ | Timestamp / Counter / Run Result JSON | 合格 |
| 9 | 同じ条件の再試験と結果比較 | scenario_hashとRun Result比較 | 合格 |
| 10 | ChatGPTでも編集可能なJSONシナリオ | 可読JSON、schema_version、シナリオ項目 | 合格 |
| 10 | Seed / Asset ID / Generator設定 / Version | random_seed、asset_id、asset_generation_settings、version | 合格 |
| 10 | 欠落・未知バージョン・不正値の検出 | JSONロード検査と既定値ログ | 合格 |
| 11 | GTX 1660 SUPER 6GB / 3～40 FPS目安 | 性能試験条件、規模別Pass計測 | 合格 |
| 11 | GPUメモリを全量占有しない | Buffer / Texture割当量とDXGI使用量・予算の記録 | 合格 |
| 12 | 物理・衝突・描画・性能の受入試験 | T01～T28 | 合格 |
| 13 | 初期版の対象外範囲 | Strand Hair、Morph Target等の初期範囲を明記 | 合格 |
| 14 | 設計レビュー通過後に実装 | 詳細設計と本レビューを実装前ゲートとして維持 | 合格 |

## 3. 詳細設計1.1で見つかった記述不足と対応

これらは直接矛盾ではなく、要件を実装・検証するための詳細化不足だった。

| ID | 不足していた内容 | v1.2での対応 |
|---|---|---|
| RQ-01 | VolumeとShape Recoveryの独立した調整・Paint規則 | 偏差ひずみと体積ひずみの独立エネルギー項、scale値を定義 |
| RQ-02 | 四面体メッシュ解像度とキャッシュ条件 | 目標辺長・目標要素数・生成器設定・Cache Keyを追加 |
| RQ-03 | 全Mesh Pairのデフォルト衝突と除外 | Collision Matrix、Self Collision、Mesh Pair除外を追加 |
| RQ-04 | 初期貫通の総修正量上限 | 頂点ごとの累積修正予算とλ整合を追加 |
| RQ-05 | Animation Discontinuityと速度引継ぎ | preserve_velocity / reset_velocityを定義 |
| RQ-06 | Test Environmentの全操作とシナリオ項目 | Play / Stop / Pause / Reset / Step / Time Scale / Gravity / Impulse、JSONを追加 |
| RQ-07 | Paint値がソルバーへ作用する式 | 制約Compliance、材料Scale、Damping、Trackingへの頂点ウェイト反映式を追加 |
| RQ-08 | デバッグ可視化と試験結果比較 | GPU Debug View、Run Result JSON、scenario_hashを追加 |
| RQ-09 | GPUメモリ予算の計測 | Resource割当量とDXGIのプロセス使用量・OS予算を記録する方針を追加 |
| RQ-10 | 変形対象外のRender Attribute保持 | UV / Color / Material / Indexは元Meshを維持すると明記 |

## 4. 矛盾ではない初期版の範囲制限

以下は初期版の実装範囲を絞る設計判断であり、要件定義1.1と矛盾しない。

1. **Importer**: 初期形式はglTF 2.0 / GLBとOBJ。要件は複数形式を要求するが、全形式対応までは要求していない。
2. **Morph Target**: 初期版対象外。要件定義はMorph Target変形を必須としていない。
3. **Hair**: Hair Cards / Hair BundlesはClothとして扱い、Strand Hairは要件定義でも初期版対象外。
4. **Collision-only Collider**: 初期版はStatic / Kinematic Triangle Mesh。要件は衝突専用オブジェクトを要求するが、動的Colliderの種類やプリミティブ形状までは指定していない。
5. **変形メッシュ用低解像度Collision Proxy**: 初期版はSoft Body境界面とCloth表面を接触形状に使用する。変形Proxyから高解像度Physics Meshへ接触補正を戻す仕組みは将来拡張とする。要件定義はCollision Proxyを用語として定義しているが、この変形Proxy機能を必須受入項目にはしていない。
6. **性能値**: 3～40 FPSは開発時の許容目安として扱い、全規模の保証値にしていない。要件の記述と一致する。

将来、上記の初期版制限を変更する場合は、要件追加または詳細設計の対象範囲変更として記録する。

## 5. 最終判定

**要件定義1.1と詳細設計1.2の整合性レビューは合格。**

- 直接矛盾: 0件
- 要件を満たすために必要だった記述不足: v1.2で対応
- 明示した初期版の範囲制限: 要件定義と矛盾なし
- 実装開始前に残る作業: 係数・容量・閾値の実GPU測定、数値検証、各段階の受入試験

本レビューは設計文書間の整合性判定であり、物理品質や性能が実機で達成済みであることを意味しない。詳細設計18章の実装段階ごとの試験は引き続き必要である。
