# 詳細設計書 初版批判的レビュー記録

- 対象文書: docs/detailed_design.md 1.0
- 対応文書: docs/detailed_design.md 1.1
- 初版レビュー結果: **不合格**
- v1.1再レビュー結果: **合格（段階試験ゲート付き）**
- レビュー対象環境: Direct3D 11 / HLSL Shader Model 5 / C++17
- レビュー方針: 数式の整合性、数値安定性、GPU並列実行、衝突候補の完全性、失敗時の状態管理を重点確認

## 1. 総評

詳細設計書1.0は、Soft Body、Cloth、共通衝突、GPU実行、試験環境までの構成を定義していた。一方、複数の重要な式が異なる物理モデルから混在しており、XPBDの内部状態と位置更新が不整合だった。また、Broadphaseの候補完全性、TOI後の速度処理、Skinningの座標変換など、実装結果を変えてしまう未定義事項が残っていた。

これらは文章上の不足に留まらず、実装すると誤った弾性応答、衝突の見逃し、接触後の再貫通、スキニング位置のずれにつながるため、初版は不合格と判断した。

v1.1では数式・データフロー・エラー処理を修正し、初期版の対象範囲も明確化した。残る項目は主に係数選定と対象GPUでの実測であり、段階的な実装・試験で確定できる状態になった。

## 2. 初版の重大指摘と対応

| ID | 重要度 | 初版の問題 | v1.1での対応 |
|---|---|---|---|
| DD-01 | 致命的 | Corotational FEMの応力式に、別形式の体積項が混在していた | Corotational Linear Elasticityのエネルギー・第一Piola応力を一貫した式へ変更。せん断μ・体積Kを独立化 |
| DD-02 | 高 | 極分解の収束、特異要素、反転四面体の扱いが不十分 | Scaled Newton Polar Iteration、収束検査、SVD補助経路、有限の反転ペナルティを追加 |
| DD-03 | 高 | 半陰的Eulerの安定時間刻みが固定値中心で、材料・要素品質との関係が不明 | 縦波速度と最小四面体高度を用いたCFL型の初期推定、Tracking剛性による追加制約を追加 |
| DD-04 | 致命的 | XPBD Jacobiで位置補正だけを緩和し、λを全量更新していた | Projected Δλ自体へ緩和係数を適用し、λ更新と位置補正に同じ値を使用 |
| DD-05 | 高 | Cloth Shear / Bendの制約式・勾配が不明 | Shear内積ひずみ制約、符号付き二面角、正規化ベクトルとatan2の微分手順を追加 |
| DD-06 | 高 | Complianceの単位、物理材料値からsolver値への変換が不足 | Stretch / Shear / BendのC・剛性k・Compliance alphaの単位と、Rest Geometryに基づく変換式を追加 |
| DD-07 | 高 | VT / EEの双方への質量配分、摩擦・反発が曖昧 | VT重心座標、EE線分パラメータに基づく双方への勾配・質量配分と、速度Impulse方式を定義 |
| DD-08 | 致命的 | 粒子の1セル登録だけでは隣接セルや接触厚みを含む候補を取りこぼす可能性があった。大型要素処理も未定義 | 多段階Spatial Hash、膨張AABB、異なるlevel間の検索、整数セル座標照合、候補完全性試験を追加 |
| DD-09 | 高 | 可変長GPUバッファ超過時に、同じ物理Stepを安全に再実行する手順がなかった | Count / Scan / Capacity Check、simulation_status、Block / Retry、確定状態スナップショット、同一logical stepの再実行を追加 |
| DD-10 | 致命的 | CCDが区間細分化中心で、速度上限・TOI・時間単位が不明だった | Swept AABB + Conservative Advancement、秒単位TOI、進行・距離許容値、最大反復・分割上限を定義 |
| DD-11 | 致命的 | 四面体内部頂点のBone Weightを生成する方法がなかった | 境界補間と内部グラフLaplacian調和拡張を追加 |
| DD-12 | 高 | Render Bindingの境界外offsetの座標系が曖昧 | Rest Pose局所基底と変形後局所基底によるoffset変換を明記 |
| DD-13 | 高 | glTFスキニング行列にMesh Node変換が含まれていなかった | inverse(mesh_node_global) * joint_global * inverse_bind_matrixへ修正 |
| DD-14 | 高 | Collision-only Colliderと変形メッシュProxyの初期対応範囲が曖昧 | 初期版はStatic / Kinematic Triangle Meshをサポートし、変形ProxyのPhysics Meshへの応答写像は将来拡張へ分離 |
| DD-15 | 中 | D3D11のSRV / UAV View切替、GPUリソースの読書き責務が曖昧 | Buffer表とPass境界のUAV unbind / SRV bindを追加。浮動小数点Atomic Addを使用しない方針を明記 |
| DD-16 | 中 | 接触点の重複、初期貫通、Soft Body内部へのペイント属性転送が未定義 | Contact Patch重複抑制、初期接触の段階修正、Soft Body内部への属性調和拡張を追加 |

## 3. v1.1で特に確認した数式・方式

### 3.1 Corotational FEM

v1.1では、ひずみエネルギーと第一Piola応力の組を一致させた。

    epsilon_corot = transpose(R) * F - I

    Psi = mu * norm(F - R)^2
         + 0.5 * lambda_lame * trace(epsilon_corot)^2

    P = 2 * mu * (F - R)
      + lambda_lame * trace(epsilon_corot) * R

Lameの第一定数はlambda_lame = K - 2*mu/3とし、UIのμ・Kを別々に調整できる。初版の(J-1)*cofactor(F)をCorotational Linear Elasticityの体積応力として使用する誤りを修正した。

### 3.2 XPBD Jacobi

v1.1では、λの射影後の差分に緩和係数を適用し、同じ差分でλと位置を更新する。位置補正だけを緩和する方式を修正した。

### 3.3 Broadphase

v1.1では、小さいPrimitiveが同じlevelから粗いlevelまでを検索する構造とし、接触厚みを含むAABBの全セルを照合する。Hash値に加えてlevelと整数セル座標を比較する。小規模全探索を参照実装として、Broadphaseが確定接触候補を漏らさないことを検証する。

### 3.4 CCD

v1.1では、Conservative Advancementの進行量を距離ギャップと相対速度上限から求める。TOIは秒単位で扱い、TOI時刻で接触Impulseを一度適用してから残り時間を再積分する。Impulse済み接触をStep終端でも重複適用しない。

## 4. v1.1再レビュー結果

| 確認項目 | 判定 | コメント |
|---|---|---|
| Corotational応力・材料定義 | 合格 | 応力式と材料パラメータの対応が一貫 |
| 反転・特異四面体 | 合格 | 通常極分解経路とSVD補助経路、ペナルティを分離 |
| Soft Body時間積分 | 合格 | CFL型の推定を追加。数学的な安定保証ではないことも明記 |
| Cloth Stretch / Shear / Bend | 合格 | 制約式と勾配の算出方針を明記 |
| XPBD λ / Gather Jacobi | 合格 | 緩和後のλと位置更新の整合を確保 |
| 双方向VT / EE接触 | 合格 | 双方の頂点に補正を配分する式を明記 |
| 摩擦・反発 | 合格 | 位置拘束と速度Impulseを分離 |
| Contact Patch | 合格 | 近接重複の抑制ルールと試験項目を追加 |
| Spatial Hash | 合格 | 多段階セルと候補完全性試験を定義 |
| Overflow | 合格 | 未Commit、Block / Retry、同一Step再実行を定義 |
| CCD / TOI | 合格 | 時間単位、速度上限、TOI時Impulse、残時間処理を定義 |
| Skinning | 合格 | 内部頂点ウェイト生成とglTF Skin Matrixを定義 |
| Render Binding | 合格 | 境界外offsetの座標系と誤差検査を定義 |
| Collision-only Collider | 合格 | Static / Kinematic Triangle Meshの初期範囲を定義 |
| GPUリソース・計測 | 合格 | SRV / UAV、Pass同期、GPU Timestampを定義 |

## 5. 段階試験で確定する残項目

以下は方式未定義ではなく、対象GPU・アセットで校正するパラメータと試験項目である。初期値を達成済みと扱わず、測定結果を保存して確定する。

1. Corotational Linear Elasticityの適用可能なひずみ範囲。大変形で品質不足があればNeo-Hookean等と比較する。
2. SVD補助Passの反復精度、反転ペナルティの係数、反転要素の診断閾値。
3. μ / K / 密度、Animation Tracking stiffness / dampingの代表プリセットと時間刻み。
4. Cloth E_2D / G_2D / Bend rigidityのプリセットとメッシュ解像度変更時のエネルギー誤差。
5. Bend解析勾配と有限差分勾配の一致。相対誤差1e-3を初期目標とする。
6. Spatial Hashのbase cell width、level数、max_cells_per_primitive、各候補Bufferの容量。
7. Contact Patchのcluster radius・法線角度閾値、Contact数削減率と最大貫通の関係。
8. Conservative Advancementのsafety、距離許容値、最小進行時間、TOI分割上限。
9. GTX 1660 SUPER 6GBでの実使用可能メモリ、各PassのGPU時間、代表規模のスケーリング。
10. glTF / GLBのSkinning、Mesh Node Transform、STEP / LINEAR / CUBICSPLINEを含むテストアセット。

これらの値は実測なしに保証できないため、実装各段階の完了条件として扱う。

## 6. 最終判定

**詳細設計書1.1は合格。**

初版にあった物理式の不整合、GPU候補の欠落可能性、XPBD内部状態の不整合、TOI後の速度再利用の不足は修正された。残項目は段階試験による数値校正と適用範囲の確認である。

したがって、設計レビュー工程は通過した。実装開始はユーザーの明示指示後とし、実装順序は詳細設計書18章に従う。初期段階ではAsset / Test Environmentとソルバー単体検証を先行し、各段階の受け入れ試験を通過してから次へ進む。
