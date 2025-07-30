# MetaLimbs Gazebo Simulation プロジェクト構成

## 最終更新: 2025-07-30

---

## 進捗まとめ

### 2025-07-30 実施内容

#### 完了した作業

1. **初期問題の解決**
   - 問題: Gazeboでロボットが白く表示される（マテリアル情報なし）
   - 原因: OBJファイルに対応するMTLファイルが存在しない
   - 解決: Pythonスクリプトでパーツごとに色を動的に追加

2. **ロボットの向きの調整**
   - 問題: ロボットアームが垂直に立っている
   - 解決: pose設定で回転（Roll: 3.1416, Yaw: -1.5708）

3. **人間モデルとの統合**
   - Standing personモデルをIgnition Fuelから読み込み
   - launch_ign_standing_person.shを作成

4. **カメラビューの設定**
   - MinimalSceneプラグインでGUI設定
   - 正面からの視点を実現

#### 現在の状態
- Standing personと一緒にロボットが表示される
- ロボット位置: X=0, Y=0, Z=1.5
- カメラ: 正面3mから撮影

#### 今後の課題
- ODEワーニングの解決
- ros2_controlの実装
- 人間モデルとロボットの物理的接続

---

## 概要
このプロジェクトは、MetaLimbsロボットアームをIgnition Gazebo（Gazebo）でシミュレーションするためのROS2パッケージです。

## ディレクトリ構成

```
metalimbs-ros2-ws/
├── src/
│   ├── metalimbs_description/      # ロボット記述パッケージ
│   │   ├── robot/                  # URDFファイル
│   │   ├── meshes/                 # 3Dメッシュファイル
│   │   └── launch/                 # ROS2 launchファイル
│   └── metalimbs_gazebo_bringup/   # Gazebo起動パッケージ
├── launch_*.sh                     # Gazebo起動スクリプト
└── setup_gazebo_env.sh            # 環境設定スクリプト
```

## 主要ファイル説明

### URDFファイル（src/metalimbs_description/robot/）

- **metalimbs2.urdf.xacro** - メインのロボット記述ファイル
- **metalimbs2_fixed.urdf.xacro** - 固定ジョイント版（シミュレーション用）
- **metalimbs2_gazebo.urdf.xacro** - Classic Gazebo用設定
- **metalimbs2_ign_corrected.urdf.xacro** - Ignition Gazebo用（ジョイント構造に問題あり）

### 起動スクリプト

#### 1. launch_ign_original.sh
- **用途**: 基本的なロボット起動（白色表示）
- **特徴**: OBJローダーのワーニングが大量に出る
- **使用URDF**: metalimbs2_fixed.urdf.xacro

#### 2. launch_ign_colored.sh
- **用途**: パーツごとに色分けされたロボット表示
- **特徴**: 
  - 各パーツが異なる色で表示（識別しやすい）
  - OBJローダーのワーニング抑制機能付き
- **使用URDF**: metalimbs2_fixed.urdf.xacro

#### 3. launch_ign_with_robot.sh
- **用途**: 部品がバラバラになる問題の確認用
- **特徴**: metalimbs2_ign_corrected.urdf.xacroを使用（問題あり）
- **注意**: ロボットがバラバラになるため実用的ではない

#### 4. launch_ign_human_selection.sh ⭐推奨
- **用途**: 人間モデルと一緒にロボットを表示
- **特徴**:
  - インタラクティブに人間モデルを選択可能
  - ロボットは180度回転して背負っているように表示
  - 10種類の人間モデルから選択可能
- **使用URDF**: metalimbs2_fixed.urdf.xacro

### 環境設定ファイル

#### setup_gazebo_env.sh
- Ignition Gazebo用の環境変数設定
- すべての起動スクリプトから呼び出される

## 使用方法

### 基本的な起動（色付き）
```bash
./launch_ign_colored.sh
```

### 人間モデルと一緒に表示
```bash
./launch_ign_human_selection.sh
# 番号を選択（1-10）
```

## 技術的なポイント

### 座標系について
- **ROS/Gazebo標準**: Z軸が上向き（重力と反対）
- **メッシュファイル**: `mesh_rpy="${PI_2} ${PI} -${PI_2}"`で回転補正
- **ロボット配置**: 人間の背中に配置する際は`pose="0 -0.2 1.4 0 0 3.14159"`

### 色の設定
Pythonスクリプトでパーツごとに以下の色を設定：
- base_mount: グレー
- shoulder: 青系
- upper_arm: 赤系
- elbow: 緑系
- forearm: 黄系
- wrist: 紫系
- handunit: 紺色系

### 問題と解決策

1. **OBJローダーワーニング**
   - 原因: OBJファイルに対応するMTLファイルが存在しない
   - 解決: スクリプト内でワーニングをフィルタリング

2. **ロボットがバラバラになる問題**
   - 原因: metalimbs2_ign_corrected.urdf.xacroのジョイント親子関係が不適切
   - 解決: metalimbs2_fixed.urdf.xacroを使用

3. **向きの問題**
   - 原因: メッシュファイルの座標系とROSの座標系の違い
   - 解決: mesh_rpyパラメータで補正済み

## 今後の改善点

1. MTLファイルの作成によるワーニングの根本的解決
2. ros2_controlの実装による関節制御
3. より詳細な人間モデルの統合

## 依存関係

- ROS2 Humble
- Ignition Gazebo (Fortress)
- xacro
- Python3

## ビルド方法

```bash
cd metalimbs-ros2-ws
colcon build --packages-select metalimbs_description metalimbs_gazebo_bringup
source install/setup.bash
```