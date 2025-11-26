#include <iostream>
#include "math.h"
//#include <pcl/io/hdl_grabber.h>
#include <pcl/io/pcd_io.h>
#include <boost/thread/thread.hpp>
#include <pcl/registration/transformation_estimation_3point.h>
//#include <Windows.h>
#include <stdlib.h>

// 永井追加
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include "GroundSegmentation.h"
#include <limits>
// 永井追加ここまで

//#pragma comment(lib,"winmm.lib")

using namespace std;

//// we use the 2D and 3D SLAM types here
//G2O_USE_TYPE_GROUP(slam2d);
//G2O_USE_TYPE_GROUP(slam3d);

typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 7, 1> Vector7d;//(scan_id,x,y,z,roll,pitch,yaw)

// ===== 永井追加：段差検出改善用 =====

// Union-Find（平面マージ用）
class UnionFind {
private:
	std::vector<int> parent, rank;
public:
	UnionFind(int n) : parent(n), rank(n, 0) {
		for (int i = 0; i < n; ++i) parent[i] = i;
	}
	int find(int x) {
		if (parent[x] != x) parent[x] = find(parent[x]);
		return parent[x];
	}
	void unite(int x, int y) {
		int rx = find(x), ry = find(y);
		if (rx == ry) return;
		if (rank[rx] < rank[ry]) std::swap(rx, ry);
		parent[ry] = rx;
		if (rank[rx] == rank[ry]) rank[rx]++;
	}
};

struct ClusterGeometry {
	// 矩形面積と点密度
	float rect_area;           // 四隅点からの四辺形面積 [m^2]
	float point_density;       // 点密度 [points/m^2]
	float actual_area;         // 実面積（凸包） [m^2]

	// 固有値（実値）
	float lambda1_raw;         // 最大固有値
	float lambda2_raw;         // 中間固有値
	float lambda3_raw;         // 最小固有値

	// 固有値（正規化: λi / sum(λ))
	float lambda1_norm;
	float lambda2_norm;
	float lambda3_norm;

	Eigen::Vector3f eigenvector1;  // 第一主成分方向（最大固有値）
	Eigen::Vector3f eigenvector2;  // 第二主成分方向
	Eigen::Vector3f eigenvector3;  // 第三主成分方向（= 法線）

	// 形状特徴量（実値ベース）
	float linearity_raw;       // (λ1 - λ2) / λ1
	float planarity_raw;       // (λ2 - λ3) / λ1
	float sphericity_raw;      // λ3 / λ1
	float anisotropy_raw;      // (λ1 - λ3) / λ1
	float omnivariance_raw;    // (λ1 * λ2 * λ3)^(1/3)
	float surface_variation_raw; // λ3 / (λ1 + λ2 + λ3)

	// 形状特徴量（正規化ベース）
	float linearity_norm;
	float planarity_norm;
	float sphericity_norm;
	float anisotropy_norm;
	float omnivariance_norm;
	float surface_variation_norm;

	// スケール情報
	float eigenvalue_sum;          // λ1 + λ2 + λ3
	float eigenvalue_condition;    // λ1 / λ3
	float structural_tensor_norm;  // sqrt(λ1² + λ2² + λ3²)
	float eigenvalue_ratio_21;     // λ2 / λ1
	float eigenvalue_ratio_32;     // λ3 / λ2

	// 法線統合指標
	float oriented_planarity;      // planarity × |nz|
	float horizontal_confidence;   // planarity × (1-|nz|)
	float normal_eigen_alignment;  // |n · v_min|

	// 段差検出用複合スコア
	float step_likelihood_score;
	float weighted_planarity;      // (λ2 - λ3) × sqrt(λ1)

	// 座標中央値
	float median_x;  // x座標の中央値
	float median_y;  // y座標の中央値
	float median_z;  // z座標の中央値

	// ===== ★追加：踏面判定用の形状特徴量★ =====
	float length;              // 第一主成分方向の長さ [m]
	float width;               // 第二主成分方向の長さ [m]
	float aspect_ratio;        // アスペクト比 (length / width)
	float fill_ratio;          // 充填率 (actual_area / rect_area)
	float rectangularity;      // 矩形度 (actual_area / (length * width))

	// バウンディングボックス情報
	float bbox_min_x, bbox_max_x;
	float bbox_min_y, bbox_max_y;
	float bbox_min_z, bbox_max_z;

	ClusterGeometry()
		: rect_area(0), point_density(0), actual_area(0),
		lambda1_raw(0), lambda2_raw(0), lambda3_raw(0),
		lambda1_norm(0), lambda2_norm(0), lambda3_norm(0),
		length(0), width(0), aspect_ratio(0), fill_ratio(0), rectangularity(0),
		bbox_min_x(0), bbox_max_x(0),
		bbox_min_y(0), bbox_max_y(0),
		bbox_min_z(0), bbox_max_z(0),
		linearity_raw(0), planarity_raw(0), sphericity_raw(0),
		anisotropy_raw(0), omnivariance_raw(0), surface_variation_raw(0),
		linearity_norm(0), planarity_norm(0), sphericity_norm(0),
		anisotropy_norm(0), omnivariance_norm(0), surface_variation_norm(0),
		eigenvalue_sum(0), eigenvalue_condition(0), structural_tensor_norm(0),
		eigenvalue_ratio_21(0), eigenvalue_ratio_32(0),
		oriented_planarity(0), horizontal_confidence(0), normal_eigen_alignment(0),
		step_likelihood_score(0), weighted_planarity(0),
		median_x(0), median_y(0), median_z(0),
		eigenvector1(Eigen::Vector3f::UnitX()),
		eigenvector2(Eigen::Vector3f::UnitY()),
		eigenvector3(Eigen::Vector3f::UnitZ()) {
	}
};

// 段差クラスタ構造体

// StepCluster 構造体に geometry メンバを追加
struct StepCluster {
	std::vector<int> indices;
	float nx, ny, nz, d;
	float cx, cy, cz;
	ClusterGeometry geometry; // ← 追加

	StepCluster() : nx(0), ny(0), nz(1), d(0), cx(0), cy(0), cz(0) {}
};


struct CandidateInfo {
	StepCluster cluster;
	bool is_step;
	bool ground_checked;
	float ground_gap_med;
	CandidateInfo() : is_step(false), ground_checked(false), ground_gap_med(std::numeric_limits<float>::quiet_NaN()) {}
};

class Save {
private:
	//std::string inputFilenamePrefix; //※尾崎追加　 サブマップのファイル名プレフィックス
	std::string inputFilename;
	std::string pose_file_path;
	std::string pcd_file_path;
	std::string pcd_file;
	////※↓尾崎追加
	//int scan_count;
	//Eigen::Matrix<double, 7, 1> robot_pose;

	//Eigen::Matrix<double, 7, 1> Quaternion_to_EularAngle(std::vector<std::string> v);
	////※↑尾崎追加

	std::string submap_output_base = "Output_Submaps"; //永井追加．逐次マップ出力パス

	// 永井追加．連続加算サブマップ用
	pcl::PointCloud<pcl::PointXYZI> submap_active_; // 世界座標で蓄積
	int submap_window_ = 33;      // サブマップに使うスキャン数
	int submap_count_ = 0;        // たまってるスキャン数
	int submap_start_id_ = -1;    // 現サブマップの開始スキャンID
	int submap_end_id_ = -1;      // 現サブマップの終了スキャンID

	// 永井追加．車体→センサ（固定外部パラメタ）
	Eigen::Matrix4f T_vs_ = Eigen::Matrix4f::Identity();
	bool has_T_vs_ = false;

	// 永井追加．先頭スキャンの World←Sensor
	Eigen::Matrix4f T_ws0_ = Eigen::Matrix4f::Identity();
	bool has_T_ws0_ = false;

	int submap_stride_ = 1;
	std::vector<pcl::PointCloud<pcl::PointXYZI> > submap_scans_;
	std::vector<int> submap_scan_ids_;
	std::vector<Eigen::Matrix4f> submap_T_wr_;


	// 最新サブマップを確定して出力・分類・累積する用
	void finalize_active_submap_(
		patchwork::Ground_Segmentation& GS,
		pcl::PointCloud<pcl::PointXYZRGB>& road_map_accum,
		pcl::PointCloud<pcl::PointXYZRGB>& solid_map_accum,
		pcl::PointCloud<pcl::PointXYZRGB>& wall_map_accum,
		pcl::PointCloud<pcl::PointXYZRGB>& float_map_accum
	);

	// 段差検出パラメータ
	float step_max_xy_distance_ = 10.0f;  // XY平面上の最大距離 [m]
	float MAX_LAMBDA2_LENGTH = 0.6f;  // [m] 第二主成分長の上限

	// ================== 永井追加：局所地図バッチ処理用 ==================

	// サブマップ1つ分のメタデータ構造体
	struct SubmapMetadata {
		std::string pcd_path;           // 保存したpcdのパス (all_colored)
		Eigen::Matrix4f T_ws0;          // NDT姿勢（Robot->World）
		Eigen::Matrix3f R_level;        // 水平化に使用した回転行列 (Sensor->Level)
		Eigen::Vector3f ground_normal;  // そのサブマップで推定された路面法線
		int start_id;
		int end_id;
	};

	std::vector<SubmapMetadata> submap_batch_list_; // バッチ処理待ちのリスト
	int batch_size_ = 100; // 何個溜まったら局所地図を作るか

	// バッチ処理実行関数
	void process_submap_batch();

	// ベクトルaをベクトルbに合わせる回転行列を計算するヘルパー
	Eigen::Matrix3f calc_rotation_from_vectors(const Eigen::Vector3f& from, const Eigen::Vector3f& to);

	// ====================================================================

public:
	Save();
	void initial_setting(std::string file_path);
	//クォータニオン→オイラー角変換　(g2o→姿勢[m][rad]変換)
	Eigen::Matrix<double, 7, 1> Quaternion_to_EulerAngle(std::vector<std::string> v);
	//姿勢の出力
	void generate_csv();
	//地図の出力
	void generate_map(int start_scan, int end_scan, int Matchig_NUM);

	float Intensity = 20;
	int scan_count;//地図のスキャン数
	//Eigen::Matrix<double, 7, 1> robot_pose[1000];//(☆)地図の姿勢
	std::map<int, Vector7d> robot_pose; //地図の姿勢
	Eigen::Matrix<double, 7, 1> Matching_pose[10];//☆☆マップマッチングに使用する姿勢

	Eigen::Affine3d affine_trans;

	// 永井追加．pcdと出力先パス切り替え
	void set_pcd_base(const std::string& dir) { pcd_file_path = dir; }
	void set_output_base(const std::string& dir) { submap_output_base = dir; }

	// 永井追加．NDTSMの結果保存
	void set_pose_from_ndt(int scan_id, double x, double y, double z, double roll, double pitch, double yaw) {
		if (scan_id < 0 || scan_id >= 1000) return;
		robot_pose[scan_id](0) = scan_id;
		robot_pose[scan_id](1) = x;
		robot_pose[scan_id](2) = y;
		robot_pose[scan_id](3) = z;
		robot_pose[scan_id](4) = roll;  // rad
		robot_pose[scan_id](5) = pitch; // rad
		robot_pose[scan_id](6) = yaw;   // rad
	}

	// 永井追加． 10スキャン逐次マップ生成→GPFで分類→色別PCD出力＆全体カテゴリ地図に追加
	void generate_and_classify_submap(
		int start_scan,
		int Matchig_NUM,
		patchwork::Ground_Segmentation& GS,
		pcl::PointCloud<pcl::PointXYZRGB>& road_map_accum,
		pcl::PointCloud<pcl::PointXYZRGB>& solid_map_accum,
		pcl::PointCloud<pcl::PointXYZRGB>& wall_map_accum,
		pcl::PointCloud<pcl::PointXYZRGB>& float_map_accum
	);

	void set_submap_window(int n) { submap_window_ = std::max(1, n); }
	void set_submap_stride(int n) { if (n < 1) n = 1; submap_stride_ = n; }
	// cloud_robot: ロボット座標の1スキャン点群, T_wr: ロボット→世界の変換（NDTの結果）
	void submap_add_scan(int scan_id,
		const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_robot,
		const Eigen::Matrix4f& T_wr);
	// 窓幅に達したら確定して出力・分類・累積（達していなければ何もしない）
	void submap_finalize_if_ready(
		patchwork::Ground_Segmentation& GS,
		pcl::PointCloud<pcl::PointXYZRGB>& road_map_accum,
		pcl::PointCloud<pcl::PointXYZRGB>& solid_map_accum,
		pcl::PointCloud<pcl::PointXYZRGB>& wall_map_accum,
		pcl::PointCloud<pcl::PointXYZRGB>& float_map_accum
	);
	// 強制確定（アプリ終了時などに使用）
	void submap_force_finalize(
		patchwork::Ground_Segmentation& GS,
		pcl::PointCloud<pcl::PointXYZRGB>& road_map_accum,
		pcl::PointCloud<pcl::PointXYZRGB>& solid_map_accum,
		pcl::PointCloud<pcl::PointXYZRGB>& wall_map_accum,
		pcl::PointCloud<pcl::PointXYZRGB>& float_map_accum
	);

	// 永井追加．車体→センサの外部パラメタ（固定）を設定
	// 引数は 4x4 の同次変換（Vehicle←Sensor）
	void set_vehicle_to_sensor(const Eigen::Matrix4f& T_vs) {
		T_vs_ = T_vs;
		has_T_vs_ = true;
	}

	// パラメータ設定関数
	void set_step_max_xy_distance(float dist) {
		step_max_xy_distance_ = std::max(0.0f, dist);
	}
	float get_step_max_xy_distance() const {
		return step_max_xy_distance_;
	}

	// バッチサイズの変更用セッター
	void set_batch_size(int size) { batch_size_ = size; }

	// 終了時に残っているバッチを強制出力する関数
	void flush_submap_batch() {
		if (!submap_batch_list_.empty()) {
			process_submap_batch();
		}
	}

};