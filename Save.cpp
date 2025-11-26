#include"Save.h"
#include "Logging.h"//永井追加
#include "SubmapMovie.h"//永井追加
#include <direct.h>//永井追加
#include <pcl/io/hdl_grabber.h>
#include <pcl/common/common.h>//　永井追加
#include <pcl/common/centroid.h>
#include <pcl/kdtree/kdtree_flann.h>//　永井追加．段差検出アルゴリズム用
#include <pcl/filters/voxel_grid.h>
#include <queue>    // BFS用
#include <set>      // 連結成分管理用
#include <tuple>    // セクタキー用
#include <map>      // ヒストグラム用
#include <unordered_map> //　永井追加
#include <unordered_set> //　永井追加
#include <algorithm>
#include <cmath>
#include <limits>
#include <fstream>
#include <string>
#include <sstream>
#include <iomanip>

//　永井追加．競合回避用にヘッダファイルから移籍
#define NOMINMAX
#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#include <mmsystem.h>
#pragma comment(lib,"winmm.lib")
#pragma comment(lib,"Ws2_32.lib")

extern double MAP_HEIGHT_MAX;
// 除外点を記録するグローバル配列
static std::vector<int> global_rejected_indices;

using namespace submap_movie;

Save::Save() {

}

void Save::initial_setting(std::string file_path) {
	//入力ファイルのパス
	inputFilename = "D:/Databox/submap/Output/Pose_graph_optimized.g2o";	//☆プログラムの出力g2oファイル(最適化) Output_Data/Pose_graph_optimized.g2o
	pcd_file_path = file_path + "D:/Databox/submap/Input/Output_PCD_路面なし";	//☆☆pcdファイルが存在するフォルダへのパス　Output_PCD  Input_Data(0-740)路面あり/Output_PCD"
	//出力ファイスのパス
	pose_file_path = "D:/Databox/submap/Output/Optimized_pose.csv";	//☆csvファイルを出力するフォルダ Output_Data/Optimized_pose.csv
}

//クォータニオン→オイラー角変換　(g2o→姿勢[m][rad]変換)
Eigen::Matrix<double, 7, 1> Save::Quaternion_to_EulerAngle(std::vector<std::string> v) {
	auto to_d = [](const std::string& s)->double {
		try { return std::stod(s); }
		catch (...) { return 0.0; }
	};

	Vector7d pose = Vector7d::Zero();

	if (v.size() >= 8) {
		// 想定: 0:id 1:x 2:y 3:z 4:qx 5:qy 6:qz 7:qw
		double scan = to_d(v[0]);
		double x = to_d(v[1]);
		double y = to_d(v[2]);
		double z = to_d(v[3]);
		double qx = to_d(v[4]);
		double qy = to_d(v[5]);
		double qz = to_d(v[6]);
		double qw = to_d(v[7]);

		Eigen::Quaterniond q(qw, qx, qy, qz);
		q.normalize();
		Eigen::Vector3d eul = q.toRotationMatrix().eulerAngles(2, 1, 0); // ZYX: yaw, pitch, roll

		pose(0) = scan;
		pose(1) = x;
		pose(2) = y;
		pose(3) = z;
		pose(4) = eul[2]; // roll
		pose(5) = eul[1]; // pitch
		pose(6) = eul[0]; // yaw
		return pose;
	}

	// 別フォーマットの保険: [id, x,y,z, roll, pitch, yaw] だった場合（既にEuler）
	if (v.size() >= 7) {
		pose(0) = to_d(v[0]);
		pose(1) = to_d(v[1]);
		pose(2) = to_d(v[2]);
		pose(3) = to_d(v[3]);
		pose(4) = to_d(v[4]); // roll
		pose(5) = to_d(v[5]); // pitch
		pose(6) = to_d(v[6]); // yaw
		return pose;
	}

	// 不足時もゼロで必ずreturn（ビルドを止めないため）
	return pose;
}

//csvファイルの生成
void Save::generate_csv() {
	Eigen::Matrix<double, 7, 1> pose_tmp;
	scan_count = 0;

	//csvファイルに最終的な自己姿勢を書き出し
	std::ofstream ofs;
	ofs.open(pose_file_path, std::ios::out);
	ofs << "scan,x,y,z,roll,pitch,yaw" << std::endl;


	//姿勢情報の読み込み
	std::cout << "file path:" << inputFilename << std::endl;
	std::ifstream ifs_pose(inputFilename, std::ios::in);
	if (!ifs_pose)
		std::cout << "error:posedata" << std::endl;	// File open failed
	std::string buf_pose;
	while (ifs_pose && std::getline(ifs_pose, buf_pose)) {
		std::vector<std::string> v;
		boost::algorithm::split(v, buf_pose, boost::is_any_of(" "));
		if (v.size() < 1)
			continue;

		if (v[0] == "VERTEX_SE3:QUAT") {
			pose_tmp = Quaternion_to_EulerAngle(v);
			//GNSS制約を除くため
			if (pose_tmp(0) < 50000) {
				scan_count = pose_tmp(0);//見るスキャン数合わせる
				robot_pose[scan_count] = pose_tmp;
				ofs << robot_pose[scan_count](0) << "," << robot_pose[scan_count](1) << "," << robot_pose[scan_count](2) << "," << robot_pose[scan_count](3) << ","
					<< robot_pose[scan_count](4) << "," << robot_pose[scan_count](5) << "," << robot_pose[scan_count](6) << std::endl;
				scan_count++;
			}
		}
	}
	std::cout << "Output csv finished" << std::endl;
}

//地図の生成
void Save::generate_map(int node, int end_scan, int Matchig_NUM) {
	int i;
	int j;
	int scan_id;
	int count;//map_cloud数カウント


	pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud2(new pcl::PointCloud<pcl::PointXYZI>);
	Eigen::Affine3d affine_trans_inverse;

	map_cloud->clear();//ココにはなくてもよし
	//std::cout << node << std::endl;

	for (int i = node; i < node + Matchig_NUM; i++) {
		std::cout << robot_pose[i](0) << std::endl;
		affine_trans = Eigen::Translation3d(robot_pose[i](1), robot_pose[i](2), robot_pose[i](3))
			* Eigen::AngleAxisd(robot_pose[i](6), Eigen::Vector3d::UnitZ())
			* Eigen::AngleAxisd(robot_pose[i](5), Eigen::Vector3d::UnitY())
			* Eigen::AngleAxisd(robot_pose[i](4), Eigen::Vector3d::UnitX());

		scan_id = robot_pose[i](0);
		pcd_file = pcd_file_path + "/cloud_" + std::to_string(scan_id) + ".pcd";//(☆)

		pcl::io::loadPCDFile(pcd_file, *input_cloud);
		pcl::transformPointCloud(*input_cloud, *input_cloud, affine_trans.matrix().cast<float>());
		*map_cloud += *input_cloud;
		count++;


		if (count == 10) {//☆5scan毎にmap出したい
			char file_name_pcd[50];
			sprintf(file_name_pcd, "D:/Databox/submap/Output/路面なし/cloud_%d.pcd", node - 100);//☆ Output_Data/cloud_%d.pcd
			affine_trans_inverse = affine_trans.inverse();
			pcl::transformPointCloud(*map_cloud, *map_cloud, affine_trans_inverse.matrix().cast<float>());//世界座標系→センサ座標系

			pcl::io::savePCDFileBinary(file_name_pcd, *map_cloud);
			*map_cloud2 += *map_cloud;
			map_cloud->clear();

			break;
		}
	}


	//char file_name_pcd[50];
	//sprintf(file_name_pcd, "Output_Data/map_%d.pcd", node + (Matchig_NUM - 1));
	//pcl::io::savePCDFileBinary(file_name_pcd, *map_cloud);
	//


	//*map_cloud2 += *map_cloud;
	//map_cloud->clear();


	pcl::io::savePCDFileBinary("D:/Databox/submap/Output/map.pcd", *map_cloud2);//☆最終map(最適化後)  Output_Data/map.pcd
	std::cout << "Output Map finished" << std::endl;
}

// 以下永井追加
// 追加: インデックス配列をキー配列でソートする簡易クイックソート（const,auto,static,do不使用）
void swap_int(int* a, int* b) {
	int t = *a;
	*a = *b;
	*b = t;
}
void quicksort_indices_by_key(int* idx, float* key, int left, int right) {
	int i = left;
	int j = right;
	float pivot = key[idx[(left + right) / 2]];
	while (i <= j) {
		while (key[idx[i]] < pivot) i++;
		while (key[idx[j]] > pivot) j--;
		if (i <= j) {
			swap_int(&idx[i], &idx[j]);
			i++;
			j--;
		}
	}
	if (left < j) quicksort_indices_by_key(idx, key, left, j);
	if (i < right) quicksort_indices_by_key(idx, key, i, right);
}

// 追加: 単純なPCAで平面推定（最小固有値の固有ベクトルを法線に採用）。返り値: 0=失敗,1=成功
int estimate_plane_from_indices(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
	std::vector<int>& indices,
	float* nx, float* ny, float* nz, float* d,
	float* mean_z, float* cx, float* cy, float* cz) {

	int n = (int)indices.size();
	if (n < 3) return 0;

	Eigen::MatrixXf M(n, 3);
	int r = 0;
	for (int i = 0; i < n; i++) {
		pcl::PointXYZI p = (*cloud)[indices[i]];
		M(r, 0) = p.x; M(r, 1) = p.y; M(r, 2) = p.z;
		r++;
	}
	Eigen::Vector3f meanv = M.colwise().mean();
	Eigen::MatrixXf C = M.rowwise() - meanv.transpose();
	Eigen::Matrix3f cov = (C.adjoint() * C) / (float)(n - 1);

	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(cov);
	if (solver.info() != Eigen::Success) return 0;
	Eigen::Vector3f normal = solver.eigenvectors().col(0);
	if (normal(2) < 0) normal = -normal;
	float nnorm = sqrt(normal(0) * normal(0) + normal(1) * normal(1) + normal(2) * normal(2));
	if (nnorm == 0.0f) return 0;
	normal = normal / nnorm;

	*nx = normal(0);
	*ny = normal(1);
	*nz = normal(2);
	*cx = meanv(0);
	*cy = meanv(1);
	*cz = meanv(2);
	*mean_z = meanv(2);
	*d = -((*nx) * (*cx) + (*ny) * (*cy) + (*nz) * (*cz));
	return 1;
}

// 点と平面の符号付き距離（法線は正規化済みを想定）
float point_plane_dist(float x, float y, float z, float nx, float ny, float nz, float d) {
	return nx * x + ny * y + nz * z + d;
}

// === 追加ヘルパ: 平面のz値と対称高さ差の判定 ===
// 平面: nx*x + ny*y + nz*z + d = 0 → z = -(nx*x + ny*y + d)/nz
static inline bool plane_z_at(float nx, float ny, float nz, float d, float x, float y, float& z_out) {
	if (std::fabs(nz) < 1e-6f) return false;  // nz 極小は不可
	z_out = -(nx * x + ny * y + d) / nz;
	return true;
}

// 平面の高さ差の対称チェック（双方の重心で ±MERGE_HEIGHT_TOL 以内）
static inline bool plane_height_diff_ok(const StepCluster& A, const StepCluster& B, float tol /*=MERGE_HEIGHT_TOL*/) {
	float zaA, zbA, zaB, zbB;
	if (!plane_z_at(A.nx, A.ny, A.nz, A.d, A.cx, A.cy, zaA)) return false;
	if (!plane_z_at(B.nx, B.ny, B.nz, B.d, A.cx, A.cy, zbA)) return false;
	if (!plane_z_at(A.nx, A.ny, A.nz, A.d, B.cx, B.cy, zaB)) return false;
	if (!plane_z_at(B.nx, B.ny, B.nz, B.d, B.cx, B.cy, zbB)) return false;
	return (std::fabs(zaA - zbA) <= tol) && (std::fabs(zaB - zbB) <= tol);
}


// 追加: 近傍インデックスから球状度を計算（λ3 / (λ1+λ2+λ3)）。小さいほど「平面/線状」、大きいほど「球状」。
static float compute_sphericity_from_indices(
	const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
	const std::vector<int>& indices) {
	int n = (int)indices.size();
	if (n < 3) return 1.0f; // 点が少なすぎる場合は「球状度が高い」とみなして弾く

	Eigen::MatrixXf M(n, 3);
	for (int i = 0; i < n; ++i) {
		const auto& p = (*cloud)[indices[i]];
		M(i, 0) = p.x; M(i, 1) = p.y; M(i, 2) = p.z;
	}
	Eigen::Vector3f meanv = M.colwise().mean();
	Eigen::MatrixXf C = M.rowwise() - meanv.transpose();
	Eigen::Matrix3f cov = (C.adjoint() * C) / (float)(std::max(1, n - 1));

	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(cov);
	if (solver.info() != Eigen::Success) return 1.0f;

	// 固有値は昇順（λ1<=λ2<=λ3）で返る
	Eigen::Vector3f ev = solver.eigenvalues();
	float l1 = ev(0), l2 = ev(1), l3 = ev(2);
	float sum = l1 + l2 + l3;
	if (sum <= 1e-12f) return 1.0f; // 特異の場合は弾く
	return l1 / sum; // 最小固有値 / 総和（=球状度）
}

// 永井追加: 平面度（planarity）を計算（(λ2 - λ3) / λ1）。大きいほど平面に近い。
static float compute_planarity_from_indices(
	const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
	const std::vector<int>& indices) {
	int n = (int)indices.size();
	if (n < 3) return 0.0f;

	Eigen::MatrixXf M(n, 3);
	for (int i = 0; i < n; ++i) {
		const auto& p = (*cloud)[indices[i]];
		M(i, 0) = p.x; M(i, 1) = p.y; M(i, 2) = p.z;
	}
	Eigen::Vector3f meanv = M.colwise().mean();
	Eigen::MatrixXf C = M.rowwise() - meanv.transpose();
	Eigen::Matrix3f cov = (C.adjoint() * C) / (float)std::max(1, n - 1);

	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(cov);
	if (solver.info() != Eigen::Success) return 0.0f;

	// solver.eigenvalues() は昇順 (a1<=a2<=a3)。降順(Λ1>=Λ2>=Λ3)に並べ替える
	Eigen::Vector3f asc = solver.eigenvalues();
	const float L1 = asc(2); // 最大固有値
	const float L2 = asc(1);
	const float L3 = asc(0); // 最小固有値

	if (L1 <= 1e-12f) return 0.0f;

	// Planarity = (Λ2 - Λ3) / Λ1
	return (L2 - L3) / L1;
}

// ===== 新フロー用の定数とヘルパ =====
static const int   STEP_K_NEIGHBOR = 30;
static const float STEP_NZ_THRESH_SEED = 0.98f;  // seed 用
static const float STEP_NZ_THRESH = 0.95f;
static const float STEP_PLANE_DIST_THRESH = 0.05f;
static const float STEP_RADIUS_SEED = 1.0f; // もともと1.0f 0.3がベスト
static const float STEP_RADIUS_GROW = 5.0f; // もともと2.0f 0.5がベスト
static const int   STEP_MIN_POINTS_STRICT = 5;
static const int   STEP_MIN_POINTS_LOOSE = 5;
static const float STEP_HEIGHT_MAX = 0.20f;
static const float STEP_PROPAGATE_RADIUS1 = 1.5f;
static const float STEP_PROPAGATE_RADIUS2 = 1.5f;
static const float STEP_MAX_ANGLE_DEG = 10.0f;
static const float STEP_VOXEL_SIZE = 0.02f;
// === 追加（統合用しきい値） ===
static const float MERGE_RADIUS_XY = 3.0f; // XY半径 [m]
static const float MERGE_HEIGHT_TOL = 0.10f; // 平面高さ差の許容 [m]
static const float Z_CLIP_HALF = 0.03f; // z窓クリップの半幅 [m]
static const float STEP_TRIM_RATIO = 0.05f;
static const int   STEP_MAX_TRIM_ITER = 8;
static const float STEP_TRIMMED_MEDIAN_TRIM = 0.25f;

struct DistanceIndex {
	float distance;
	int index;
};
// ===== リジェクト構造体 =====
struct RejectedCluster {
	int seed_index;
	std::vector<int> rejected_indices;
};
static std::vector<RejectedCluster> global_rejected_clusters;

static bool distance_index_greater(const DistanceIndex& a, const DistanceIndex& b) {
	return a.distance > b.distance;
}

static void append_unique_indices(std::vector<int>& base, const std::vector<int>& addition) {
	for (size_t i = 0; i < addition.size(); ++i) {
		int idx = addition[i];
		bool exists = false;
		for (size_t j = 0; j < base.size(); ++j) {
			if (base[j] == idx) {
				exists = true;
				break;
			}
		}
		if (!exists) {
			base.push_back(idx);
		}
	}
}

static bool compute_plane_for_indices(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
	const std::vector<int>& indices,
	StepCluster& cluster_out) {
	std::vector<int> tmp = indices;
	float nx = 0.0f;
	float ny = 0.0f;
	float nz = 1.0f;
	float d = 0.0f;
	float mean_z = 0.0f;
	float cx = 0.0f;
	float cy = 0.0f;
	float cz = 0.0f;
	int ok = estimate_plane_from_indices(cloud, tmp, &nx, &ny, &nz, &d, &mean_z, &cx, &cy, &cz);
	if (!ok) {
		return false;
	}
	cluster_out.nx = nx;
	cluster_out.ny = ny;
	cluster_out.nz = nz;
	cluster_out.d = d;
	cluster_out.cx = cx;
	cluster_out.cy = cy;
	cluster_out.cz = cz;
	return true;
}

// ===== 中央値計算ヘルパ関数 =====
static float compute_median(std::vector<float>& values) {
	if (values.empty()) return 0.0f;

	size_t n = values.size();
	size_t mid = n / 2;

	// nth_element で中央値を求める（部分ソート）
	std::nth_element(values.begin(), values.begin() + mid, values.end());

	if (n % 2 == 1) {
		// 奇数個：中央の値
		return values[mid];
	}
	else {
		// 偶数個：中央2つの平均
		float val1 = values[mid];
		// mid-1 の最大値を探す
		std::nth_element(values.begin(), values.begin() + mid - 1, values.begin() + mid);
		float val2 = values[mid - 1];
		return (val1 + val2) * 0.5f;
	}
}

// ===== クラスタ点群の座標中央値を計算 =====
static bool compute_coordinate_medians(
	const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
	const std::vector<int>& indices,
	float& median_x_out,
	float& median_y_out,
	float& median_z_out)
{
	if (indices.empty()) return false;

	// 各軸の座標を収集
	std::vector<float> x_coords, y_coords, z_coords;
	x_coords.reserve(indices.size());
	y_coords.reserve(indices.size());
	z_coords.reserve(indices.size());

	for (size_t i = 0; i < indices.size(); ++i) {
		const auto& p = cloud->points[indices[i]];
		x_coords.push_back(p.x);
		y_coords.push_back(p.y);
		z_coords.push_back(p.z);
	}

	// 各軸の中央値を計算
	median_x_out = compute_median(x_coords);
	median_y_out = compute_median(y_coords);
	median_z_out = compute_median(z_coords);

	return true;
}

static bool trim_z_ends_if_needed(
	const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
	std::vector<int>& indices,
	float z_gate,
	float trim_ratio_each)
{
	int N = (int)indices.size();
	if (N < 10) return false; // 小さすぎるときは何もしない

	// z_min / z_max 計算
	float zmin = 0.0f, zmax = 0.0f;
	for (int i = 0; i < N; ++i) {
		const pcl::PointXYZI& p = cloud->points[indices[i]];
		if (i == 0) { zmin = p.z; zmax = p.z; }
		else {
			if (p.z < zmin) zmin = p.z;
			if (p.z > zmax) zmax = p.z;
		}
	}
	if ((zmax - zmin) <= z_gate) return false; // ゲート以内なら適用しない

	int k = (int)(trim_ratio_each * (float)N);
	if (k < 1) k = 1;                  // 最低1点は削る
	if (2 * k >= N) return false;      // 削り過ぎガード

	// 各要素の削除フラグ
	std::vector<char> del(N, 0);

	// 低い側から k 個削る（毎回最小 z を探す）
	for (int t = 0; t < k; ++t) {
		int min_pos = -1;
		float min_z = 0.0f;
		for (int i = 0; i < N; ++i) {
			if (del[i]) continue;
			float zi = cloud->points[indices[i]].z;
			if (min_pos < 0 || zi < min_z) {
				min_pos = i; min_z = zi;
			}
		}
		if (min_pos >= 0) del[min_pos] = 1;
	}

	// 高い側から k 個削る（毎回最大 z を探す）
	for (int t = 0; t < k; ++t) {
		int max_pos = -1;
		float max_z = 0.0f;
		for (int i = 0; i < N; ++i) {
			if (del[i]) continue;
			float zi = cloud->points[indices[i]].z;
			if (max_pos < 0 || zi > max_z) {
				max_pos = i; max_z = zi;
			}
		}
		if (max_pos >= 0) del[max_pos] = 1;
	}

	// 残すインデックスを再構成
	std::vector<int> kept;
	kept.reserve(N - 2 * k);
	for (int i = 0; i < N; ++i) {
		if (!del[i]) kept.push_back(indices[i]);
	}
	if ((int)kept.size() < 3) return false; // 平面推定に足りない場合は不採用

	indices.swap(kept);
	return true; // z端トリムを適用した
}

static void compute_yz_minmax(
	const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
	const std::vector<int>& indices,
	float& y_min, float& y_max,
	float& z_min, float& z_max)
{
	y_min = std::numeric_limits<float>::max();
	y_max = std::numeric_limits<float>::lowest();
	z_min = std::numeric_limits<float>::max();
	z_max = std::numeric_limits<float>::lowest();
	for (int idx : indices) {
		const auto& p = cloud->points[idx];
		if (p.y < y_min) y_min = p.y; if (p.y > y_max) y_max = p.y;
		if (p.z < z_min) z_min = p.z; if (p.z > z_max) z_max = p.z;
	}
}

// 既存シグネチャを維持
static bool refine_plane_with_trim(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
	std::vector<int>& indices,
	int min_points,
	StepCluster& cluster_out)
{
	int current_points = (int)indices.size();
	if (current_points < min_points) {
		return false;
	}

	// しきい（zレンジ上限と上下端トリム率）
	const float Z_GATE = 0.10f;   // 0.15m を超えないように
	const float Z_TRIM_EACH = 0.03f; // 高い側と低い側を各3%

	for (int iter = 0; iter < STEP_MAX_TRIM_ITER; ++iter) {
		// 1) まず現在の平面推定
		if (!compute_plane_for_indices(cloud, indices, cluster_out)) {
			return false;
		}

		// 2) 十分水平なら完了
		if (cluster_out.nz >= STEP_NZ_THRESH) {
			return true;
		}

		// 3) 点数チェック
		current_points = (int)indices.size();
		if (current_points < min_points) {
			return false;
		}

		// 4) zレンジをチェック → 超えていたら上下端を各3%トリム
		bool applied_ztrim = trim_z_ends_if_needed(cloud, indices, Z_GATE, Z_TRIM_EACH);

		// z端トリム後に点数を再確認
		current_points = (int)indices.size();
		if (current_points < min_points) {
			return false;
		}

		// 5) z端トリムを適用「しなかった」場合のみ、従来の5%距離トリムを実行
		if (!applied_ztrim) {
			int trim_count = (int)std::ceil((double)current_points * STEP_TRIM_RATIO);
			if (trim_count < 1) trim_count = 1;
			if (current_points - trim_count < min_points) {
				return false;
			}

			// 平面からの距離が大きい順に上位 trim_count を除去
			std::vector<DistanceIndex> dist_list;
			dist_list.reserve(indices.size());
			for (size_t i = 0; i < indices.size(); ++i) {
				int idx = indices[i];
				const pcl::PointXYZI& pt = (*cloud)[idx];
				float distance = std::fabs(point_plane_dist(
					pt.x, pt.y, pt.z, cluster_out.nx, cluster_out.ny, cluster_out.nz, cluster_out.d));
				DistanceIndex di; di.distance = distance; di.index = idx;
				dist_list.push_back(di);
			}

			std::sort(dist_list.begin(), dist_list.end(), distance_index_greater);

			std::vector<int> trimmed;
			trimmed.reserve(dist_list.size() - trim_count);
			for (size_t i = trim_count; i < dist_list.size(); ++i) {
				trimmed.push_back(dist_list[i].index);
			}
			// ← 除外される点を記録
			for (size_t i = 0; i < trim_count; ++i) {
				global_rejected_indices.push_back(dist_list[i].index);
			}
			indices.swap(trimmed);


			current_points = (int)indices.size();
			if (current_points < min_points) {
				return false;
			}
		}

		// ループ継続：次イテレーションで再推定 → nz>=閾値 or さらにトリム
		// ※ zレンジがまだ0.15を超えていれば、次周回でもう一度 z端3%がかかります
	}

	// 収束しなかったら最終確認
	return cluster_out.nz >= STEP_NZ_THRESH;
}

static bool refine_plane_with_trim_debug(
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
	std::vector<int>& indices,
	int min_points,
	StepCluster& cluster_out,
	int seed_index,
	std::ofstream& debug_log)
{
	int current_points = (int)indices.size();
	if (current_points < min_points) {
		return false;
	}

	const float Z_GATE = 0.10f;
	const float Z_TRIM_EACH = 0.03f;

	for (int iter = 0; iter < STEP_MAX_TRIM_ITER; ++iter) {
		int points_before_iter = (int)indices.size();

		// 1) 平面推定
		if (!compute_plane_for_indices(cloud, indices, cluster_out)) {
			return false;
		}

		float nz_before = cluster_out.nz;

		// 2) 十分水平なら完了

		if (cluster_out.nz >= STEP_NZ_THRESH) {
			if (debug_log && iter > 0) {
				float y_min, y_max, z_min, z_max;
				compute_yz_minmax(cloud, indices, y_min, y_max, z_min, z_max);
				debug_log << seed_index << ",refine_converged," << iter << ","
					<< points_before_iter << "," << points_before_iter << ",0,"
					<< nz_before << "," << nz_before << ","
					<< cluster_out.cx << "," << cluster_out.cy << "," << cluster_out.cz << ","
					<< y_min << "," << y_max << "," << (y_max - y_min) << ","
					<< z_min << "," << z_max << "," << (z_max - z_min) << ","
					<< "CONVERGE,nz_ok\n";
			}
			return true;
		}

		// 3) 点数チェック
		current_points = (int)indices.size();
		if (current_points < min_points) {
			return false;
		}

		// 4) z端トリム
		bool applied_ztrim = trim_z_ends_if_needed(cloud, indices, Z_GATE, Z_TRIM_EACH);

		int points_after_ztrim = (int)indices.size();

		if (applied_ztrim && debug_log) {
			// 再推定して新しいnzを取得
			compute_plane_for_indices(cloud, indices, cluster_out);
			float nz_after = cluster_out.nz;

			float y_min, y_max, z_min, z_max;
			compute_yz_minmax(cloud, indices, y_min, y_max, z_min, z_max);

			debug_log << seed_index << ",z_end_trim," << iter << ","
				<< points_before_iter << "," << points_after_ztrim << ","
				<< (points_before_iter - points_after_ztrim) << ","
				<< nz_before << "," << nz_after << ","
				<< cluster_out.cx << "," << cluster_out.cy << "," << cluster_out.cz << ","
				<< y_min << "," << y_max << "," << (y_max - y_min) << ","
				<< z_min << "," << z_max << "," << (z_max - z_min) << ","
				<< "TRIM,z_ends_3pct\n";
		}

		current_points = (int)indices.size();
		if (current_points < min_points) {
			return false;
		}

		// 5) z端トリム未適用時は距離トリム
		if (!applied_ztrim) {
			int trim_count = (int)std::ceil((double)current_points * STEP_TRIM_RATIO);
			if (trim_count < 1) trim_count = 1;
			if (current_points - trim_count < min_points) {
				return false;
			}

			int points_before_dist = current_points;

			std::vector<DistanceIndex> dist_list;
			dist_list.reserve(indices.size());
			for (size_t i = 0; i < indices.size(); ++i) {
				int idx = indices[i];
				const pcl::PointXYZI& pt = (*cloud)[idx];
				float distance = std::fabs(point_plane_dist(
					pt.x, pt.y, pt.z, cluster_out.nx, cluster_out.ny, cluster_out.nz, cluster_out.d));
				DistanceIndex di; di.distance = distance; di.index = idx;
				dist_list.push_back(di);
			}

			// ← 除外点を記録
			RejectedCluster rejected;
			rejected.seed_index = seed_index;
			for (size_t i = 0; i < trim_count; ++i) {
				rejected.rejected_indices.push_back(dist_list[i].index);
			}
			global_rejected_clusters.push_back(rejected);

			std::sort(dist_list.begin(), dist_list.end(), distance_index_greater);

			std::vector<int> trimmed;
			trimmed.reserve(dist_list.size() - trim_count);
			for (size_t i = trim_count; i < dist_list.size(); ++i) {
				trimmed.push_back(dist_list[i].index);
			}
			indices.swap(trimmed);

			int points_after_dist = (int)indices.size();

			if (debug_log) {
				compute_plane_for_indices(cloud, indices, cluster_out);
				float nz_after = cluster_out.nz;

				float y_min, y_max, z_min, z_max;
				compute_yz_minmax(cloud, indices, y_min, y_max, z_min, z_max);

				debug_log << seed_index << ",distance_trim," << iter << ","
					<< points_before_dist << "," << points_after_dist << ","
					<< (points_before_dist - points_after_dist) << ","
					<< nz_before << "," << nz_after << ","
					<< cluster_out.cx << "," << cluster_out.cy << "," << cluster_out.cz << ","
					<< y_min << "," << y_max << "," << (y_max - y_min) << ","
					<< z_min << "," << z_max << "," << (z_max - z_min) << ","
					<< "TRIM,plane_dist_5pct\n";
			}

			current_points = (int)indices.size();
			if (current_points < min_points) {
				return false;
			}
		}
	}

	// 収束しなかった
	return cluster_out.nz >= STEP_NZ_THRESH;
}


static bool gather_knn_indices(const pcl::KdTreeFLANN<pcl::PointXYZI>& kdtree,
	const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
	int query_index,
	int k,
	std::vector<int>& indices_out) {
	pcl::PointXYZI query_point = (*cloud)[query_index];
	std::vector<int> k_indices;
	std::vector<float> k_dist;
	k_indices.resize(k);
	k_dist.resize(k);
	int found = kdtree.nearestKSearch(query_point, k, k_indices, k_dist);
	if (found < k) {
		return false;
	}
	indices_out.clear();
	for (int i = 0; i < k; ++i) {
		indices_out.push_back(k_indices[i]);
	}
	return true;
}

// 収集：中心centerまわりのXY半径・（任意の）Y/Zウィンドウ・平面距離で候補点を抽出
static void collect_radius_indices_xy(
	const pcl::KdTreeFLANN<pcl::PointXYZI>& kdtree,
	const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
	const pcl::PointXYZI& center,
	float radius_xy,
	const StepCluster& plane,
	std::vector<int>& indices_out,
	const std::vector<char>& cluster_point_mask,  // ← 追加
	float y_half_width = -1.0f,
	float z_half_width = -1.0f
) {
	float radius_sq = radius_xy * radius_xy;
	float radius_search =
		std::sqrt(radius_xy * radius_xy + STEP_PLANE_DIST_THRESH * STEP_PLANE_DIST_THRESH) + 1e-3f;

	std::vector<int> tmp_indices;
	std::vector<float> tmp_dist;
	tmp_indices.reserve(64);
	tmp_dist.reserve(64);

	kdtree.radiusSearch(center, radius_search, tmp_indices, tmp_dist);

	indices_out.clear();
	for (size_t i = 0; i < tmp_indices.size(); ++i) {
		int idx = tmp_indices[i];

		// 既に他のクラスタに属する点は除外
		if (cluster_point_mask[idx]) continue;

		const pcl::PointXYZI& pt = (*cloud)[idx];

		// 1) XY半径
		float dx = pt.x - center.x;
		float dy = pt.y - center.y;
		float xy_sq = dx * dx + dy * dy;
		if (xy_sq > radius_sq) continue;

		// 2) Yウィンドウ（任意）
		if (y_half_width > 0.0f) {
			if (std::fabs(pt.y - center.y) > y_half_width) continue;
		}

		// 3) Zウィンドウ（任意）
		if (z_half_width > 0.0f) {
			if (std::fabs(pt.z - center.z) > z_half_width) continue;
		}

		// 4) 平面距離
		float distance = point_plane_dist(pt.x, pt.y, pt.z,
			plane.nx, plane.ny, plane.nz, plane.d);
		if (std::fabs(distance) <= STEP_PLANE_DIST_THRESH) {
			indices_out.push_back(idx);
		}
	}
}


// クラスタ重心czを中心に±0.05mの範囲に入っていない点を一括除去
static void clip_indices_by_z_window(
	const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
	std::vector<int>& indices,
	float cz,
	float half_window,
	int seed_index)
{
	std::vector<int> kept;
	kept.reserve(indices.size());
	float zmin = cz - half_window, zmax = cz + half_window;

	// ← 除外点を記録する構造体を準備
	RejectedCluster rejected;
	rejected.seed_index = seed_index;

	for (size_t i = 0; i < indices.size(); ++i) {
		const auto& p = cloud->points[indices[i]];
		if (p.z >= zmin && p.z <= zmax) {
			kept.push_back(indices[i]);
		}
		else {
			// ← 除外点を記録
			rejected.rejected_indices.push_back(indices[i]);
		}
	}

	// ← 除外点があればグローバル配列に追加
	if (!rejected.rejected_indices.empty()) {
		global_rejected_clusters.push_back(rejected);
	}

	if (kept.size() >= 3) indices.swap(kept);
}
// y座標幅でクラスタをトリム（重心から遠い順に削除）
static void trim_cluster_by_y_width(
	const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
	std::vector<int>& indices,
	float cy,  // クラスタのy重心
	float max_y_width,  // 許容最大幅 [m]
	float trim_ratio = 0.05f)  // 1回あたりのトリム率
{
	if (indices.size() < 10) return;  // 点数が少なすぎる場合は何もしない

	const int MAX_ITERATIONS = 10;  // 最大反復回数

	for (int iter = 0; iter < MAX_ITERATIONS; ++iter) {
		// 現在のy座標幅を計算
		float y_min = std::numeric_limits<float>::max();
		float y_max = std::numeric_limits<float>::lowest();

		for (size_t i = 0; i < indices.size(); ++i) {
			float y = cloud->points[indices[i]].y;
			if (y < y_min) y_min = y;
			if (y > y_max) y_max = y;
		}

		float current_width = y_max - y_min;

		// 幅が許容範囲内なら終了
		if (current_width <= max_y_width) {
			break;
		}

		// トリムする点数を計算
		int n = (int)indices.size();
		int trim_count = std::max(1, (int)(n * trim_ratio));

		// 点数が最小限を下回る場合は終了
		if (n - trim_count < 3) {
			break;
		}

		// 重心からのy距離を計算してソート用配列を作成
		std::vector<std::pair<float, int>> dist_idx;
		dist_idx.reserve(indices.size());

		for (size_t i = 0; i < indices.size(); ++i) {
			float y = cloud->points[indices[i]].y;
			float dist = std::fabs(y - cy);
			dist_idx.push_back(std::make_pair(dist, indices[i]));
		}

		// y距離の降順にソート（遠い順）
		std::sort(dist_idx.begin(), dist_idx.end(),
			[](const std::pair<float, int>& a, const std::pair<float, int>& b) {
			return a.first > b.first;
		});

		// 遠い方から trim_count 個を除去
		std::vector<int> kept;
		kept.reserve(indices.size() - trim_count);

		for (size_t i = trim_count; i < dist_idx.size(); ++i) {
			kept.push_back(dist_idx[i].second);
		}

		indices.swap(kept);

		// 重心を再計算（次のイテレーション用）
		float sum_y = 0.0f;
		for (size_t i = 0; i < indices.size(); ++i) {
			sum_y += cloud->points[indices[i]].y;
		}
		cy = sum_y / (float)indices.size();
	}
}

//static void compute_yz_minmax(
//	const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
//	const std::vector<int>& indices,
//	float& y_min, float& y_max,
//	float& z_min, float& z_max)
//{
//	y_min = std::numeric_limits<float>::max();
//	y_max = std::numeric_limits<float>::lowest();
//	z_min = std::numeric_limits<float>::max();
//	z_max = std::numeric_limits<float>::lowest();
//	for (int idx : indices) {
//		const auto& p = cloud->points[idx];
//		if (p.y < y_min) y_min = p.y; if (p.y > y_max) y_max = p.y;
//		if (p.z < z_min) z_min = p.z; if (p.z > z_max) z_max = p.z;
//	}
//}

// 平面距離（|n·p + d|）で一括トリム
static void clip_indices_by_plane_distance(
	const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
	std::vector<int>& indices,
	const StepCluster& plane,
	float dist_thresh /* 例: 0.05f */
) {
	std::vector<int> kept;
	kept.reserve(indices.size());
	for (size_t i = 0; i < indices.size(); ++i) {
		int idx = indices[i];
		const auto& p = cloud->points[idx];
		float dist = std::fabs(
			plane.nx * p.x + plane.ny * p.y + plane.nz * p.z + plane.d
		);
		if (dist <= dist_thresh) {
			kept.push_back(idx);
		}
	}
	if (kept.size() >= 3) {
		indices.swap(kept);
	}
}

static  bool symmetric_height_ok_by_normal(
	const StepCluster& A, const StepCluster& B, float tol)
{
	float sBA = A.nx * B.cx + A.ny * B.cy + A.nz * B.cz + A.d; // B重心のA平面からの法線距離
	float sAB = B.nx * A.cx + B.ny * A.cy + B.nz * A.cz + B.d; // A重心のB平面からの法線距離
	return (std::fabs(sBA) <= tol) && (std::fabs(sAB) <= tol);
}



static bool compute_trimmed_median(const std::vector<float>& values, float& median_out) {
	if (values.empty()) {
		return false;
	}
	std::vector<float> sorted = values;
	std::sort(sorted.begin(), sorted.end());
	int n = (int)sorted.size();
	int trim = (int)(STEP_TRIMMED_MEDIAN_TRIM * n);
	if (trim * 2 >= n) {
		trim = 0;
	}
	int start = trim;
	int end = n - trim;
	int length = end - start;
	if (length <= 0) {
		return false;
	}
	int mid = start + length / 2;
	if (length % 2 == 1) {
		median_out = sorted[mid];
	}
	else {
		median_out = 0.5f * (sorted[mid - 1] + sorted[mid]);
	}
	return true;
}

static float compute_xy_distance(const StepCluster& a, const StepCluster& b) {
	float dx = a.cx - b.cx;
	float dy = a.cy - b.cy;
	return std::sqrt(dx * dx + dy * dy);
}

static float compute_normal_angle_deg(const StepCluster& a, const StepCluster& b) {
	float dot = a.nx * b.nx + a.ny * b.ny + a.nz * b.nz;
	if (dot > 1.0f) {
		dot = 1.0f;
	}
	if (dot < -1.0f) {
		dot = -1.0f;
	}
	float angle = std::acos(dot);
	return angle * 180.0f / static_cast<float>(M_PI);
}

static void find_up_down_gaps(const std::vector<CandidateInfo>& infos,
	int index,
	float xy_radius,
	float& up_gap,
	float& down_gap) {
	up_gap = std::numeric_limits<float>::quiet_NaN();
	down_gap = std::numeric_limits<float>::quiet_NaN();
	float best_up = std::numeric_limits<float>::max();
	float best_down = std::numeric_limits<float>::max();
	const StepCluster& base = infos[index].cluster;
	for (size_t i = 0; i < infos.size(); ++i) {
		if (i == (size_t)index) {
			continue;
		}
		if (!infos[i].is_step) {
			continue;
		}
		const StepCluster& other = infos[i].cluster;
		float xy = compute_xy_distance(base, other);
		if (xy > xy_radius) {
			continue;
		}
		float dz = other.cz - base.cz;
		if (dz > 0.0f) {
			if (dz < best_up) {
				best_up = dz;
			}
		}
		else if (dz < 0.0f) {
			float dz_abs = std::fabs(dz);
			if (dz_abs < best_down) {
				best_down = dz_abs;
			}
		}
	}
	if (best_up < std::numeric_limits<float>::max()) {
		up_gap = best_up;
	}
	if (best_down < std::numeric_limits<float>::max()) {
		down_gap = best_down;
	}
}

struct StepClusterLogHelper {
	int submap_start;
	int submap_end;
	int cluster_id;
	const CandidateInfo* info;
	const std::string* reason;
	float up_gap;
	float down_gap;
	StepClusterLogHelper(int s_start, int s_end, int cid,
		const CandidateInfo* info_ptr,
		const std::string* reason_ptr,
		float up_value,
		float down_value)
		: submap_start(s_start), submap_end(s_end), cluster_id(cid), info(info_ptr), reason(reason_ptr),
		up_gap(up_value), down_gap(down_value) {
	}
	void operator()(JsonWriter& jw) const {
		std::string submap_label = std::to_string(submap_start) + "-" + std::to_string(submap_end);
		jw.key("submap_id").string(submap_label.c_str());
		jw.key("cluster_id").integer((long long)cluster_id);
		jw.key("size").integer((long long)info->cluster.indices.size());

		jw.key("centroid").beginObject();
		jw.key("cx").number(info->cluster.cx, 6);
		jw.key("cy").number(info->cluster.cy, 6);
		jw.key("cz").number(info->cluster.cz, 6);
		jw.endObject();

		// ===== ここに追加 =====
		jw.key("median").beginObject();
		jw.key("median_x").number(info->cluster.geometry.median_x, 6);
		jw.key("median_y").number(info->cluster.geometry.median_y, 6);
		jw.key("median_z").number(info->cluster.geometry.median_z, 6);
		jw.endObject();
		// ==================

		jw.key("normal").beginObject();
		jw.key("nx").number(info->cluster.nx, 6);
		jw.key("ny").number(info->cluster.ny, 6);
		jw.key("nz").number(info->cluster.nz, 6);
		jw.endObject();
		jw.key("d").number(info->cluster.d, 6);
		jw.key("ground_gap_median").number(info->ground_gap_med, 6);
		jw.key("ground_checked").boolean(info->ground_checked);
		jw.key("is_step").boolean(info->is_step);
		jw.key("reason").string(reason->c_str());
		jw.key("up_gap").number(up_gap, 6);
		jw.key("down_gap").number(down_gap, 6);
		jw.key("geometry").beginObject();
		const ClusterGeometry& g = info->cluster.geometry;
		jw.key("rect_area").number(g.rect_area, 6);
		jw.key("actual_area").number(g.actual_area, 6);
		jw.key("point_density").number(g.point_density, 6);

		jw.key("eigenvalues_raw").beginObject();
		jw.key("lambda1").number(g.lambda1_raw, 6);
		jw.key("lambda2").number(g.lambda2_raw, 6);
		jw.key("lambda3").number(g.lambda3_raw, 6);
		jw.endObject();

		jw.key("eigenvalues_norm").beginObject();
		jw.key("lambda1").number(g.lambda1_norm, 6);
		jw.key("lambda2").number(g.lambda2_norm, 6);
		jw.key("lambda3").number(g.lambda3_norm, 6);
		jw.endObject();

		jw.key("features_raw").beginObject();
		jw.key("linearity").number(g.linearity_raw, 6);
		jw.key("planarity").number(g.planarity_raw, 6);
		jw.key("sphericity").number(g.sphericity_raw, 6);
		jw.key("anisotropy").number(g.anisotropy_raw, 6);
		jw.key("omnivariance").number(g.omnivariance_raw, 6);
		jw.key("surface_variation").number(g.surface_variation_raw, 6);
		jw.endObject();

		jw.key("features_norm").beginObject();
		jw.key("linearity").number(g.linearity_norm, 6);
		jw.key("planarity").number(g.planarity_norm, 6);
		jw.key("sphericity").number(g.sphericity_norm, 6);
		jw.key("anisotropy").number(g.anisotropy_norm, 6);
		jw.key("omnivariance").number(g.omnivariance_norm, 6);
		jw.key("surface_variation").number(g.surface_variation_norm, 6);
		jw.endObject();

		jw.endObject(); // geometry
	}
};

// ===== 永井追加：平面マージ関数（Save.cpp 内に実装）=====
static std::vector<StepCluster> merge_step_clusters(
	const std::vector<StepCluster>& clusters,
	const patchwork::Params& params,
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
	int N = (int)clusters.size();
	if (N <= 1) {
		return clusters;
	}

	// しきい
	double angle_thresh_rad = params.step_merge_angle_deg * M_PI / 180.0;

	// Union-Find 準備
	UnionFind uf(N);

	// -------- 1) ペアワイズ判定（法線角 / オフセット / XY距離）で unite --------
	for (int i = 0; i < N; ++i) {
		for (int j = i + 1; j < N; ++j) {
			const StepCluster& ci = clusters[i];
			const StepCluster& cj = clusters[j];

			// (a) 法線角度差
			double dot = ci.nx * cj.nx + ci.ny * cj.ny + ci.nz * cj.nz;
			if (dot < 0.0) {
				dot = -dot;
			}
			if (dot > 1.0) {
				dot = 1.0;  // 安全クランプ
			}
			double angle = std::acos(dot);
			// if (angle > angle_thresh_rad) {
			//     continue;
			// }

			// (b) オフセット差（対称チェック）
			double di_to_j = std::fabs(cj.nx * ci.cx + cj.ny * ci.cy + cj.nz * ci.cz + cj.d);
			double dj_to_i = std::fabs(ci.nx * cj.cx + ci.ny * cj.cy + ci.nz * cj.cz + ci.d);
			if (di_to_j > params.step_merge_offset_thresh ||
				dj_to_i > params.step_merge_offset_thresh) {
				continue;
			}

			// (c) XY 重心距離
			double dx = ci.cx - cj.cx;
			double dy = ci.cy - cj.cy;
			double xy_dist = std::sqrt(dx * dx + dy * dy);
			if (xy_dist > params.step_merge_xy_dist) {
				continue;
			}

			// (d) マージ
			uf.unite(i, j);
		}
	}

	// -------- 2) グループ化（map なしで代表ID→グループID を自前で割当） --------
	// roots[i] = 代表（親）
	std::vector<int> roots(N, -1);
	for (int i = 0; i < N; ++i) {
		roots[i] = uf.find(i);
	}

	// unique_roots: 出現順に代表IDを保持
	std::vector<int> unique_roots;
	for (int i = 0; i < N; ++i) {
		int r = roots[i];
		int found = 0;
		for (size_t u = 0; u < unique_roots.size(); ++u) {
			if (unique_roots[u] == r) {
				found = 1;
				break;
			}
		}
		if (!found) {
			unique_roots.push_back(r);
		}
	}

	// groups[g] に「元クラスタのインデックス」を入れる
	std::vector< std::vector<int> > groups;
	groups.resize(unique_roots.size());
	for (int i = 0; i < N; ++i) {
		int r = roots[i];
		int g = -1;
		for (size_t u = 0; u < unique_roots.size(); ++u) {
			if (unique_roots[u] == r) {
				g = (int)u;
				break;
			}
		}
		if (g >= 0) {
			groups[g].push_back(i);
		}
	}

	// -------- 3) 統合クラスタ再構築（indices結合 → 平面再推定） --------
	std::vector<StepCluster> merged;
	for (size_t g = 0; g < groups.size(); ++g) {
		// indices を結合
		StepCluster mc;
		for (size_t t = 0; t < groups[g].size(); ++t) {
			int idx_cluster = groups[g][t];
			const StepCluster& src = clusters[idx_cluster];
			for (size_t k = 0; k < src.indices.size(); ++k) {
				mc.indices.push_back(src.indices[k]);
			}
		}

		// 点が3未満なら推定できないのでスキップ
		if (mc.indices.size() < 3) {
			continue;
		}

		// 再推定
		std::vector<int> temp_idx;
		temp_idx.reserve(mc.indices.size());
		for (size_t k = 0; k < mc.indices.size(); ++k) {
			temp_idx.push_back(mc.indices[k]);
		}

		float out_nx = 0.0f, out_ny = 0.0f, out_nz = 0.0f, out_d = 0.0f;
		float mean_z = 0.0f, cx2 = 0.0f, cy2 = 0.0f, cz2 = 0.0f;

		int ok = estimate_plane_from_indices(
			cloud,
			temp_idx,
			&out_nx, &out_ny, &out_nz, &out_d,
			&mean_z, &cx2, &cy2, &cz2);

		if (ok) {
			mc.nx = out_nx;
			mc.ny = out_ny;
			mc.nz = out_nz;
			mc.d = out_d;
			mc.cx = cx2;
			mc.cy = cy2;
			mc.cz = cz2;
			merged.push_back(mc);
		}
	}

	return merged;
}

// ===== 永井追加：セクタ連結成分抽出（Save.cpp 内に実装）=====
static std::set<std::tuple<int, int, int>> extract_largest_connected_sectors(
	const std::map<std::tuple<int, int, int>, int>& sector_hist,
	const patchwork::Params& params)
{
	if (sector_hist.empty()) return std::set<std::tuple<int, int, int>>();

	std::map<std::tuple<int, int, int>, std::vector<std::tuple<int, int, int>>> adj;

	for (auto& kv : sector_hist) {
		// C++14互換：tuple を個別に取り出す
		int z = std::get<0>(kv.first);
		int r = std::get<1>(kv.first);
		int s = std::get<2>(kv.first);

		int num_sectors = params.num_sectors_each_zone[z];
		int num_rings = params.num_rings_each_zone[z];

		int sl = (s - 1 + num_sectors) % num_sectors;
		int sr = (s + 1) % num_sectors;

		// ラムダ関数で隣接セクタを追加
		auto try_add = [&](int z2, int r2, int s2) {
			std::tuple<int, int, int> key = std::make_tuple(z2, r2, s2);
			if (sector_hist.count(key)) {
				adj[kv.first].push_back(key);
			}
		};

		// 同リングの左右
		try_add(z, r, sl);
		try_add(z, r, sr);

		// 内側リング
		if (r > 0) {
			try_add(z, r - 1, sl);
			try_add(z, r - 1, s);
			try_add(z, r - 1, sr);
		}

		// 外側リング
		if (r < num_rings - 1) {
			try_add(z, r + 1, sl);
			try_add(z, r + 1, s);
			try_add(z, r + 1, sr);
		}
	}

	// BFSで連結成分を列挙
	std::set<std::tuple<int, int, int>> visited;
	std::vector<std::set<std::tuple<int, int, int>>> components;

	for (auto& kv : sector_hist) {
		if (visited.count(kv.first)) continue;

		std::set<std::tuple<int, int, int>> comp;
		std::queue<std::tuple<int, int, int>> q;
		q.push(kv.first);
		visited.insert(kv.first);

		while (!q.empty()) {
			std::tuple<int, int, int> cur = q.front();
			q.pop();
			comp.insert(cur);

			if (adj.count(cur)) {
				for (size_t i = 0; i < adj[cur].size(); ++i) {
					std::tuple<int, int, int> nb = adj[cur][i];
					if (!visited.count(nb)) {
						visited.insert(nb);
						q.push(nb);
					}
				}
			}
		}
		components.push_back(comp);
	}

	// 最大サイズの連結成分を返す
	if (components.empty()) {
		return std::set<std::tuple<int, int, int>>();
	}

	size_t max_size = 0;
	size_t max_idx = 0;
	for (size_t i = 0; i < components.size(); ++i) {
		if (components[i].size() > max_size) {
			max_size = components[i].size();
			max_idx = i;
		}
	}

	return components[max_idx];
}

// 候補クラスタの点インデックスを CSV 出力
static void dump_step_cluster_members(
	const std::vector<CandidateInfo>& infos,
	int submap_start, int submap_end,
	const std::string& out_dir,
	const pcl::PointCloud<pcl::PointXYZI>::Ptr& nonground_cloud)
{
	std::string path = out_dir
		+ "/step_cluster_members_"
		+ std::to_string(submap_start)
		+ "_"
		+ std::to_string(submap_end)
		+ ".csv";

	std::ofstream ofs(path.c_str());
	if (!ofs) {
		return;
	}

	ofs << "cluster_id,point_index,x,y,z\n";

	for (size_t ci = 0; ci < infos.size(); ++ci) {
		const StepCluster& c = infos[ci].cluster;
		for (size_t k = 0; k < c.indices.size(); ++k) {
			int idx = c.indices[k];
			if (idx >= 0 && idx < static_cast<int>(nonground_cloud->points.size())) {
				const pcl::PointXYZI& p = nonground_cloud->points[idx];
				ofs << ci << "," << idx << "," << p.x << "," << p.y << "," << p.z << "\n";
			}
		}
	}

	ofs.close();
}


// 追加: 中央値ヘルパ
static float median_abs(std::vector<float>& v) {
	if (v.empty()) return 0.0f;
	size_t n = v.size();
	size_t mid = n / 2;
	std::nth_element(v.begin(), v.begin() + mid, v.end());
	float m = v[mid];
	if ((n & 1) == 0) {
		// even -> もう片方との平均（任意）
		auto max_it = std::max_element(v.begin(), v.begin() + mid);
		m = 0.5f * (m + *max_it);
	}
	return m;
}

// 追加: クラスタ点群 -> 路面パッチ平面へのロバスト距離中央値（gap_median）
static bool cluster_to_patch_gap_median(
	const StepCluster& c,
	const std::vector<int>& cluster_indices,  // indices in nonground_cloud
	const pcl::PointCloud<pcl::PointXYZI>::Ptr& nonground_cloud,
	const patchwork::PatchPCAResult& pr,
	float& gap_median_out,
	float& xy_out)
{
	// 近傍XY距離（重心間）を併記して返す
	float dx = c.cx - pr.pc_mean(0);
	float dy = c.cy - pr.pc_mean(1);
	xy_out = std::sqrt(dx * dx + dy * dy);

	// 共有法線はパッチ法線を採用（向きだけ整合）
	Eigen::Vector3f n = pr.normal;
	Eigen::Vector3f nc(c.nx, c.ny, c.nz);
	if (n.dot(nc) < 0.0f) n = -n;
	float d = (n(0) == pr.normal(0) && n(1) == pr.normal(1) && n(2) == pr.normal(2)) ? pr.d : -pr.d;

	// 各クラスタ点の点→平面距離 |n·p + d| の中央値
	std::vector<float> absdist;
	absdist.reserve(cluster_indices.size());
	for (int idx : cluster_indices) {
		const auto& p = (*nonground_cloud)[idx];
		float dist = std::fabs(n(0) * p.x + n(1) * p.y + n(2) * p.z + d);
		absdist.push_back(dist);
	}
	if (absdist.size() < 1) return false;

	gap_median_out = median_abs(absdist);
	return true;
}

// === 追加: 路面セクタモルフォロジ（ground→nonground への再分類を事前に実施） ===
struct CellKey { int z, r, s; };
struct CellKeyHash {
	size_t operator()(CellKey const& k) const {
		return (size_t)k.z * 73856093u ^ (size_t)k.r * 19349663u ^ (size_t)k.s * 83492791u;
	}
};
struct CellKeyEq {
	bool operator()(CellKey const& a, CellKey const& b) const {
		return a.z == b.z && a.r == b.r && a.s == b.s;
	}
};

static void apply_road_sector_morphology(
	Eigen::MatrixX3f& ground,
	Eigen::MatrixX3f& nonground,
	patchwork::Ground_Segmentation& GS,
	const std::string& submap_id)
{
	using Eigen::Vector3f;

	// 1) ground 点を CZM に投影して road セル集合を作成
	std::unordered_set<CellKey, CellKeyHash, CellKeyEq> road_cells;
	road_cells.reserve((size_t)ground.rows());

	for (int i = 0; i < ground.rows(); ++i) {
		int z, r, s;
		if (GS.pointToCZM(ground(i, 0), ground(i, 1), z, r, s)) {
			road_cells.insert(CellKey{ z, r, s });
		}
	}
	if (road_cells.empty()) return;

	// 2) 8近傍を数えて除去対象セルを決定
	std::unordered_set<CellKey, CellKeyHash, CellKeyEq> remove_cells;

	auto has = [&](int z, int r, int s)->bool {
		const int num_sectors = GS.params_.num_sectors_each_zone[z];
		return road_cells.count(CellKey{ z, r, (s + num_sectors) % num_sectors }) > 0;
	};

	for (const auto& key : road_cells) {
		const int z = key.z, r = key.r, s = key.s;
		const int num_rings = GS.params_.num_rings_each_zone[z];
		const int num_sectors = GS.params_.num_sectors_each_zone[z];

		// 最内周は保護
		if (r == 0) continue;

		int total_nb = 0;
		int inner_nb = 0;

		// 同リング s±1
		total_nb += has(z, r, s - 1) ? 1 : 0;
		total_nb += has(z, r, s + 1) ? 1 : 0;

		// 内側リング r-1 の s±1, s
		if (r - 1 >= 0) {
			int c0 = has(z, r - 1, s - 1) ? 1 : 0;
			int c1 = has(z, r - 1, s) ? 1 : 0;
			int c2 = has(z, r - 1, s + 1) ? 1 : 0;
			total_nb += c0 + c1 + c2;
			inner_nb += c0 + c1 + c2;
		}

		// 外側リング r+1 の s±1, s
		if (r + 1 < num_rings) {
			total_nb += has(z, r + 1, s - 1) ? 1 : 0;
			total_nb += has(z, r + 1, s) ? 1 : 0;
			total_nb += has(z, r + 1, s + 1) ? 1 : 0;
		}

		// 条件: 近傍3以下 かつ 内側リング0 → 外側へ単発に飛び出たと判断
		if (total_nb <= 3 && inner_nb == 0) {
			remove_cells.insert(key);
		}
	}

	if (!remove_cells.empty()) {
		JsonlLogger::instance().logEvent("road_morph_removed_cells", [&](JsonWriter& jw) {
			jw.key("schema_version").string("1.0.0");
			jw.key("submap_id").string(submap_id);  // ← Save から渡された ID をそのまま書く
			jw.key("count").integer((int)remove_cells.size());
			jw.key("cells").beginArray();
			for (const auto& key : remove_cells) {
				jw.beginObject();
				jw.key("z").integer(key.z);
				jw.key("r").integer(key.r);
				jw.key("s").integer(key.s);
				jw.endObject();
			}
			jw.endArray();
		});
	}


	if (remove_cells.empty()) return;

	// 3) ground を分割し、除去セルの点は nonground へ移動
	std::vector<Vector3f> g_keep;       g_keep.reserve((size_t)ground.rows());
	std::vector<Vector3f> ng_append;    ng_append.reserve((size_t)ground.rows() / 8);

	for (int i = 0; i < ground.rows(); ++i) {
		Vector3f p = ground.row(i);
		int z, r, s;
		if (!GS.pointToCZM(p(0), p(1), z, r, s)) {
			g_keep.push_back(p);
			continue;
		}
		CellKey ck{ z, r, s };
		if (remove_cells.count(ck)) {
			ng_append.push_back(p); // 非路面化
		}
		else {
			g_keep.push_back(p);
		}
	}

	// ground を更新
	Eigen::MatrixX3f ground_new((int)g_keep.size(), 3);
	for (int i = 0; i < (int)g_keep.size(); ++i) ground_new.row(i) = g_keep[i];
	ground = std::move(ground_new);

	// nonground に追加
	if (!ng_append.empty()) {
		Eigen::MatrixX3f nonground_new((int)nonground.rows() + (int)ng_append.size(), 3);
		// 既存
		for (int i = 0; i < nonground.rows(); ++i) nonground_new.row(i) = nonground.row(i);
		// 追加
		int base = nonground.rows();
		for (int i = 0; i < (int)ng_append.size(); ++i) nonground_new.row(base + i) = ng_append[i];
		nonground = std::move(nonground_new);
	}
}

// === 追加: モルフォロジ適用“後”の ground からアンカー（PCA パッチ群）を再生成する ===
struct AnchorPatch {
	// CZM 位置
	int zone_idx;
	int ring_idx;
	int sector_idx;

	// 幾何
	Eigen::Vector3f centroid; // ground 点の平均
	Eigen::Vector3f normal;   // λmin の固有ベクトル（上向き整合）
	float d;                  // 平面の定数項: n·x + d = 0

	// 品質・参考量（必要最小限）
	int point_count;
	float lambda1;
	float lambda2;
	float lambda3;
	float planarity;   // (λ2 - λ3) / λ1
	float linearity;   // (λ1 - λ2) / λ1
	float sphericity;  // λ3 / λ1
	float uprightness; // |nz|
};

// CZM の (z,r,s) ごとに ground 点を束ね、PCA を回してアンカー集合を作る

static std::vector<AnchorPatch> build_anchor_patches_from_ground(
	const Eigen::MatrixX3f& ground,
	patchwork::Ground_Segmentation& GS,
	int min_points_per_cell = 10)
{
	using Eigen::Vector3f;
	struct CellAgg {
		std::vector<Vector3f> pts;
		Vector3f mean = Eigen::Vector3f::Zero();
	};

	// 1) ground を CZM に投影してセル集約
	std::unordered_map<long long, CellAgg> bins;
	bins.reserve((size_t)ground.rows());

	auto key64 = [](int z, int r, int s)->long long {
		// 20bitずつ詰める（z,r,s が仕様範囲内である前提）
		return ((long long)(z & 0xFFFFF) << 40)
			| ((long long)(r & 0xFFFFF) << 20)
			| ((long long)(s & 0xFFFFF));
	};

	for (int i = 0; i < ground.rows(); ++i) {
		int z, r, s;
		if (!GS.pointToCZM(ground(i, 0), ground(i, 1), z, r, s)) continue;
		auto k = key64(z, r, s);
		auto& agg = bins[k];
		Vector3f p = ground.row(i);
		agg.pts.push_back(p);
		agg.mean += p;
	}

	std::vector<AnchorPatch> patches;
	patches.reserve(bins.size());

	// 2) 各セルで PCA を実行してアンカー生成
	for (auto& kv : bins) {
		auto& agg = kv.second;
		const int N = (int)agg.pts.size();
		if (N < min_points_per_cell) continue;

		agg.mean /= (float)N;

		// 共分散
		Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
		for (const auto& p : agg.pts) {
			Eigen::Vector3f q = p - agg.mean;
			cov.noalias() += q * q.transpose();
		}
		cov /= (float)N;

		// 固有分解（昇順: λ0 ≤ λ1 ≤ λ2）
		Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es(cov);
		if (es.info() != Eigen::Success) continue;

		const auto& evals = es.eigenvalues();
		const auto& evecs = es.eigenvectors();

		float lam0 = std::max(0.0f, (float)evals(0));
		float lam1 = std::max(0.0f, (float)evals(1));
		float lam2 = std::max(0.0f, (float)evals(2));

		Eigen::Vector3f n = evecs.col(0); // 最小固有値の固有ベクトル
		// 上向きに整合（必要に応じてルール調整: nz を正に）
		if (n.z() < 0.0f) n = -n;

		float d = -n.dot(agg.mean);

		// セル座標を復元
		int z = (int)((kv.first >> 40) & 0xFFFFF);
		int r = (int)((kv.first >> 20) & 0xFFFFF);
		int s = (int)(kv.first & 0xFFFFF);

		// 品質指標（一般的な定義）
		float denom = std::max(lam2, 1e-9f);
		float linearity = (lam2 - lam1) / std::max(lam2, 1e-6f);
		float planarity = (lam1 - lam0) / std::max(lam2, 1e-6f);
		float sphericity = lam0 / std::max(lam2, 1e-6f);
		float uprightness = std::abs(n.z());

		AnchorPatch ap;
		ap.zone_idx = z;
		ap.ring_idx = r;
		ap.sector_idx = s;
		ap.centroid = agg.mean;
		ap.normal = n;
		ap.d = d;
		ap.point_count = N;
		ap.lambda1 = lam2; // 慣例的に λ1≥λ2≥λ3 と置く系に合わせるなら適宜リネーム
		ap.lambda2 = lam1;
		ap.lambda3 = lam0;
		ap.planarity = planarity;
		ap.linearity = linearity;
		ap.sphericity = sphericity;
		ap.uprightness = uprightness;

		patches.push_back(std::move(ap));
	}

	return patches;
}

// 追加: クラスタ点群 -> AnchorPatch平面へのロバスト距離中央値（gap_median）
static bool cluster_to_anchor_gap_median(
	const StepCluster& c,
	const std::vector<int>& cluster_indices,  // indices in nonground_cloud
	const pcl::PointCloud<pcl::PointXYZI>::Ptr& nonground_cloud,
	const AnchorPatch& ap,
	float& gap_median_out,
	float& xy_out)
{
	// 近傍XY距離（重心間）
	float dx = c.cx - ap.centroid(0);
	float dy = c.cy - ap.centroid(1);
	xy_out = std::sqrt(dx * dx + dy * dy);

	// パッチ法線をクラスタ法線方向に整合
	Eigen::Vector3f n = ap.normal;
	Eigen::Vector3f nc(c.nx, c.ny, c.nz);
	if (n.dot(nc) < 0.0f) n = -n;

	// 法線に対応した d を再計算（d = -n・centroid）
	float d = -n.dot(ap.centroid);

	// 各クラスタ点の点→平面距離 |n·p + d| の中央値
	std::vector<float> absdist;
	absdist.reserve(cluster_indices.size());
	for (int idx : cluster_indices) {
		const auto& p = (*nonground_cloud)[idx];
		float dist = std::fabs(n(0) * p.x + n(1) * p.y + n(2) * p.z + d);
		absdist.push_back(dist);
	}
	if (absdist.empty()) return false;

	gap_median_out = median_abs(absdist);
	return true;
}

// クラスタ重心 -> AnchorPatch平面の距離（重心距離）
static bool cluster_to_anchor_gap_centroid(
	const StepCluster& c,
	const AnchorPatch& ap,
	float& gap_out,
	float& xy_out)
{
	// XY近傍（重心間）
	float dx = c.cx - ap.centroid(0);
	float dy = c.cy - ap.centroid(1);
	xy_out = std::sqrt(dx * dx + dy * dy);

	// パッチ法線をクラスタ法線方向に整合
	Eigen::Vector3f n = ap.normal;
	Eigen::Vector3f nc(c.nx, c.ny, c.nz);
	if (n.dot(nc) < 0.0f) n = -n;

	// d = -n・centroid（アンカー側の重心で再計算）
	float d = -n.dot(ap.centroid);

	// クラスタの重心（StepClusterに既存）
	Eigen::Vector3f cc(c.cx, c.cy, c.cz);

	gap_out = std::fabs(n.dot(cc) + d);
	return std::isfinite(gap_out);
}


// 追加: クラスタ点群から planarity と sphericity を同時計算（λ1>=λ2>=λ3 の流儀に整合）
static bool compute_planarity_sphericity_from_indices(
	const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
	const std::vector<int>& idx,
	float& planarity_out,
	float& sphericity_out)
{
	if (!cloud || idx.size() < 3) return false;

	// 平均
	Eigen::Vector3f mean = Eigen::Vector3f::Zero();
	for (int i : idx) {
		const auto& p = (*cloud)[i];
		mean += Eigen::Vector3f(p.x, p.y, p.z);
	}
	mean /= static_cast<float>(idx.size());

	// 共分散
	Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
	for (int i : idx) {
		const auto& p = (*cloud)[i];
		Eigen::Vector3f v(p.x, p.y, p.z);
		v -= mean;
		cov += v * v.transpose();
	}
	cov /= std::max<int>(1, (int)idx.size() - 1);

	// SVD（固有分解）
	Eigen::JacobiSVD<Eigen::Matrix3f> svd(cov, Eigen::ComputeFullU);
	Eigen::Vector3f sv = svd.singularValues(); // ここは降順
	// 既存コードの流儀に合わせて最大=asc(2)、最小=asc(0)
	const float lambda1 = sv(0);
	const float lambda2 = sv(1);
	const float lambda3 = sv(2);

	if (lambda1 <= 1e-12f) return false;

	// 既存の定義に合わせる
	planarity_out = (lambda2 - lambda3) / lambda1;
	sphericity_out = (lambda3) / lambda1;
	return true;
}

// ===== 四隅点からの矩形面積計算 =====
static float compute_rectangular_area(
	const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
	const std::vector<int>& indices)
{
	if (indices.size() < 4) return 0.0f;

	// XY平面への投影で最小/最大を求める
	float min_x = std::numeric_limits<float>::max();
	float max_x = std::numeric_limits<float>::lowest();
	float min_y = std::numeric_limits<float>::max();
	float max_y = std::numeric_limits<float>::lowest();

	for (size_t i = 0; i < indices.size(); ++i) {
		const auto& p = cloud->points[indices[i]];
		if (p.x < min_x) min_x = p.x;
		if (p.x > max_x) max_x = p.x;
		if (p.y < min_y) min_y = p.y;
		if (p.y > max_y) max_y = p.y;
	}

	float width = max_x - min_x;
	float height = max_y - min_y;
	return width * height;
}

// ===== 凸包面積計算（簡易2D） =====
static float compute_convex_hull_area(
	const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
	const std::vector<int>& indices)
{
	if (indices.size() < 3) return 0.0f;

	// XY平面に投影した2D点群の凸包
	std::vector<std::pair<float, float>> points_2d;
	points_2d.reserve(indices.size());
	for (size_t i = 0; i < indices.size(); ++i) {
		const auto& p = cloud->points[indices[i]];
		points_2d.push_back(std::make_pair(p.x, p.y));
	}

	// Graham Scan による凸包
	auto cross = [](const std::pair<float, float>& O,
		const std::pair<float, float>& A,
		const std::pair<float, float>& B) -> float {
		return (A.first - O.first) * (B.second - O.second) -
			(A.second - O.second) * (B.first - O.first);
	};

	std::sort(points_2d.begin(), points_2d.end());

	std::vector<std::pair<float, float>> hull;
	// Lower hull
	for (size_t i = 0; i < points_2d.size(); ++i) {
		while (hull.size() >= 2 && cross(hull[hull.size() - 2], hull[hull.size() - 1], points_2d[i]) <= 0)
			hull.pop_back();
		hull.push_back(points_2d[i]);
	}
	// Upper hull
	size_t lower_size = hull.size();
	for (int i = (int)points_2d.size() - 2; i >= 0; --i) {
		while (hull.size() > lower_size &&
			cross(hull[hull.size() - 2], hull[hull.size() - 1], points_2d[i]) <= 0)
			hull.pop_back();
		hull.push_back(points_2d[i]);
	}
	hull.pop_back(); // 最後の点は重複

	// 凸包の面積（Shoelace formula）
	float area = 0.0f;
	for (size_t i = 0; i < hull.size(); ++i) {
		size_t j = (i + 1) % hull.size();
		area += hull[i].first * hull[j].second;
		area -= hull[j].first * hull[i].second;
	}
	return std::fabs(area) * 0.5f;
}

// 第二主成分の長さを計算（既にPCA済みのクラスタ用）
static float compute_second_component_length(const StepCluster& cluster) {
	// lambda2_raw は既に ClusterGeometry に格納されている
	// 第二主成分の長さ = sqrt(λ2) × 2σ（95%信頼区間なら2σ、99%なら3σ）
	const float sigma_factor = 2.0f; // 調整可能
	return sigma_factor * std::sqrt(cluster.geometry.lambda2_raw);
}

static bool check_lambda2_length(const StepCluster& cluster, float max_lambda2_length) {
	float len = compute_second_component_length(cluster);
	return len <= max_lambda2_length;
}

// 統合判定：第二主成分が短い（細長い）クラスタ同士のみ統合
static bool plane_merge_ok_with_shape_check(
	const StepCluster& A,
	const StepCluster& B,
	float height_tol,           // 既存の高さ差許容
	float max_lambda2_length,   // 第二主成分長の上限 [m]
	float angle_tol_deg = 15.0f // 法線角度差の許容 [度]
) {
	// 1) 既存の対称高さ差チェック
	if (!plane_height_diff_ok(A, B, height_tol)) {
		return false;
	}

	// 2) 第二主成分長のチェック（両方が基準以下であること）
	float len_A = compute_second_component_length(A);
	float len_B = compute_second_component_length(B);

	if (len_A > max_lambda2_length || len_B > max_lambda2_length) {
		// どちらかが「広すぎる」→ 異なる踏面の可能性
		return false;
	}

	// 3) 法線の向きチェック
	float angle_deg = compute_normal_angle_deg(A, B);
	if (angle_deg > angle_tol_deg) {
		return false;
	}

	//// 4) （オプション）第一主成分の方向が揃っているかチェック
	//// これにより「同じ向きの細長い踏面」のみ統合できる
	//Eigen::Vector3f dir_A = get_first_principal_direction(A);
	//Eigen::Vector3f dir_B = get_first_principal_direction(B);
	//float dir_dot = std::abs(dir_A.dot(dir_B));
	//if (dir_dot < 0.866f) { // cos(30°) ≈ 0.866
	//	return false; // 主軸が30度以上ずれていたら別クラスタ
	//}

	return true;
}

// ===== クラスタ幾何特徴量の計算（完全版） =====
static bool compute_cluster_geometry(
	const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
	std::vector<int>& indices,
	float nx, float ny, float nz,  // 法線ベクトル
	ClusterGeometry& geom_out)
{
	int n = (int)indices.size();
	if (n < 3) return false;

	// 立体矩形寸法計算
	geom_out.bbox_min_x = std::numeric_limits<float>::max();
	geom_out.bbox_max_x = std::numeric_limits<float>::lowest();
	geom_out.bbox_min_y = std::numeric_limits<float>::max();
	geom_out.bbox_max_y = std::numeric_limits<float>::lowest();
	geom_out.bbox_min_z = std::numeric_limits<float>::max();
	geom_out.bbox_max_z = std::numeric_limits<float>::lowest();

	for (int i = 0; i < n; ++i) {
		const auto& p = cloud->points[indices[i]];
		if (p.x < geom_out.bbox_min_x) geom_out.bbox_min_x = p.x;
		if (p.x > geom_out.bbox_max_x) geom_out.bbox_max_x = p.x;
		if (p.y < geom_out.bbox_min_y) geom_out.bbox_min_y = p.y;
		if (p.y > geom_out.bbox_max_y) geom_out.bbox_max_y = p.y;
		if (p.z < geom_out.bbox_min_z) geom_out.bbox_min_z = p.z;
		if (p.z > geom_out.bbox_max_z) geom_out.bbox_max_z = p.z;
	}

	// 1. 矩形面積（XYバウンディングボックス）
	geom_out.rect_area = (geom_out.bbox_max_x - geom_out.bbox_min_x) *
		(geom_out.bbox_max_y - geom_out.bbox_min_y);

	// 2. 実面積（凸包）
	geom_out.actual_area = compute_convex_hull_area(cloud, indices);
	if (geom_out.actual_area < 1e-6f) geom_out.actual_area = geom_out.rect_area;

	// 3. 点密度
	geom_out.point_density = (geom_out.actual_area > 1e-6f) ?
		(float)n / geom_out.actual_area : 0.0f;

	//// 1. 矩形面積
	//geom_out.rect_area = compute_rectangular_area(cloud, indices);

	//// 2. 実面積（凸包）
	//geom_out.actual_area = compute_convex_hull_area(cloud, indices);
	//if (geom_out.actual_area < 1e-6f) geom_out.actual_area = geom_out.rect_area;

	//// 3. 点密度
	//geom_out.point_density = (geom_out.actual_area > 1e-6f) ?
	//	(float)n / geom_out.actual_area : 0.0f;

	// 4. PCA（固有値・固有ベクトル計算）
	Eigen::MatrixXf M(n, 3);
	for (int i = 0; i < n; ++i) {
		const auto& p = cloud->points[indices[i]];
		M(i, 0) = p.x; M(i, 1) = p.y; M(i, 2) = p.z;
	}
	Eigen::Vector3f mean = M.colwise().mean();
	Eigen::MatrixXf C = M.rowwise() - mean.transpose();
	Eigen::Matrix3f cov = (C.adjoint() * C) / (float)(n - 1);

	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(cov);
	if (solver.info() != Eigen::Success) return false;

	// 固有値は昇順（λ3 <= λ2 <= λ1）で取得
	Eigen::Vector3f ev = solver.eigenvalues();
	Eigen::Matrix3f evec = solver.eigenvectors();

	geom_out.lambda3_raw = std::max(0.0f, ev(0)); // 最小
	geom_out.lambda2_raw = std::max(0.0f, ev(1)); // 中間
	geom_out.lambda1_raw = std::max(0.0f, ev(2)); // 最大

	float sum_raw = geom_out.lambda1_raw + geom_out.lambda2_raw + geom_out.lambda3_raw;
	if (sum_raw < 1e-9f) return false;

	geom_out.eigenvector3 = evec.col(0); // 最小固有値 → 法線方向
	geom_out.eigenvector2 = evec.col(1); // 中間固有値
	geom_out.eigenvector1 = evec.col(2); // 最大固有値 → 主軸方向

	// 法線の向きを揃える
	if (geom_out.eigenvector3.dot(Eigen::Vector3f(nx, ny, nz)) < 0.0f) {
		geom_out.eigenvector3 = -geom_out.eigenvector3;
	}

	// ===== ★追加：幅と長さの計算★ =====
	// 第一主成分方向（長さ）：各点を第一主成分軸に投影して範囲を求める
	float min_proj1 = std::numeric_limits<float>::max();
	float max_proj1 = std::numeric_limits<float>::lowest();

	// 第二主成分方向（幅）：各点を第二主成分軸に投影して範囲を求める
	float min_proj2 = std::numeric_limits<float>::max();
	float max_proj2 = std::numeric_limits<float>::lowest();

	for (int i = 0; i < n; ++i) {
		const auto& p = cloud->points[indices[i]];
		Eigen::Vector3f point(p.x, p.y, p.z);
		Eigen::Vector3f centered = point - mean;

		// 第一主成分軸への投影
		float proj1 = centered.dot(geom_out.eigenvector1);
		if (proj1 < min_proj1) min_proj1 = proj1;
		if (proj1 > max_proj1) max_proj1 = proj1;

		// 第二主成分軸への投影
		float proj2 = centered.dot(geom_out.eigenvector2);
		if (proj2 < min_proj2) min_proj2 = proj2;
		if (proj2 > max_proj2) max_proj2 = proj2;
	}

	geom_out.length = max_proj1 - min_proj1;  // 第一主成分方向の長さ
	geom_out.width = max_proj2 - min_proj2;   // 第二主成分方向の長さ

	// アスペクト比
	geom_out.aspect_ratio = (geom_out.width > 1e-6f) ?
		geom_out.length / geom_out.width : 0.0f;

	// 充填率（凸包面積 / 矩形面積）
	geom_out.fill_ratio = (geom_out.rect_area > 1e-6f) ?
		geom_out.actual_area / geom_out.rect_area : 0.0f;

	// 矩形度（凸包面積 / (長さ × 幅)）
	float pca_rect_area = geom_out.length * geom_out.width;
	geom_out.rectangularity = (pca_rect_area > 1e-6f) ?
		geom_out.actual_area / pca_rect_area : 0.0f;


	// 5. 正規化固有値
	geom_out.lambda1_norm = geom_out.lambda1_raw / sum_raw;
	geom_out.lambda2_norm = geom_out.lambda2_raw / sum_raw;
	geom_out.lambda3_norm = geom_out.lambda3_raw / sum_raw;

	// 6. 形状特徴量（実値ベース）
	float l1 = geom_out.lambda1_raw;
	float l2 = geom_out.lambda2_raw;
	float l3 = geom_out.lambda3_raw;

	if (l1 > 1e-9f) {
		geom_out.linearity_raw = (l1 - l2) / l1;
		geom_out.planarity_raw = (l2 - l3) / l1;
		geom_out.sphericity_raw = l3 / l1;
		geom_out.anisotropy_raw = (l1 - l3) / l1;
	}
	geom_out.omnivariance_raw = std::pow(l1 * l2 * l3, 1.0f / 3.0f);
	geom_out.surface_variation_raw = (sum_raw > 0) ? l3 / sum_raw : 0.0f;

	// 7. 形状特徴量（正規化ベース）
	float ln1 = geom_out.lambda1_norm;
	float ln2 = geom_out.lambda2_norm;
	float ln3 = geom_out.lambda3_norm;

	if (ln1 > 1e-9f) {
		geom_out.linearity_norm = (ln1 - ln2) / ln1;
		geom_out.planarity_norm = (ln2 - ln3) / ln1;
		geom_out.sphericity_norm = ln3 / ln1;
		geom_out.anisotropy_norm = (ln1 - ln3) / ln1;
	}
	geom_out.omnivariance_norm = std::pow(ln1 * ln2 * ln3, 1.0f / 3.0f);
	geom_out.surface_variation_norm = ln3;

	// 8. スケール情報
	geom_out.eigenvalue_sum = sum_raw;
	geom_out.eigenvalue_condition = (l3 > 1e-9f) ? l1 / l3 : 1e9f;
	geom_out.structural_tensor_norm = std::sqrt(l1 * l1 + l2 * l2 + l3 * l3);
	geom_out.eigenvalue_ratio_21 = (l1 > 1e-9f) ? l2 / l1 : 0.0f;
	geom_out.eigenvalue_ratio_32 = (l2 > 1e-9f) ? l3 / l2 : 0.0f;
	geom_out.weighted_planarity = (l2 - l3) * std::sqrt(l1);

	// 9. 法線統合指標
	Eigen::Vector3f v_min = evec.col(0); // 最小固有値の固有ベクトル
	Eigen::Vector3f normal(nx, ny, nz);

	// 向き揃え
	if (v_min.dot(normal) < 0.0f) v_min = -v_min;
	geom_out.normal_eigen_alignment = std::fabs(v_min.dot(normal));

	float abs_nz = std::fabs(nz);
	geom_out.oriented_planarity = geom_out.planarity_raw * abs_nz;
	geom_out.horizontal_confidence = geom_out.planarity_raw * (1.0f - abs_nz);

	// 10. 段差スコア
	geom_out.step_likelihood_score =
		abs_nz *
		geom_out.planarity_raw *
		(1.0f - geom_out.linearity_raw) *
		std::min(1.0f, geom_out.actual_area / 0.5f);

	// ===== ここに追加 =====
	// 11. 座標中央値の計算
	if (!compute_coordinate_medians(cloud, indices,
		geom_out.median_x,
		geom_out.median_y,
		geom_out.median_z)) {
		// 計算失敗時はデフォルト値（0）のまま
	}
	// ==================

	return true;
}

// 2つのベクトル間の回転行列を計算 (Rodrigues' rotation formula)
Eigen::Matrix3f Save::calc_rotation_from_vectors(const Eigen::Vector3f& from, const Eigen::Vector3f& to) {
	Eigen::Vector3f f = from.normalized();
	Eigen::Vector3f t = to.normalized();

	// 既に揃っている場合
	if (f.dot(t) > 0.9999f) {
		return Eigen::Matrix3f::Identity();
	}
	// 逆向きの場合 (180度回転)
	if (f.dot(t) < -0.9999f) {
		// 任意の直交軸周りに180度回す
		return Eigen::Matrix3f::Identity() * -1.0f; // 簡易実装
		// 本来は垂直ベクトルを見つけて180度回転させる
	}

	Eigen::Vector3f v = f.cross(t);
	float s = v.norm();
	float c = f.dot(t);

	Eigen::Matrix3f vx;
	vx << 0, -v(2), v(1),
		v(2), 0, -v(0),
		-v(1), v(0), 0;

	Eigen::Matrix3f R = Eigen::Matrix3f::Identity() + vx + vx * vx * ((1 - c) / (s * s));
	return R;
}

// サブマップを統合して局所地図を作成・保存する関数
void Save::process_submap_batch() {
	if (submap_batch_list_.empty()) return;

	pcl::PointCloud<pcl::PointXYZRGB> batch_cloud_world; // 世界座標系での統合点群
	Eigen::Vector3f accumulated_normal(0, 0, 0);
	int valid_normal_count = 0;

	std::cout << "[LocalMap] Processing batch of " << submap_batch_list_.size() << " submaps..." << std::endl;

	// 1. 各サブマップを読み込み -> 水平化解除 -> 世界座標配置
	for (const auto& meta : submap_batch_list_) {
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

		// ファイル読み込み (Leveled状態)
		if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(meta.pcd_path, *temp_cloud) == -1) {
			std::cerr << "[Error] Could not read " << meta.pcd_path << std::endl;
			continue;
		}

		// 変換行列作成: Leveled -> Sensor -> World
		// P_sensor = R_level^(-1) * P_leveled
		// P_world  = T_ws0 * P_sensor
		// よって T_total = T_ws0 * R_level.transpose()

		Eigen::Matrix4f T_leveled_to_sensor = Eigen::Matrix4f::Identity();
		T_leveled_to_sensor.block<3, 3>(0, 0) = meta.R_level.transpose();

		Eigen::Matrix4f T_total = meta.T_ws0 * T_leveled_to_sensor;

		pcl::transformPointCloud(*temp_cloud, *temp_cloud, T_total);

		// 単純な加算（間引きたい場合はここで行う）
		batch_cloud_world += *temp_cloud;

		// 平均法線計算用に加算 (z成分が正=上向きである前提)
		if (meta.ground_normal.norm() > 0.1) {
			accumulated_normal += meta.ground_normal;
			valid_normal_count++;
		}
	}

	if (batch_cloud_world.empty()) {
		submap_batch_list_.clear();
		return;
	}

	// 2. 平均法線の計算
	Eigen::Vector3f avg_normal(0, 0, 1);
	if (valid_normal_count > 0) {
		avg_normal = accumulated_normal.normalized();
	}

	// 3. 再水平化 (Re-leveling)
	// 世界座標系において、平均法線 avg_normal が Z軸(0,0,1) になるように回転させる
	Eigen::Matrix3f R_final_level = calc_rotation_from_vectors(avg_normal, Eigen::Vector3f(0, 0, 1));

	Eigen::Matrix4f T_final = Eigen::Matrix4f::Identity();
	T_final.block<3, 3>(0, 0) = R_final_level;

	// 座標変換実行 (World -> Local Leveled)
	pcl::transformPointCloud(batch_cloud_world, batch_cloud_world, T_final);

	// 4. 保存
	int start_id = submap_batch_list_.front().start_id;
	int end_id = submap_batch_list_.back().end_id;

	// 保存先ディレクトリ作成
	_mkdir("Output_PCD_LocalMaps");

	char out_filename[512];
	std::snprintf(out_filename, sizeof(out_filename),
		"Output_PCD_LocalMaps/local_map_%d_%d.pcd", start_id, end_id);

	pcl::io::savePCDFileBinary(out_filename, batch_cloud_world);

	std::cout << "[LocalMap] Saved re-leveled local map: " << out_filename << std::endl;
	std::cout << "           Avg Normal: " << avg_normal.transpose() << std::endl;

	// 5. リストをクリア
	submap_batch_list_.clear();
}

// 最新サブマップを確定→保存→分類→累積　こっちを使う
void Save::finalize_active_submap_(
	patchwork::Ground_Segmentation& GS,
	pcl::PointCloud<pcl::PointXYZRGB>& road_map_accum,
	pcl::PointCloud<pcl::PointXYZRGB>& solid_map_accum,
	pcl::PointCloud<pcl::PointXYZRGB>& wall_map_accum,
	pcl::PointCloud<pcl::PointXYZRGB>& float_map_accum
) {
	if (submap_active_.empty() || submap_start_id_ < 0) {
		// 何もない場合はリセットだけ
		submap_active_.clear();
		submap_count_ = 0;
		submap_start_id_ = -1;
		submap_end_id_ = -1;
		has_T_ws0_ = false; //永井追加．最初のセンサ座標用
		return;
	}

	//サブマップごとに閾値リセット（ドリフトによる全黒防止）
	GS.resetAdaptiveState(/*reset_thresholds=*/true);

	_mkdir(submap_output_base.c_str());
	std::string road_dir = submap_output_base + "/road";
	std::string solid_dir = submap_output_base + "/solid";
	std::string wall_dir = submap_output_base + "/wall";
	std::string float_dir = submap_output_base + "/float";
	std::string step_dir = submap_output_base + "/step";
	std::string promoted_dir = submap_output_base + "/promoted";

	// all_colored用のディレクトリパスを定義
	std::string all_colored_dir = submap_output_base + "/all_colored";

	_mkdir(road_dir.c_str());
	_mkdir(solid_dir.c_str());
	_mkdir(wall_dir.c_str());
	_mkdir(float_dir.c_str());
	_mkdir(step_dir.c_str());
	_mkdir(promoted_dir.c_str());
	_mkdir(all_colored_dir.c_str());

	int start_id = submap_start_id_;
	int end_id = submap_end_id_;
	char fn[512];

	// 例：サブマップ番号が分かったところで
	int sid = start_id;   // 既存の変数名に置き換えてください
	int eid = end_id;     // 既存の変数名に置き換えてください

	std::ostringstream oss;
	std::string submap_id;
	{
		oss << start_id << "-" << end_id;           // "2977-2987" 形式
		// oss << "submap_" << start_id << "_" << end_id; // 必要ならこちらの形式
		submap_id = oss.str();
	}

	GS.setSubmapId(oss.str());


	// 全点サブマップ保存（世界座標）XYZI
	{
		std::snprintf(fn, sizeof(fn), "%s/submap_%d_%d.pcd", submap_output_base.c_str(), start_id, end_id);
		pcl::io::savePCDFileBinary(fn, submap_active_);
	}

	// local(先頭センサ)へ戻すための T_sw0 を決定
	Eigen::Affine3f T_sw0 = Eigen::Affine3f::Identity();

	if (has_T_ws0_) {
		// センサ生座標を厳密に再現
		T_sw0 = Eigen::Affine3f(T_ws0_).inverse();
	}
	else {
		// 従来ロジック（robot_pose があればそれを使用、無ければ平行移動のみ）
		Eigen::Affine3f T_wl0 = Eigen::Affine3f::Identity();
		auto it = robot_pose.find(submap_start_id_);
		if (it != robot_pose.end()) {
			const auto& p = it->second;
			Eigen::Affine3d Tw =
				Eigen::Translation3d(p(1), p(2), p(3)) *
				Eigen::AngleAxisd(p(6), Eigen::Vector3d::UnitZ()) *
				Eigen::AngleAxisd(p(5), Eigen::Vector3d::UnitY()) *
				Eigen::AngleAxisd(p(4), Eigen::Vector3d::UnitX());
			T_wl0 = Tw.cast<float>();
		}
		else {
			Eigen::Vector4f minp, maxp; pcl::getMinMax3D<pcl::PointXYZI>(submap_active_, minp, maxp);
			Eigen::Vector3f c = 0.5f * (minp.head<3>() + maxp.head<3>());
			T_wl0 = Eigen::Translation3f(c.x(), c.y(), c.z());
		}
		T_sw0 = T_wl0.inverse();
	}

	// World→先頭センサへ
	pcl::PointCloud<pcl::PointXYZI> submap_local;
	pcl::transformPointCloud(submap_active_, submap_local, T_sw0.matrix());

	// ローカル化
	//pcl::PointCloud<pcl::PointXYZI> submap_local;
	//pcl::transformPointCloud(submap_active_, submap_local, T_l0w);

	// サブマップで路面推定
	Eigen::MatrixXf cloud_in;
	cloud_in.resize((int)submap_local.points.size(), 4);
	for (int i = 0; i < (int)submap_local.points.size(); ++i) {
		cloud_in.row(i) << submap_local.points[i].x,
			submap_local.points[i].y,
			submap_local.points[i].z,
			submap_local.points[i].intensity;
	}

	double old_max_range = GS.params_.max_range;
	if (old_max_range < 40.0) GS.params_.max_range = 60.0;
	GS.resetAdaptiveState(true);
	GS.estimateGround(cloud_in);

	// 永井追加：PCA結果をCSV出力
	//if (GS.params_.pca_check) {
	//	char pca_fn[512];
	//	std::snprintf(pca_fn, sizeof(pca_fn),
	//		"%s/submap_%d_%d_pca_results.csv",
	//		submap_output_base.c_str(), start_id, end_id);
	//	GS.exportPCAResults(pca_fn);

	//	// 次のサブマップのために結果をクリア
	//	GS.clearPCAResults();
	//}
	//GS.estimateGroundByGrid(cloud_in);
	GS.params_.max_range = old_max_range;


	// 分類取得
	Eigen::MatrixX3f ground = GS.getGround();
	Eigen::MatrixX3f nonground = GS.getNonground();
	Eigen::MatrixX3f border = GS.getborder();
	Eigen::MatrixX3f wall = GS.getwall();
	Eigen::MatrixX3f step = GS.getstep();

	// PCA結果取得と路面法線計算
	const auto& pca_results = GS.getPCAResults();

	Eigen::Matrix3f R_ground_to_horizontal = Eigen::Matrix3f::Identity();
	bool ground_normal_valid = false;

	if (!pca_results.empty()) {
		std::vector<float> nx_values, ny_values, nz_values;
		nx_values.reserve(pca_results.size());
		ny_values.reserve(pca_results.size());
		nz_values.reserve(pca_results.size());

		int filtered_count = 0;

		// フィルタリング：is_ground=1 かつ nz>=0.95 かつ point_count>=10
		for (size_t i = 0; i < pca_results.size(); ++i) {
			const auto& pr = pca_results[i];

			if (!pr.is_ground) continue;

			float nz = std::fabs((float)pr.normal(2));
			if (nz < 0.95f) continue;

			if (pr.point_count < 10) continue;

			float sign = (pr.normal(2) >= 0.0) ? 1.0f : -1.0f;

			nx_values.push_back((float)pr.normal(0) * sign);
			ny_values.push_back((float)pr.normal(1) * sign);
			nz_values.push_back(nz);

			filtered_count++;
		}

		JsonlLogger::instance().logEvent("ground_normal_filtering", [&](JsonWriter& jw) {
			jw.key("submap_id").string(submap_id);
			jw.key("total_patches").integer((long long)pca_results.size());
			jw.key("filtered_patches").integer((long long)filtered_count);
		});

		if (filtered_count >= 5) {
			float median_nx = compute_median(nx_values);
			float median_ny = compute_median(ny_values);
			float median_nz = compute_median(nz_values);

			Eigen::Vector3f ground_normal(median_nx, median_ny, median_nz);
			float norm = ground_normal.norm();

			if (norm > 1e-6f && median_nz > 0.7f) {
				ground_normal.normalize();

				Eigen::Vector3f z_axis(0.0f, 0.0f, 1.0f);
				Eigen::Vector3f v = ground_normal.cross(z_axis);
				float c = ground_normal.dot(z_axis);

				if (c > 0.9999f) {
					R_ground_to_horizontal = Eigen::Matrix3f::Identity();
				}
				else if (c < -0.9999f) {
					R_ground_to_horizontal << 1, 0, 0,
						0, -1, 0,
						0, 0, -1;
				}
				else {
					float s = v.norm();
					Eigen::Matrix3f v_skew;
					v_skew << 0, -v(2), v(1),
						v(2), 0, -v(0),
						-v(1), v(0), 0;

					R_ground_to_horizontal = Eigen::Matrix3f::Identity() +
						v_skew +
						v_skew * v_skew * ((1.0f - c) / (s * s));
				}

				ground_normal_valid = true;

				JsonlLogger::instance().logEvent("ground_normal_correction", [&](JsonWriter& jw) {
					jw.key("submap_id").string(submap_id);
					jw.key("ground_normal").beginObject();
					jw.key("nx").number(ground_normal(0), 6);
					jw.key("ny").number(ground_normal(1), 6);
					jw.key("nz").number(ground_normal(2), 6);
					jw.endObject();
					jw.key("num_filtered_patches").integer((long long)filtered_count);
				});
			}
		}
	}

	// フォールバック
	static Eigen::Matrix3f last_valid_R = Eigen::Matrix3f::Identity();
	static bool has_last_valid_R = false;

	if (!ground_normal_valid && has_last_valid_R) {
		R_ground_to_horizontal = last_valid_R;

		JsonlLogger::instance().logEvent("ground_normal_fallback", [&](JsonWriter& jw) {
			jw.key("submap_id").string(submap_id);
			jw.key("reason").string("using_previous_transform");
		});
	}
	else if (ground_normal_valid) {
		last_valid_R = R_ground_to_horizontal;
		has_last_valid_R = true;
	}

	// 一括座標変換
	if (ground_normal_valid || has_last_valid_R) {
		// Eigen行列に対して変換を適用
		ground = (R_ground_to_horizontal * ground.transpose()).transpose();
		nonground = (R_ground_to_horizontal * nonground.transpose()).transpose();
		border = (R_ground_to_horizontal * border.transpose()).transpose();
		wall = (R_ground_to_horizontal * wall.transpose()).transpose();
		step = (R_ground_to_horizontal * step.transpose()).transpose();

		JsonlLogger::instance().logEvent("all_categories_transformed", [&](JsonWriter& jw) {
			jw.key("submap_id").string(submap_id);
			jw.key("ground_points").integer((long long)ground.rows());
			jw.key("nonground_points").integer((long long)nonground.rows());
			jw.key("border_points").integer((long long)border.rows());
			jw.key("wall_points").integer((long long)wall.rows());
			jw.key("step_points").integer((long long)step.rows());
			jw.key("transform_applied").boolean(true);
		});
	}

	// =====CZM境界も変換
	Eigen::MatrixX3f czm_boundary;
	czm_boundary = GS.getCzmBoundary();

	if ((ground_normal_valid || has_last_valid_R) && czm_boundary.rows() > 0) {
		czm_boundary = (R_ground_to_horizontal * czm_boundary.transpose()).transpose();
	}

	// 先に路面セクタモルフォロジを適用
	//apply_road_sector_morphology(ground, nonground, GS, submap_id);

	// 2) モルフォロジ後の ground からアンカーを再生成
	std::vector<AnchorPatch> anchor_patches = build_anchor_patches_from_ground(ground, GS, /*min_points=*/10);

	// 永井追加．段差の検出回数別
	//Eigen::MatrixX3f step1 = GS.getStepIter(1);
	//Eigen::MatrixX3f step2 = GS.getStepIter(2);
	//Eigen::MatrixX3f step3 = GS.getStepIter(3);
	//Eigen::MatrixX3f step4 = GS.getStepIter(4);
	//Eigen::MatrixX3f step5p = GS.getStepIter(5); // 5回以上


	pcl::PointCloud<pcl::PointXYZRGB>::Ptr road(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr solid(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr wallc(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr floatc(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr stepc(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr stepc_rejected(new pcl::PointCloud<pcl::PointXYZRGB>); // 永井追加：リジェクト段差用
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr dump(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr czm_boundaryc(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr promotedc(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr step_bbox(new pcl::PointCloud<pcl::PointXYZRGB>);

	// デバッグモードが有効な場合のみCZMグリッド境界点を取得
	//Eigen::MatrixX3f czm_boundary;
	if (GS.params_.enable_czm_boundary) {
		czm_boundary = GS.getCzmBoundary();
		std::cout << "[DEBUG] Get time czm_boundary rows: "
			<< czm_boundary.rows()
			<< std::endl;
	}

	std::cout << "[DEBUG] Get time czm_boundary rows: "
		<< czm_boundary.rows()
		<< std::endl;

	// 永井追加．検出回数別の段差
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr stepc1(new pcl::PointCloud<pcl::PointXYZRGB>); // 1回目: Yellow
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr stepc2(new pcl::PointCloud<pcl::PointXYZRGB>); // 2回目: Magenta
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr stepc3(new pcl::PointCloud<pcl::PointXYZRGB>); // 3回目: Cyan
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr stepc4(new pcl::PointCloud<pcl::PointXYZRGB>); // 4回目: White
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr stepc5p(new pcl::PointCloud<pcl::PointXYZRGB>); // 5+ : Red

	auto appendXYZRGB = [](pcl::PointCloud<pcl::PointXYZRGB>::Ptr& dst, float x, float y, float z, uint8_t r, uint8_t g, uint8_t b) {
		pcl::PointXYZRGB pt; pt.x = x; pt.y = y; pt.z = z; pt.r = r; pt.g = g; pt.b = b; dst->push_back(pt);
	};

	std::cout << "[DEBUG] before append czm_boundary rows: "
		<< czm_boundary.rows()
		<< std::endl;

	if (GS.params_.enable_czm_boundary) {
		std::cout << "[DEBUG] before append czm_boundary rows: "
			<< czm_boundary.rows()
			<< std::endl;
		for (int i = 0; i < czm_boundary.rows(); ++i) {
			appendXYZRGB(czm_boundaryc, czm_boundary(i, 0), czm_boundary(i, 1), czm_boundary(i, 2), 255, 165, 0);
		}
		std::cout << "[DEBUG] after append czm_boundaryc points: "
			<< (czm_boundaryc ? czm_boundaryc->size() : 0)
			<< std::endl;
	}

	std::cout << "[DEBUG] after append czm_boundaryc points: "
		<< (czm_boundaryc ? czm_boundaryc->size() : 0)
		<< std::endl;

	for (int i = 0; i < ground.rows(); ++i) appendXYZRGB(road, ground(i, 0), ground(i, 1), ground(i, 2), 0, 255, 0);
	for (int i = 0; i < step.rows(); ++i) appendXYZRGB(stepc, step(i, 0), step(i, 1), step(i, 2), 255, 0, 0);

	//for (int i = 0; i < wall.rows(); ++i) appendXYZRGB(wallc, wall(i, 0), wall(i, 1), wall(i, 2), 0, 0, 255); //デバッグ用点群色付け（青）
	for (int i = 0; i < wall.rows(); ++i) appendXYZRGB(wallc, wall(i, 0), wall(i, 1), wall(i, 2), 0, 0, 0);

	//for (int i = 0; i < nonground.rows(); ++i) appendXYZRGB(solid, nonground(i, 0), nonground(i, 1), nonground(i, 2), 0, 0, 0);
	//for (int i = 0; i < border.rows(); ++i) appendXYZRGB(solid, border(i, 0), border(i, 1), border(i, 2), 0, 0, 0);

	// 永井追加．検出回数別の段差検出回数別色割り当て
	//for (int i = 0; i < step1.rows(); ++i) appendXYZRGB(stepc1, step1(i, 0), step1(i, 1), step1(i, 2), 255, 255, 0);   // Yellow
	//for (int i = 0; i < step2.rows(); ++i) appendXYZRGB(stepc2, step2(i, 0), step2(i, 1), step2(i, 2), 255, 0, 255);   // Magenta
	//for (int i = 0; i < step3.rows(); ++i) appendXYZRGB(stepc3, step3(i, 0), step3(i, 1), step3(i, 2), 0, 255, 255);   // Cyan
	//for (int i = 0; i < step4.rows(); ++i) appendXYZRGB(stepc4, step4(i, 0), step4(i, 1), step4(i, 2), 255, 122, 0); // White
	//for (int i = 0; i < step5p.rows(); ++i)appendXYZRGB(stepc5p, step5p(i, 0), step5p(i, 1), step5p(i, 2), 255, 0, 0);      // Red (5+)

		/********************************************************************
	 *  永井追加アルゴリズム: nonground の各点に対して近傍20点 PCA → 段差判定
	 ********************************************************************/

	 // 簡易append
	pcl::PointXYZRGB tmp_rgb;

	// GroundSegmentation 側で計算済みの CZM 路面パッチの PCA 結果を取得する。
	// 後段の「平面投影差の中央値」計算で路面基準として参照。
	// PCA結果を取得（この時点では clearPCAResults していない）
	//const std::vector<patchwork::PatchPCAResult>& pca_results = GS.getPCAResults();
	//const auto& pca_results = GS.getPCAResults();

	// デバッグログ用のCSV出力準備
	std::ofstream debug_log;
	{
		char fn[512];
		std::snprintf(fn, sizeof(fn),
			"%s/submap_%d_%d_cluster_debug.csv",
			submap_output_base.c_str(), start_id, end_id);
		debug_log.open(fn);
		if (debug_log) {
			debug_log << "seed_index,stage,iteration,points_before,points_after,"
				<< "points_removed,nz_before,nz_after,cx,cy,cz,"
				<< "y_min,y_max,y_width,z_min,z_max,z_range,action,reason\n";
		}
	}

	// Eigen 行列の non-ground（非路面）点群を PCL PointCloud に変換。
	// 以降、KD-tree 検索や距離計算はこの PCL 形式で行う。
	global_rejected_indices.clear();
	pcl::PointCloud<pcl::PointXYZI>::Ptr nonground_cloud(new pcl::PointCloud<pcl::PointXYZI>);
	nonground_cloud->reserve(static_cast<size_t>(nonground.rows()));
	for (int i = 0; i < nonground.rows(); ++i) {
		pcl::PointXYZI pt;
		pt.x = nonground(i, 0);
		pt.y = nonground(i, 1);
		pt.z = nonground(i, 2);
		pt.intensity = 0.0f;
		nonground_cloud->push_back(pt);
	}

	// 全点数と、候補クラスタに確定済みの点を二度と使わないためのマスク。
	int total_points = (int)nonground_cloud->size();
	std::vector<char> cluster_point_mask((size_t)total_points, 0);

	// 路面へ昇格させたクラスタの点を記録するマスク（solid から除外用）
	std::vector<char> promote_point_mask((size_t)total_points, 0);

	// 点数が最低点数（厳格）以上なら、候補生成のフローへ進む。
	if (total_points >= STEP_MIN_POINTS_STRICT) {
		// ① 非路面点群を y 昇順で並べ替えるための index/key 配列を準備。
		std::vector<int> sorted_indices((size_t)total_points);
		std::vector<float> sorted_keys((size_t)total_points);
		for (int i = 0; i < total_points; ++i) {
			sorted_indices[i] = i;
			sorted_keys[i] = (*nonground_cloud)[i].y;
		}
		if (total_points > 0) {
			// y座標の昇順に並べ替え（①に対応）
			quicksort_indices_by_key(&sorted_indices[0], &sorted_keys[0], 0, total_points - 1);
		}

		// 近傍探索用 KD-tree 構築
		pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
		kdtree.setInputCloud(nonground_cloud);

		// 線状ノイズ（壁のスキャンライン等）を平面と誤認しないよう、
		// 密度を均一化したデータで事前の形状チェックを行うために作成します。
		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZI>);
		pcl::KdTreeFLANN<pcl::PointXYZI> kdtree_downsampled;
		bool use_downsampled_check = false;

		if (nonground_cloud->size() > 50) {
			pcl::VoxelGrid<pcl::PointXYZI> sor;
			sor.setInputCloud(nonground_cloud);
			// 2cmグリッド (過密な線状データをスカスカにするためのサイズ)
			sor.setLeafSize(STEP_VOXEL_SIZE, STEP_VOXEL_SIZE, STEP_VOXEL_SIZE);
			sor.filter(*cloud_downsampled);

			if (cloud_downsampled->size() > 20) {
				kdtree_downsampled.setInputCloud(cloud_downsampled);
				use_downsampled_check = true;
			}
		}

		// 生成された段差「候補」クラスタの格納先（確定ではない）
		std::vector<StepCluster> candidate_clusters;
		candidate_clusters.reserve((size_t)(total_points / STEP_MIN_POINTS_STRICT + 1));

		std::ofstream seedlog;
		{
			char fn[512];
			std::snprintf(fn, sizeof(fn),
				"%s/submap_%d_%d_seed_log.csv",
				submap_output_base.c_str(), start_id, end_id);
			seedlog.open(fn);
			if (seedlog) {
				seedlog << "seed_index,x,y,z,initial_nz,initial_n,pass_nz,reason\n";
			}
		}

		std::ofstream seed_member_log;
		std::string seed_mem_path = submap_output_base + "/submap_" + std::to_string(submap_start_id_) + "_" + std::to_string(submap_end_id_) + "_seed_members.csv";
		seed_member_log.open(seed_mem_path);
		if (seed_member_log) seed_member_log << "seed_index,point_index,x,y,z\n";

		// ===== ここに追加：距離チェック用ログ =====
		std::ofstream distlog;
		{
			char fn2[512];
			std::snprintf(fn2, sizeof(fn2),
				"%s/submap_%d_%d_distance_reject_log.csv",
				submap_output_base.c_str(), start_id, end_id);
			distlog.open(fn2);
			if (distlog) {
				distlog << "seed_index,num_points,cx,cy,cz,median_x,median_y,median_z,"
					<< "xy_distance_from_origin,threshold,rejected\n";
			}
		}

		std::ofstream stagelog;
		{
			char fn2[512];
			std::snprintf(fn2, sizeof(fn2),
				"%s/submap_%d_%d_stage_log.csv",
				submap_output_base.c_str(), start_id, end_id);
			stagelog.open(fn2);
			if (stagelog) {
				// seed 以降の各段階で1行ずつ出す
				stagelog << "seed_index,x,y,z,stage,nz,num_points,iter,reason\n";
			}
		}

		// ② y 昇順で 1 点ずつ注目（seed）→ KNN 20 点 + seed の計 21 点で初期 PCA
		for (int si = 0; si < total_points; ++si) {
			int seed_index = sorted_indices[si];

			if (cluster_point_mask[(size_t)seed_index]) {
				continue;
			}

			const pcl::PointXYZI& sp = (*nonground_cloud)[seed_index];

			// まずは疎なデータで形状確認

			if (use_downsampled_check) {
				std::vector<int> ds_indices;
				std::vector<float> ds_dists;
				int k_check = 20; // 判定に使う点数

				// 注目点(生座標)を使って、ダウンサンプリング空間を検索
				// 戻り値が足りない場合は判定不能なので次へ
				if (kdtree_downsampled.nearestKSearch(sp, k_check, ds_indices, ds_dists) >= 3) {

					// PCA用の点群を作成
					pcl::PointCloud<pcl::PointXYZI>::Ptr check_cloud(new pcl::PointCloud<pcl::PointXYZI>);
					check_cloud->reserve(ds_indices.size() + 1);
					check_cloud->push_back(sp); // 自分自身も含める
					for (int idx : ds_indices) {
						check_cloud->push_back((*cloud_downsampled)[idx]);
					}

					// PCA計算
					Eigen::Vector4f centroid;
					pcl::compute3DCentroid(*check_cloud, centroid);
					Eigen::Matrix3f covariance_matrix;
					pcl::computeCovarianceMatrixNormalized(*check_cloud, centroid, covariance_matrix);
					Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance_matrix, Eigen::ComputeEigenvectors);

					// 法線ベクトル (最小固有値に対応する固有ベクトル)
					Eigen::Vector3f normal = eigen_solver.eigenvectors().col(0);

					if (normal.z() < 0) {
						normal = -normal;
					}
					if (normal.z() < STEP_NZ_THRESH_SEED) {
						if (debug_log) {
							debug_log << seed_index << ",ds_check_fail,0,"
								<< ds_indices.size() << ",0,0,"
								<< normal.z() << ",0," << sp.x << "," << sp.y << "," << sp.z << ","
								<< "0,0,0,0,0,0,FAIL,ds_normal_low\n";
						}
						continue;
					}
				}
			}
			// 生データで本来のシード形成を行う
			// KNN収集 (生データ)
			std::vector<int> knn_indices;
			knn_indices.reserve(STEP_K_NEIGHBOR + 1);
			if (!gather_knn_indices(kdtree, nonground_cloud, seed_index, STEP_K_NEIGHBOR + 1, knn_indices)) {
				if (debug_log) {
					debug_log << seed_index << ",knn_gather,0,"
						<< 0 << "," << 0 << "," << 0 << ","
						<< "0,0," << sp.x << "," << sp.y << "," << sp.z << ","
						<< "0,0,0,0,0,0,FAIL,insufficient_neighbors\n";
				}
				continue;
			}
			if (seed_member_log) {
				for (int kidx : knn_indices) {
					const auto& p = (*nonground_cloud)[kidx];
					seed_member_log << seed_index << "," << kidx << ","
						<< p.x << "," << p.y << "," << p.z << "\n";
				}
			}

			// 初期クラスタ候補
			StepCluster cluster_candidate;
			cluster_candidate.indices = knn_indices;

			int initial_points = (int)cluster_candidate.indices.size();

			// 生データで正確な平面パラメータを計算
			if (!compute_plane_for_indices(nonground_cloud, cluster_candidate.indices, cluster_candidate)) {
				if (debug_log) {
					debug_log << seed_index << ",initial_pca,0,"
						<< initial_points << ",0," << initial_points << ","
						<< "0,0," << sp.x << "," << sp.y << "," << sp.z << ","
						<< "0,0,0,0,0,0,FAIL,pca_failed\n";
				}
				continue;
			}

			// 平面性チェック
			// まず幾何特徴量を計算する
			// compute_cluster_geometry は Save.cpp 内にある static 関数を利用
			compute_cluster_geometry(nonground_cloud, cluster_candidate.indices,
				cluster_candidate.nx, cluster_candidate.ny, cluster_candidate.nz,
				cluster_candidate.geometry);

			//// 閾値チェック: Planarity が 0.5 未満なら棄却
			//if (cluster_candidate.geometry.planarity_raw < 0.3f) {
			//	if (debug_log) {
			//		int initial_points = (int)cluster_candidate.indices.size();
			//		debug_log << seed_index << ",planarity_check,0,"
			//			<< initial_points << ",0," << initial_points << ","
			//			<< cluster_candidate.geometry.planarity_raw << ",0," // nz_before の代わりに planarity を記録
			//			<< cluster_candidate.cx << "," << cluster_candidate.cy << "," << cluster_candidate.cz << ","
			//			<< "0,0,0,0,0,0,FAIL,low_planarity\n";
			//	}
			//	// シードログにも残す場合
			//	if (seedlog) {
			//		const auto& sp = (*nonground_cloud)[seed_index];
			//		seedlog << seed_index << "," << sp.x << "," << sp.y << "," << sp.z << ","
			//			<< cluster_candidate.nz << "," << knn_indices.size() << ","
			//			<< cluster_candidate.geometry.planarity_raw << ",rejected_low_planarity\n";
			//	}
			//	continue;
			//}

			float initial_nz = cluster_candidate.nz;

			// 直立性ゲート
			if (cluster_candidate.nz < STEP_NZ_THRESH_SEED) {
				if (debug_log) {
					debug_log << seed_index << ",nz_check,0,"
						<< initial_points << "," << initial_points << ",0,"
						<< initial_nz << "," << initial_nz << ","
						<< cluster_candidate.cx << "," << cluster_candidate.cy << "," << cluster_candidate.cz << ","
						<< "0,0,0,0,0,0,FAIL,nz_below_threshold\n";
				}
				continue;
			}

			if (debug_log) {
				debug_log << seed_index << ",nz_check,0,"
					<< initial_points << "," << initial_points << ",0,"
					<< initial_nz << "," << initial_nz << ","
					<< cluster_candidate.cx << "," << cluster_candidate.cy << "," << cluster_candidate.cz << ","
					<< "0,0,0,0,0,0,PASS,nz_ok\n";
			}

		// 拡張フェーズ（R-GPFライクな「固定領域での反復リファイン」に変更）
		bool cluster_valid = true;

		// 1) 空間的な拡張は一回だけ行う
		//    重心が移動して壁を吸い込む現象を防ぐため、探索中心を固定します。
		pcl::PointXYZI fixed_center_point;
		fixed_center_point.x = cluster_candidate.cx;
		fixed_center_point.y = cluster_candidate.cy;
		fixed_center_point.z = cluster_candidate.cz;

		std::vector<int> raw_indices;
		std::vector<float> raw_dists;
		
		// 半径3.0mで近傍点を一括取得（まだ平面距離等のフィルタはかけない生データ）
		if (kdtree.radiusSearch(fixed_center_point, STEP_RADIUS_GROW, raw_indices, raw_dists) > 0) {
			
			// 2) 固定された点群の中で、平面フィッティングを3回回す
			for (int fit_iter = 0; fit_iter < 3; ++fit_iter) {

				std::vector<int> next_indices;
				next_indices.reserve(raw_indices.size());

				// 現在の平面パラメータでフィルタリング（選抜）
				float current_cx = cluster_candidate.cx;
				float current_cy = cluster_candidate.cy;
				float current_cz = cluster_candidate.cz;
				
				for (int idx : raw_indices) {
					// 既に確定したクラスタに含まれる点は除外
					if (cluster_point_mask[idx]) continue;

					const auto& pt = (*nonground_cloud)[idx];

					//// 条件A: 重心からのZ差 (スロープ許容のため15cm確保)
					//if (std::fabs(pt.z - current_cz) > 0.05f) continue;

					// 条件B: 平面からの垂直距離 (STEP_PLANE_DIST_THRESH = 5cm)
					float dist = point_plane_dist(pt.x, pt.y, pt.z, 
												  cluster_candidate.nx, cluster_candidate.ny, cluster_candidate.nz, cluster_candidate.d);
					if (std::fabs(dist) > STEP_PLANE_DIST_THRESH) continue;

					// 合格した点のみリストに追加
					next_indices.push_back(idx);
				}

				// 点数が少なすぎたら失敗
				if (next_indices.size() < STEP_MIN_POINTS_LOOSE) {
					// 最初のイテレーションで失敗したらクラスタ自体を無効化
					if (fit_iter == 0) cluster_valid = false;
					break; 
				}

				// 選抜された点群でクラスタ情報を更新
				cluster_candidate.indices = next_indices;

				// 平面パラメータ(法線・重心)を再計算 (Fitting)
				// これにより、より確からしい平面へと収束していく
				if (!compute_cluster_geometry(nonground_cloud, cluster_candidate.indices,
					cluster_candidate.nx, cluster_candidate.ny, cluster_candidate.nz, cluster_candidate.geometry)) {
					cluster_valid = false;
					break;
				}
				
				// パラメータ更新 (dも更新)
				// PCLの関数で重心(Vector4f)を計算し、cx, cy, cz にセットする
				Eigen::Vector4f new_centroid;
				pcl::compute3DCentroid(*nonground_cloud, cluster_candidate.indices, new_centroid);

				cluster_candidate.cx = new_centroid[0];
				cluster_candidate.cy = new_centroid[1];
				cluster_candidate.cz = new_centroid[2];
				cluster_candidate.d = -(cluster_candidate.nx * cluster_candidate.cx + 
										cluster_candidate.ny * cluster_candidate.cy + 
										cluster_candidate.nz * cluster_candidate.cz);

				// ログ出力 (各フィッティング段階の状態を確認)
				if (debug_log) {
					float y_min, y_max, z_min, z_max;
					compute_yz_minmax(nonground_cloud, cluster_candidate.indices, y_min, y_max, z_min, z_max);
					debug_log << seed_index << ",fit_iter," << fit_iter << ","
						<< raw_indices.size() << "," << cluster_candidate.indices.size() << ","
						<< 0 << ","
						<< cluster_candidate.nz << "," << cluster_candidate.nz << ","
						<< cluster_candidate.cx << "," << cluster_candidate.cy << "," << cluster_candidate.cz << ","
						<< y_min << "," << y_max << "," << (y_max - y_min) << ","
						<< z_min << "," << z_max << "," << (z_max - z_min) << ","
						<< "REFINE,r_gpf_style\n";
				}
			}
		} else {
			cluster_valid = false;
		}

		if (!cluster_valid) {
			continue;
		}

		// ⑩ 候補クラスタとして確定：マスク更新
		for (int idx : cluster_candidate.indices) {
			cluster_point_mask[idx] = 1;
		}

			// ===== 幾何特徴量の計算（法線情報も渡す）=====
			if (!compute_cluster_geometry(
				nonground_cloud,
				cluster_candidate.indices,
				cluster_candidate.nx,
				cluster_candidate.ny,
				cluster_candidate.nz,
				cluster_candidate.geometry)) {
				// 計算失敗時はデフォルト値のまま（警告は任意）
				if (seedlog) {
					seedlog << seed_index << "," << sp.x << "," << sp.y << "," << sp.z
						<< "," << cluster_candidate.nz << ","
						<< (int)cluster_candidate.indices.size()
						<< "," << 0 << ",geometry_computation_failed\n";
				}
			}
			// ==================

			// ===== XY平面上の中央値距離チェック（詳細ログ版） =====
			float median_x = cluster_candidate.geometry.median_x;
			float median_y = cluster_candidate.geometry.median_y;
			float xy_distance_from_origin = std::sqrt(median_x * median_x + median_y * median_y);

			if (debug_log) {
				float y_min, y_max, z_min, z_max;
				compute_yz_minmax(nonground_cloud, cluster_candidate.indices,
					y_min, y_max, z_min, z_max);

				std::ostringstream reason;
				reason << "xy_dist_check=" << std::fixed << std::setprecision(3) << xy_distance_from_origin;

				debug_log << seed_index << ",final_check,4,"
					<< cluster_candidate.indices.size() << ","
					<< cluster_candidate.indices.size() << ",0,"
					<< cluster_candidate.nz << "," << cluster_candidate.nz << ","
					<< cluster_candidate.cx << "," << cluster_candidate.cy << "," << cluster_candidate.cz << ","
					<< y_min << "," << y_max << "," << (y_max - y_min) << ","
					<< z_min << "," << z_max << "," << (z_max - z_min) << ","
					<< (xy_distance_from_origin <= step_max_xy_distance_ ? "PASS" : "FAIL") << ","
					<< reason.str() << "\n";
			}

			const float MAX_XY_DISTANCE = step_max_xy_distance_;
			bool rejected_by_distance = (xy_distance_from_origin > MAX_XY_DISTANCE);

			// 距離チェックログに記録
			if (distlog) {
				distlog << seed_index << ","
					<< cluster_candidate.indices.size() << ","
					<< cluster_candidate.cx << ","
					<< cluster_candidate.cy << ","
					<< cluster_candidate.cz << ","
					<< median_x << ","
					<< median_y << ","
					<< cluster_candidate.geometry.median_z << ","
					<< xy_distance_from_origin << ","
					<< MAX_XY_DISTANCE << ","
					<< (rejected_by_distance ? 1 : 0) << "\n";
			}

			//total_seeds_processed++;

			if (rejected_by_distance) {
				// このクラスタの点をマスクから解除
				for (size_t pi = 0; pi < cluster_candidate.indices.size(); ++pi) {
					int idx = cluster_candidate.indices[pi];
					cluster_point_mask[(size_t)idx] = 0;
				}

				// シードログにも記録
				if (seedlog) {
					seedlog << seed_index << "," << sp.x << "," << sp.y << "," << sp.z
						<< "," << cluster_candidate.nz << ","
						<< (int)cluster_candidate.indices.size()
						<< "," << 0 << ",rejected_xy_dist_" << xy_distance_from_origin << "\n";
				}

				//rejected_by_xy_distance++;

				continue; // 次のseedへ
			}
			// ==================

			candidate_clusters.push_back(cluster_candidate);

		}

		if (debug_log) {
			debug_log.close();
		}

		// 候補クラスタ配列に、段差確定フラグ等の管理情報を付与するバッファを用意
		std::vector<CandidateInfo> candidate_infos(candidate_clusters.size());
		std::vector<std::string> acceptance_reason(candidate_clusters.size(), std::string("rejected"));
		for (size_t ci = 0; ci < candidate_clusters.size(); ++ci) {
			candidate_infos[ci].cluster = candidate_clusters[ci];
		}

		////　候補クラスタの統合（cy昇順、対称高さ差で取り込み） ===
		//if (!candidate_infos.empty()) {
		//	// 最大3パス。収束したら早期終了
		//	for (int pass = 0; pass < 20; ++pass) {
		//		if (candidate_infos.size() <= 1) break;

		//		// 1) cy 昇順に並べ替え（パスごとにやり直し）
		//		std::vector<int> order(candidate_infos.size());
		//		std::iota(order.begin(), order.end(), 0);
		//		std::sort(order.begin(), order.end(), [&](int a, int b) {
		//			return candidate_infos[a].cluster.cy < candidate_infos[b].cluster.cy;
		//		});

		//		// 未処理フラグ（パスごとに初期化）
		//		std::vector<char> alive(candidate_infos.size(), 1);
		//		int merged_count_this_pass = 0; // この回でのマージ件数

		//		// 2) 注目クラスタ（cy小→大）ごとに、近傍＋高さ差で取り込み
		//		for (size_t oi = 0; oi < order.size(); ++oi) {
		//			int i = order[oi];
		//			if (!alive[i]) continue;

		//			StepCluster& focus = candidate_infos[i].cluster;
		//			bool merged_this_focus = false;

		//			for (size_t oj = oi + 1; oj < order.size(); ++oj) {
		//				int j = order[oj];
		//				if (!alive[j]) continue;

		//				StepCluster& other = candidate_infos[j].cluster;

		//				// XY半径チェック
		//				float dx = other.cx - focus.cx;
		//				float dy = other.cy - focus.cy;
		//				if (dx * dx + dy * dy > MERGE_RADIUS_XY * MERGE_RADIUS_XY) continue;

		//				// “平面の高さ差”（対称）チェック
		//				if (!plane_height_diff_ok(focus, other, MERGE_HEIGHT_TOL)) continue;

		//				// indices を重複なく取り込み
		//				append_unique_indices(focus.indices, other.indices);

		//				// 取り込み済みは探索対象から除外
		//				alive[j] = 0;
		//				++merged_count_this_pass;     // ★件数カウント
		//				merged_this_focus = true;
		//			}

		//			// 3) 仕上げ（取り込みがあった focus のみ）：再推定 → z窓 → 再推定
		//			if (merged_this_focus) {
		//				refine_plane_with_trim(nonground_cloud, focus.indices, STEP_MIN_POINTS_STRICT, focus);
		//				clip_indices_by_z_window(nonground_cloud, focus.indices, focus.cz, Z_CLIP_HALF, i);
		//				// clip後、重心や幾何情報再計算
		//				refine_plane_with_trim(nonground_cloud, focus.indices, STEP_MIN_POINTS_STRICT, focus);
		//				compute_cluster_geometry(nonground_cloud, focus.indices,
		//					focus.nx, focus.ny, focus.nz,
		//					focus.geometry);
		//			}
		//		}

		//		// 4) 圧縮：alive のみ残す（acceptance_reason も同じ順で圧縮）
		//		if (merged_count_this_pass == 0) break;  // この回でのマージ0なら収束

		//		std::vector<CandidateInfo> next_infos;
		//		next_infos.reserve(candidate_infos.size());
		//		std::vector<std::string> next_reason;
		//		next_reason.reserve(acceptance_reason.size());

		//		for (size_t k = 0; k < candidate_infos.size(); ++k) {
		//			if (alive[k]) {
		//				next_infos.emplace_back(std::move(candidate_infos[k]));
		//				next_reason.emplace_back(std::move(acceptance_reason[k]));
		//			}
		//		}
		//		candidate_infos.swap(next_infos);
		//		acceptance_reason.swap(next_reason);

		//		// → 次へ
		//	}

		//}

		const float DENSITY_THRESH_FINAL = 10.0f;
		const float CLUSTER_MAX_HEIGHT = 1.2f; // 重心高さ上限
		const int CLUSTER_MIN_POINTS = 100;    // 最低点数 (これ以下はNG)

		for (size_t i = 0; i < candidate_infos.size(); ++i) {
			// 1. 点数チェック (100点以下は不合格)
			//    ※ STEP_MIN_POINTS_LOOSE (5点) よりも厳しい条件で上書きします
			if (candidate_infos[i].cluster.indices.size() <= CLUSTER_MIN_POINTS) continue;

			// 2. 高さチェック (重心が1.2m以上なら不合格)
			if (candidate_infos[i].cluster.cz >= CLUSTER_MAX_HEIGHT) continue;

			// 3. 密度チェック
			float area = candidate_infos[i].cluster.geometry.rect_area;
			float num = (float)candidate_infos[i].cluster.indices.size();

			if (area < 0.01f) continue;

			float density = num / area;

			if (density >= DENSITY_THRESH_FINAL) {
				candidate_infos[i].is_step = true;
				acceptance_reason[i] = "accepted_density_pass";
			}
		}

		//// ============================================================
		//// 矩形IoUでの階段踏面ふるい
		//// ============================================================

		//// -----------------------------------------------------------
		//// 矩形IoU（AABB, 重心合わせ）の計算
		//// -----------------------------------------------------------
		//// 矩形IoU（AABB, 5%トリム → 矩形中心合わせ）
		//auto compute_rect_iou_trimmed_center_aligned = [&](const StepCluster& A, const StepCluster& B) -> float {
		//	// 5%トリム数を計算（最低1点）
		//	auto make_trimmed = [&](const StepCluster& C, std::vector<int>& out) -> bool {
		//		const auto n = (int)C.indices.size();
		//		if (n < 4) return false;
		//		int trim = (int)std::ceil(n * 0.05f);
		//		if (trim < 1) trim = 1;
		//		if (n - trim < 3) return false;

		//		// (cx, cy) からのXY距離^2でソートし、遠い上位trimを捨てる
		//		std::vector<std::pair<float, int>> dist_idx;
		//		dist_idx.reserve(n);
		//		for (int idx : C.indices) {
		//			const auto& p = nonground_cloud->points[idx];
		//			float dx = p.x - C.cx;
		//			float dy = p.y - C.cy;
		//			float d2 = dx * dx + dy * dy;
		//			dist_idx.emplace_back(d2, idx);
		//		}
		//		// 近い順 [0 .. n-trim) を残す
		//		std::nth_element(dist_idx.begin(), dist_idx.end() - trim, dist_idx.end(),
		//			[](const auto& a, const auto& b) { return a.first < b.first; });
		//		out.clear();
		//		out.reserve(n - trim);
		//		for (int i = 0; i < n - trim; ++i) out.push_back(dist_idx[i].second);
		//		return true;
		//	};

		//	auto aabb_global = [&](const std::vector<int>& idxs, float& x1, float& y1, float& x2, float& y2) {
		//		x1 = std::numeric_limits<float>::max();
		//		y1 = std::numeric_limits<float>::max();
		//		x2 = std::numeric_limits<float>::lowest();
		//		y2 = std::numeric_limits<float>::lowest();
		//		for (int idx : idxs) {
		//			const auto& p = nonground_cloud->points[idx];
		//			if (p.x < x1) x1 = p.x; if (p.x > x2) x2 = p.x;
		//			if (p.y < y1) y1 = p.y; if (p.y > y2) y2 = p.y;
		//		}
		//	};

		//	std::vector<int> idxA, idxB;
		//	if (!make_trimmed(A, idxA) || !make_trimmed(B, idxB)) return 0.0f;

		//	float ax1, ay1, ax2, ay2, bx1, by1, bx2, by2;
		//	aabb_global(idxA, ax1, ay1, ax2, ay2);
		//	aabb_global(idxB, bx1, by1, bx2, by2);

		//	float aw = ax2 - ax1, ah = ay2 - ay1;
		//	float bw = bx2 - bx1, bh = by2 - by1;
		//	if (aw <= 0.0f || ah <= 0.0f || bw <= 0.0f || bh <= 0.0f) return 0.0f;

		//	// 中心合わせ（位置差を無視し、サイズだけ一致させる）
		//	float iw = std::min(aw, bw);
		//	float ih = std::min(ah, bh);
		//	float inter = iw * ih;

		//	float area_a = aw * ah;
		//	float area_b = bw * bh;
		//	float uni = area_a + area_b - inter;
		//	if (uni < 1e-9f) return 0.0f;
		//	return inter / uni;
		//};


		//// -----------------------------------------------------------
		//// 【しきい値定義】（引継ぎ資料の最終仕様）
		//// -----------------------------------------------------------
		//const float CX_MAX = 0.75f;         // 位置ゲート: |cx| の上限（両方OK必須）
		//const float DCY_MAX = 0.70f;        // 位置ゲート: |Δcy| の上限
		//const float DCY_MIN = 0.15f;        // 位置ゲート: |Δcy| の下限
		//const float DCZ_MAX = 0.30f;        // 位置ゲート: |Δcz| の上限
		//const float DCZ_MIN = 0.10f;		// 位置ゲート: |Δcz| の下限
		//const float IOU_MIN = 0.40f;        // 形状ゲート: 矩形IoU の下限

		//// ラスト1段＋廊下検出用パラメータ
		//const int   LASTSTEP_LARGE_MIN_POINTS = 800;   // 大きいクラスタの点数下限
		//const float LASTSTEP_CONTAINMENT_MIN = 0.50f; // containment_small のしきい値
		//const float LASTSTEP_CX_TOL_RATIO = 0.5f;  // 中心差許容: 0.5 * min(width_small, width_large)

		//int N = (int)candidate_infos.size();

		//// -----------------------------------------------------------
		//// 【密度ゲート】5%トリムAABBで矩形面積と点密度を算出→ 密度50以下を排除
		//// -----------------------------------------------------------
		//const float DENSITY_MIN = 50.0f;

		//// 5%トリムで残す点集合を作る（(cx,cy)からの距離の“遠い順”上位5%を削除）
		//auto make_trimmed_idx = [&](const StepCluster& C, std::vector<int>& out)->bool {
		//	const int n = (int)C.indices.size();
		//	if (n < 4) return false;
		//	int trim = (int)std::ceil(n * 0.05f);
		//	if (trim < 1) trim = 1;
		//	if (n - trim < 3) return false;

		//	std::vector<std::pair<float, int>> dist_idx;
		//	dist_idx.reserve(n);
		//	for (int idx : C.indices) {
		//		const auto& p = nonground_cloud->points[idx];
		//		float dx = p.x - C.cx;
		//		float dy = p.y - C.cy;
		//		float d2 = dx * dx + dy * dy;
		//		dist_idx.emplace_back(d2, idx);
		//	}
		//	// 近い順 [0 .. n-trim) を残す
		//	std::nth_element(dist_idx.begin(), dist_idx.end() - trim, dist_idx.end(),
		//		[](const auto& a, const auto& b) { return a.first < b.first; });

		//	out.clear(); out.reserve(n - trim);
		//	for (int i = 0; i < n - trim; ++i) out.push_back(dist_idx[i].second);
		//	return true;
		//};

		//// AABB（XY）を計算して幅・高さ・面積を返す
		//auto aabb_xy = [&](const std::vector<int>& idxs, float& w, float& h, float& area)->bool {
		//	if (idxs.size() < 3) return false;
		//	float x1 = std::numeric_limits<float>::max(), y1 = std::numeric_limits<float>::max();
		//	float x2 = std::numeric_limits<float>::lowest(), y2 = std::numeric_limits<float>::lowest();
		//	for (int idx : idxs) {
		//		const auto& p = nonground_cloud->points[idx];
		//		if (p.x < x1) x1 = p.x; if (p.x > x2) x2 = p.x;
		//		if (p.y < y1) y1 = p.y; if (p.y > y2) y2 = p.y;
		//	}
		//	w = x2 - x1; h = y2 - y1;
		//	if (w <= 0.0f || h <= 0.0f) return false;
		//	area = w * h;
		//	return true;
		//};

		//// 統合後、密度ゲート判定の前に全クラスタの幾何情報を計算
		//for (int i = 0; i < N; ++i) {
		//	if (!compute_cluster_geometry(
		//		nonground_cloud,
		//		candidate_infos[i].cluster.indices,
		//		candidate_infos[i].cluster.nx,
		//		candidate_infos[i].cluster.ny,
		//		candidate_infos[i].cluster.nz,
		//		candidate_infos[i].cluster.geometry)) {
		//		// 計算失敗時の処理
		//		candidate_infos[i].cluster.geometry = ClusterGeometry();
		//	}
		//}

		//// 各クラスタのトリム後AABB密度を計算
		//std::vector<bool> density_ok(N, false);
		//for (int i = 0; i < N; ++i) {
		//	const StepCluster& C = candidate_infos[i].cluster;

		//	std::vector<int> idx_trim;
		//	float w = 0.0f, h = 0.0f, area = 0.0f;
		//	if (!make_trimmed_idx(C, idx_trim)) {
		//		// トリムで有効点が確保できない → 密度NG
		//		candidate_infos[i].is_step = false;
		//		acceptance_reason[i] = "rejected_low_density_trim_failed";
		//		continue;
		//	}
		//	if (!aabb_xy(idx_trim, w, h, area)) {
		//		candidate_infos[i].is_step = false;
		//		acceptance_reason[i] = "rejected_low_density_invalid_aabb";
		//		continue;
		//	}
		//	const float density = (float)idx_trim.size() / area;

		//	// 幾何情報に保存（後続ログ用）
		//	candidate_infos[i].cluster.geometry.rect_area = area;
		//	candidate_infos[i].cluster.geometry.point_density = density;

		//	if (density > DENSITY_MIN) {
		//		density_ok[i] = true;
		//	}
		//	else {
		//		candidate_infos[i].is_step = false;
		//		std::ostringstream oss;
		//		oss << "rejected_low_density_density" << std::fixed << std::setprecision(2)
		//			<< density << "_lt_" << DENSITY_MIN;
		//		acceptance_reason[i] = oss.str();
		//	}
		//}


		//// -----------------------------------------------------------
		//// 【前計算】全ペアで位置ゲートとIoUゲートの通過判定を記録
		//// -----------------------------------------------------------
		//std::vector<std::vector<bool>> pos_ok(N, std::vector<bool>(N, false));
		//std::vector<std::vector<bool>> iou_ok(N, std::vector<bool>(N, false));

		//for (int i = 0; i < N; ++i) {
		//	for (int j = 0; j < N; ++j) {
		//		if (i == j) continue;

		//		// 密度ゲート（どちらかが低密度なら以降の判定を行わない）
		//		if (!density_ok[i] || !density_ok[j]) {
		//			continue;
		//		}

		//		const StepCluster& ci = candidate_infos[i].cluster;
		//		const StepCluster& cj = candidate_infos[j].cluster;

		//		// ① 位置ゲート（強化版: 両者の|cx|が上限以下 + Δcy/Δcz が範囲内）
		//		bool cx_i_ok = (std::fabs(ci.cx) <= CX_MAX);
		//		bool cx_j_ok = (std::fabs(cj.cx) <= CX_MAX);
		//		bool dcy_ok = (std::fabs(ci.cy - cj.cy) >= DCY_MIN && std::fabs(ci.cy - cj.cy) <= DCY_MAX);
		//		bool dcz_ok = (std::fabs(ci.cz - cj.cz) >= DCZ_MIN && std::fabs(ci.cz - cj.cz) <= DCZ_MAX);

		//		pos_ok[i][j] = (cx_i_ok && cx_j_ok && dcy_ok && dcz_ok);
		//		//pos_ok[i][j] = (/*cx_i_ok && cx_j_ok &&*/ dcy_ok && dcz_ok);

		//		// ② 矩形IoU見証（5%トリム＋矩形中心合わせAABB）
		//		float iou = compute_rect_iou_trimmed_center_aligned(ci, cj);
		//		iou_ok[i][j] = (iou >= IOU_MIN);
		//	}
		//}

		//// -----------------------------------------------------------
		//// 【見証型伝播】位置見証とIoU見証を別々に探索
		//// -----------------------------------------------------------
		//std::vector<bool> used(N, false);              // 成分に取り込み済みフラグ
		//std::vector<std::vector<int>> components;      // 抽出された連結成分群

		//// 密度ゲート（どちらかが低密度なら以降の判定を行わない）
		//for (int i = 0; i < N; ++i) {
		//	if (!density_ok[i]) used[i] = true;
		//}

		//while (true) {
		//	// 1) 未使用クラスタから最小IDをseedに選ぶ
		//	int seed = -1;
		//	for (int i = 0; i < N; ++i) {
		//		if (!used[i] && density_ok[i]) { // 密度OKのみseed候補
		//			seed = i;
		//			break;
		//		}
		//	}
		//	if (seed == -1) break;  // 全クラスタ処理完了

		//	// 2) 成分Sを初期化
		//	std::vector<int> S;
		//	S.push_back(seed);
		//	used[seed] = true;

		//	// 3) 伝播ループ（追加が止まるまで繰り返し）
		//	bool added = true;
		//	while (added) {
		//		added = false;

		//		for (int t = 0; t < N; ++t) {
		//			if (used[t]) continue;  // 既に成分に含まれている
		//			if (!density_ok[t]) { used[t] = true; continue; } // 密度NGは追加しない

		//			// ① 位置見証: S内の誰かと位置ゲートを通過するか？
		//			bool has_pos = false;
		//			for (size_t s_idx = 0; s_idx < S.size(); ++s_idx) {
		//				int s = S[s_idx];
		//				if (pos_ok[t][s] || pos_ok[s][t]) {
		//					has_pos = true;
		//					break;
		//				}
		//			}

		//			// ② IoU見証: S内の誰かとIoUゲートを通過するか？
		//			bool has_iou = false;
		//			for (size_t s_idx = 0; s_idx < S.size(); ++s_idx) {
		//				int s = S[s_idx];
		//				if (iou_ok[t][s] || iou_ok[s][t]) {
		//					has_iou = true;
		//					break;
		//				}
		//			}

		//			// ③ 両方OKならSに追加
		//			if (has_pos && has_iou) {
		//				S.push_back(t);
		//				used[t] = true;
		//				added = true;
		//			}
		//		}
		//	}

		//	// 4) 成分Sを確定して記録
		//	components.push_back(S);
		//}

		//// -----------------------------------------------------------
		//// 【段差認定】最大サイズの連結成分を「階段の踏面」として確定
		//// -----------------------------------------------------------
		//std::vector<int> component_id(N, -1);  // 各クラスタの成分ID
		//std::vector<int> component_size(N, 0); // 各クラスタが属する成分のサイズ

		//// 最大連結成分サイズ（足元クラスタ救済の条件にも使う）
		//size_t max_size = 0;
		//int max_idx = -1;

		//if (!components.empty()) {
		//	// 各クラスタに成分IDとサイズを記録
		//	for (size_t g = 0; g < components.size(); ++g) {
		//		for (size_t k = 0; k < components[g].size(); ++k) {
		//			int ci = components[g][k];
		//			component_id[ci] = (int)g;
		//			component_size[ci] = (int)components[g].size();
		//		}
		//	}

		//	// 最大成分を探す
		//	max_size = 0;
		//	max_idx = -1;
		//	for (size_t g = 0; g < components.size(); ++g) {
		//		if (components[g].size() > max_size) {
		//			max_size = components[g].size();
		//			max_idx = (int)g;
		//		}
		//	}

		//	// 最大成分サイズが 2 以上のときだけ IoU 由来で段差として確定
		//	if (max_size >= 2 && max_idx >= 0) {
		//		// 最大成分に属するクラスタを段差として確定
		//		const std::vector<int>& step_component = components[max_idx];
		//		for (size_t k = 0; k < step_component.size(); ++k) {
		//			int ci = step_component[k];
		//			candidate_infos[ci].is_step = true;
		//			acceptance_reason[ci] = "accepted_by_iou_witness_max_component";
		//		}

		//		// 他の成分はリジェクト（理由を詳細化）
		//		for (size_t g = 0; g < components.size(); ++g) {
		//			if ((int)g == max_idx) continue;
		//			for (size_t k = 0; k < components[g].size(); ++k) {
		//				int ci = components[g][k];
		//				candidate_infos[ci].is_step = false;

		//				std::ostringstream oss;
		//				oss << "rejected_not_max_component"
		//					<< "_comp" << g
		//					<< "_size" << components[g].size()
		//					<< "_vs_max" << max_size;
		//				acceptance_reason[ci] = oss.str();
		//			}
		//		}
		//	}
		//	// max_size < 2 のときは、ここでは IoU 由来では誰も採用しない
		//}

		//// -----------------------------------------------------------
		//// 【足元クラスタ単独認定】
		////  2個以上の要素連結が存在しない場合に、足元にあるクラスタを
		////  路面からの高さ差で踏面として追加認定する
		//// -----------------------------------------------------------
		//if (max_size < 2) {
		//	// 足元判定用のしきい値（必要に応じて調整）
		//	const float FOOT_Y_ABS_MAX = 0.75f;  // |cy| がこの値以下なら足元
		//	const float FOOT_Z_ABS_MAX = 0.15f;  // |cz| がこの値以下なら足元

		//	// 路面からの高さ差 [m]
		//	const float GAP_MIN = 0.10f; // 10cm
		//	const float GAP_MAX = 0.30f; // 30cm

		//	int best_idx = -1;
		//	float best_cx_abs = std::numeric_limits<float>::max();

		//	for (int i = 0; i < N; ++i) {
		//		// 密度NGは対象外
		//		if (!density_ok[i]) {
		//			continue;
		//		}

		//		CandidateInfo& info = candidate_infos[i];
		//		const StepCluster& C = info.cluster;

		//		// ground_gap_med が計算済みのクラスタのみ対象
		//		if (!info.ground_checked) {
		//			continue;
		//		}
		//		if (std::isnan(info.ground_gap_med)) {
		//			continue;
		//		}

		//		// 距離は絶対値で評価（上向き/下向きどちらでも）
		//		float gap = std::fabs(info.ground_gap_med);
		//		if (gap < GAP_MIN || gap > GAP_MAX) {
		//			continue;
		//		}

		//		// 足元判定：y, z が 0 付近
		//		if (std::fabs(C.cy) > FOOT_Y_ABS_MAX) {
		//			continue;
		//		}
		//		if (std::fabs(C.cz) > FOOT_Z_ABS_MAX) {
		//			continue;
		//		}

		//		// 候補が複数あれば |cx| が最も小さい（ロボット足元に一番近い）ものを採用
		//		float cx_abs = std::fabs(C.cx);
		//		if (cx_abs < best_cx_abs) {
		//			best_cx_abs = cx_abs;
		//			best_idx = i;
		//		}
		//	}

		//	if (best_idx >= 0) {
		//		candidate_infos[best_idx].is_step = true;
		//		acceptance_reason[best_idx] = "accepted_by_single_foot_cluster_gap";
		//	}
		//}

		// 追加出力：各段差クラスタに対して「一段上／一段下」の高さ差を計算・保存

		std::vector<float> up_gaps(candidate_infos.size(), std::numeric_limits<float>::quiet_NaN());
		std::vector<float> down_gaps(candidate_infos.size(), std::numeric_limits<float>::quiet_NaN());

		for (size_t ci = 0; ci < candidate_infos.size(); ++ci) {
			if (candidate_infos[ci].is_step) {
				float up_gap = std::numeric_limits<float>::quiet_NaN();
				float down_gap = std::numeric_limits<float>::quiet_NaN();
				find_up_down_gaps(candidate_infos, (int)ci, STEP_PROPAGATE_RADIUS2, up_gap, down_gap);
				up_gaps[ci] = up_gap;
				down_gaps[ci] = down_gap;
			}
		}

		// 踏面判定から漏れた段差クラスタのうち、
		// XY 1m / Z 5cm 以内に路面パッチ重心があるものを road に昇格させる

		// クラスタ単位の昇格フラグ
		std::vector<char> promote_cluster_mask(candidate_infos.size(), 0);

		// 路面パッチ重心の一覧作成
		std::vector<Eigen::Vector3f> ground_centroids;
		ground_centroids.reserve(pca_results.size());
		for (const auto& pr : pca_results) {
			if (!pr.is_ground) {
				continue;
			}
			// pc_mean は Eigen::VectorXf / Vector3f 想定
			if (pr.pc_mean.size() < 2) {
				continue;
			}

			// 元の座標系での PCA 重心
			Eigen::Vector3f gc_orig;
			float mz = 0.0f;
			if (pr.pc_mean.size() >= 3) {
				mz = pr.pc_mean(2);
			}
			gc_orig << pr.pc_mean(0), pr.pc_mean(1), mz;

			// 路面水平化用の回転を適用して、段差クラスタと同じ座標系にそろえる
			Eigen::Vector3f gc = gc_orig;
			if (ground_normal_valid || has_last_valid_R) {
				gc = R_ground_to_horizontal * gc_orig;
			}

			ground_centroids.push_back(gc);
		}


		const float ROAD_XY_RADIUS = 1.0f;   // [m]
		const float ROAD_Z_THRESH = 0.05f;  // [m] 5cm

		for (size_t ci = 0; ci < candidate_infos.size(); ++ci) {
			CandidateInfo& info = candidate_infos[ci];

			// 「踏面判定から漏れた段差クラスタ」のみ対象
			if (info.is_step) {
				continue;
			}

			const StepCluster& cluster = info.cluster;

			bool promote = false;
			for (size_t gi = 0; gi < ground_centroids.size(); ++gi) {
				const Eigen::Vector3f& gc = ground_centroids[gi];

				float dx = cluster.cx - gc.x();
				float dy = cluster.cy - gc.y();
				float xy_dist = std::sqrt(dx * dx + dy * dy);
				if (xy_dist > ROAD_XY_RADIUS) {
					continue;
				}

				float dz = std::fabs(cluster.cz - gc.z());
				if (dz <= ROAD_Z_THRESH) {
					promote = true;
					break;
				}
			}

			if (!promote) {
				continue;
			}

			// 昇格クラスタとしてマーク
			promote_cluster_mask[ci] = 1;

			// このクラスタの全点を road / promotedc に追加し、
			// solid から完全除外できるようインデックスもマーク
			for (size_t pi = 0; pi < cluster.indices.size(); ++pi) {
				int idx = cluster.indices[pi];
				if (idx < 0 || idx >= (int)nonground_cloud->points.size()) {
					continue;
				}
				const pcl::PointXYZI& src = nonground_cloud->points[idx];

				pcl::PointXYZRGB dst;
				dst.x = src.x;
				dst.y = src.y;
				dst.z = src.z;
				dst.r = 0;
				dst.g = 255;
				dst.b = 0;

				road->push_back(dst);       // 路面として追加
				if (promotedc) {
					promotedc->push_back(dst); // デバッグ用に別出力もしておく
				}

				// solid 側からも完全に除外するためマーク
				promote_point_mask[(size_t)idx] = 1;
			}
		}
		
		// 可視化：受理クラスタの点は赤（stepc）、非受理は黄（stepc_rejected）で出力。
		for (size_t ci = 0; ci < candidate_infos.size(); ++ci) {

			const StepCluster& cluster_ref = candidate_infos[ci].cluster;

			// 路面に昇格したクラスタはここでは描画しない（road / promotedc にのみ描画）
			if (!promote_cluster_mask.empty() && promote_cluster_mask[ci]) {
				continue;
			}

			// 元のクラスタ点の色分け（既存）
			for (size_t pi = 0; pi < cluster_ref.indices.size(); ++pi) {
				int idx = cluster_ref.indices[pi];
				const pcl::PointXYZI& p = (*nonground_cloud)[idx];
				tmp_rgb.x = p.x;
				tmp_rgb.y = p.y;
				tmp_rgb.z = p.z;
				if (candidate_infos[ci].is_step) {
					tmp_rgb.r = 255; tmp_rgb.g = 0; tmp_rgb.b = 0;
					stepc->push_back(tmp_rgb);
				}
				else {
					tmp_rgb.r = 0; tmp_rgb.g = 0; tmp_rgb.b = 0;
					stepc_rejected->push_back(tmp_rgb);
				}
			}

			// 踏面クラスタのバウンディングボックスを「線」で追加
			if (candidate_infos[ci].is_step) {

				const ClusterGeometry& geom = cluster_ref.geometry;

				float xmin = geom.bbox_min_x;
				float xmax = geom.bbox_max_x;
				float ymin = geom.bbox_min_y;
				float ymax = geom.bbox_max_y;
				float zmin = geom.bbox_min_z;
				float zmax = geom.bbox_max_z;

				// 何分割するか（大きくすると線が滑らかに見える）
				const int NUM_SEG = 30;

				// 線分を追加する小さな関数
				auto add_line = [&](float x1, float y1, float z1,
					float x2, float y2, float z2) {
					for (int s = 0; s <= NUM_SEG; ++s) {
						float t = static_cast<float>(s) / static_cast<float>(NUM_SEG);
						pcl::PointXYZRGB q;
						q.x = x1 + (x2 - x1) * t;
						q.y = y1 + (y2 - y1) * t;
						q.z = z1 + (z2 - z1) * t;
						// 色はシアン（お好みで変更可）
						q.r = 0;
						q.g = 255;
						q.b = 255;
						step_bbox->push_back(q);
					}
				};

				// 12本のエッジを追加
				// x方向のエッジ（y,z 固定）
				add_line(xmin, ymin, zmin, xmax, ymin, zmin);
				add_line(xmin, ymin, zmax, xmax, ymin, zmax);
				add_line(xmin, ymax, zmin, xmax, ymax, zmin);
				add_line(xmin, ymax, zmax, xmax, ymax, zmax);

				// y方向のエッジ（x,z 固定）
				add_line(xmin, ymin, zmin, xmin, ymax, zmin);
				add_line(xmax, ymin, zmin, xmax, ymax, zmin);
				add_line(xmin, ymin, zmax, xmin, ymax, zmax);
				add_line(xmax, ymin, zmax, xmax, ymax, zmax);

				// z方向のエッジ（x,y 固定）
				add_line(xmin, ymin, zmin, xmin, ymin, zmax);
				add_line(xmax, ymin, zmin, xmax, ymin, zmax);
				add_line(xmin, ymax, zmin, xmin, ymax, zmax);
				add_line(xmax, ymax, zmin, xmax, ymax, zmax);
			}
		}

		// CSV 出力：各クラスタの幾何情報と ground_gap、受理フラグ、理由、上下段差を行単位で保存

		char planes_fn[512];
		std::snprintf(planes_fn, sizeof(planes_fn), "%s/submap_%d_%d_step_planes.csv", submap_output_base.c_str(), start_id, end_id);
		std::ofstream pf(planes_fn, std::ios::out);

		if (pf.is_open()) {
			pf << "cluster_id,num_points,cx,cy,cz,nx,ny,nz,d,ground_gap_median,is_step,accept_reason,up_gap,down_gap,"
				<< "length,width,aspect_ratio,"                    // ← 追加
				<< "rect_area,actual_area,fill_ratio,rectangularity,"  // ← 追加
				<< "bbox_min_x,bbox_max_x,bbox_min_y,bbox_max_y,bbox_min_z,bbox_max_z\n";  // ← 追加
			for (size_t ci = 0; ci < candidate_infos.size(); ++ci) {
				const StepCluster& cluster_ref = candidate_infos[ci].cluster;
				const ClusterGeometry& geom = cluster_ref.geometry;
				int is_step_flag = candidate_infos[ci].is_step ? 1 : 0;
				pf << ci << "," << cluster_ref.indices.size() << ","
					<< cluster_ref.cx << "," << cluster_ref.cy << "," << cluster_ref.cz << ","
					<< cluster_ref.nx << "," << cluster_ref.ny << "," << cluster_ref.nz << ","
					<< cluster_ref.d << "," << candidate_infos[ci].ground_gap_med << "," << is_step_flag << ","
					<< acceptance_reason[ci] << "," << up_gaps[ci] << "," << down_gaps[ci] << ","
					<< geom.length << "," << geom.width << "," << geom.aspect_ratio << ","  // ← 追加
					<< geom.rect_area << "," << geom.actual_area << ","                     // ← 追加
					<< geom.fill_ratio << "," << geom.rectangularity << ","                 // ← 追加
					<< geom.bbox_min_x << "," << geom.bbox_max_x << ","                     // ← 追加
					<< geom.bbox_min_y << "," << geom.bbox_max_y << ","                     // ← 追加
					<< geom.bbox_min_z << "," << geom.bbox_max_z << "\n";
			}
		}

		// JSONL ログ：各クラスタの確定内容をイベントとして記録（デバッグ・検証用）
		for (size_t ci = 0; ci < candidate_infos.size(); ++ci) {
			StepClusterLogHelper logger(start_id, end_id, (int)ci, &candidate_infos[ci], &acceptance_reason[ci], up_gaps[ci], down_gaps[ci]);
			JsonlLogger::instance().logEvent("step_cluster", logger);
		}

		// 各クラスタの点インデックス出力
		dump_step_cluster_members(candidate_infos, submap_start_id_, submap_end_id_, submap_output_base, nonground_cloud);

		// 1. 最終的に「段差(is_step)」として採用された点だけをマークする配列を用意
		std::vector<bool> is_accepted_step_point(total_points, false);

		for (size_t ci = 0; ci < candidate_infos.size(); ++ci) {
			if (candidate_infos[ci].is_step) {
				const StepCluster& cluster = candidate_infos[ci].cluster;
				for (size_t k = 0; k < cluster.indices.size(); ++k) {
					int idx = cluster.indices[k];
					if (idx >= 0 && idx < total_points) {
						is_accepted_step_point[idx] = true;
					}
				}
			}
		}

		// solid (非路面) の抽出
		// 変更前: cluster_point_mask (候補になった全点) を除外していたため、リジェクト点が消えていた
		// 変更後: is_accepted_step_point (受理された段差) と promote (路面昇格) だけを除外する
		for (int i = 0; i < total_points; ++i) {

			// A. 路面に昇格した点は除外（roadへ行くため）
			if (!promote_point_mask.empty() && promote_point_mask[(size_t)i]) {
				continue;
			}

			// B. 段差として受理された点は除外（stepcへ行くため）
			if (is_accepted_step_point[i]) {
				continue;
			}

			// C. それ以外はすべて solid に戻す
			// (候補になったがリジェクトされた点もここに含まれるようになる)
			const pcl::PointXYZI& p = (*nonground_cloud)[i];
			tmp_rgb.x = p.x;
			tmp_rgb.y = p.y;
			tmp_rgb.z = p.z;
			tmp_rgb.r = 0; tmp_rgb.g = 0; tmp_rgb.b = 0;
			solid->push_back(tmp_rgb);
		}

	}


	char pca_fn[512];
	std::snprintf(pca_fn, sizeof(pca_fn),
		"%s/submap_%d_%d_pca_results.csv",
		submap_output_base.c_str(), start_id, end_id);
	GS.computeRadialLaplacianRobustAll();
	GS.exportPCAResults(pca_fn);
	GS.exportRVPFDetectionsCSV(submap_output_base);
	GS.clearRVPFDetections(); // 次サブマップに持ち越さない
	GS.clearPCAResults();

	//// PCA結果の export/clear は段差判定の後に実施（順序整合のためここに移動）
	//if (GS.params_.pca_check) {
	//	char pca_fn[512];
	//	std::snprintf(pca_fn, sizeof(pca_fn),
	//		"%s/submap_%d_%d_pca_results.csv",
	//		submap_output_base.c_str(), start_id, end_id);
	//	GS.computeRadialLaplacianRobustAll();
	//	GS.exportPCAResults(pca_fn);
	//	GS.exportRVPFDetectionsCSV(submap_output_base);
	//	GS.clearRVPFDetections(); // 次サブマップに持ち越さない
	//	GS.clearPCAResults();
	//}

	/*
	段差→路面の再分類
	半径・高さ差の閾値で路面へ再分類するフィルタ
	*/
	//for (int i = 0; i < border.rows(); ++i) {
	//	tmp_rgb.x = border(i, 0); tmp_rgb.y = border(i, 1); tmp_rgb.z = border(i, 2);
	//	tmp_rgb.r = 0; tmp_rgb.g = 0; tmp_rgb.b = 0;
	//	solid->push_back(tmp_rgb);
	//}
	/* 段差→路面の再分類（半径1m以内に5cm以内のZ差の路面点があれば路面へ） */
	//if (!stepc->empty() && !road->empty()) {
	//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr step_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
	//	double radius2 = 0.5;     // 1.0^2
	//	double z_thr = 0.03;      // 5cm
	//
	//	for (size_t si = 0; si < stepc->points.size(); ++si) {
	//		pcl::PointXYZRGB sp = stepc->points[si];
	//		int to_ground = 0;
	//
	//		for (size_t gi = 0; gi < road->points.size(); ++gi) {
	//			pcl::PointXYZRGB gp = road->points[gi];
	//			double dx = (double)sp.x - (double)gp.x;
	//			double dy = (double)sp.y - (double)gp.y;
	//			double dz = (double)sp.z - (double)gp.z;
	//			double d2 = dx * dx + dy * dy;
	//			if (d2 <= radius2 && fabs(dz) <= z_thr) {
	//				to_ground = 1;
	//				break;
	//			}
	//		}
	//
	//		if (to_ground == 1) {
	//			pcl::PointXYZRGB g;
	//			g.x = sp.x; g.y = sp.y; g.z = sp.z;
	//			g.r = 0; g.g = 255; g.b = 0;
	//			road->push_back(g);
	//		}
	//		else {
	//			step_filtered->push_back(sp);
	//		}
	//	}
	//	*stepc = *step_filtered;
	//}
	/********************************************************************
	 *  永井追加アルゴリズムここまで
	 ********************************************************************/


	 //for (int i = 0; i < nonground.rows(); ++i) appendXYZRGB(solid, nonground(i, 0), nonground(i, 1), nonground(i, 2), 0, 0, 0);

	 // 保存

	// reject点群の出力
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr reject_by_trim(new pcl::PointCloud<pcl::PointXYZRGB>);
	for (size_t i = 0; i < global_rejected_indices.size(); ++i) {
		int idx = global_rejected_indices[i];
		if (idx >= 0 && idx < (int)nonground_cloud->size()) {
			const pcl::PointXYZI& p = (*nonground_cloud)[idx];
			tmp_rgb.x = p.x;
			tmp_rgb.y = p.y;
			tmp_rgb.z = p.z;
			tmp_rgb.r = 255; tmp_rgb.g = 165; tmp_rgb.b = 0; // オレンジ色
			reject_by_trim->push_back(tmp_rgb);
		}
	}

	if (!road->empty()) { sprintf(fn, "%s/submap_%d_%d_road.pcd", road_dir.c_str(), start_id, end_id);  pcl::io::savePCDFileBinary(fn, *road); }
	if (!solid->empty()) { sprintf(fn, "%s/submap_%d_%d_solid.pcd", solid_dir.c_str(), start_id, end_id);  pcl::io::savePCDFileBinary(fn, *solid); }
	if (!wallc->empty()) { sprintf(fn, "%s/submap_%d_%d_wall.pcd", wall_dir.c_str(), start_id, end_id);  pcl::io::savePCDFileBinary(fn, *wallc); }
	if (!floatc->empty()) { sprintf(fn, "%s/submap_%d_%d_float.pcd", float_dir.c_str(), start_id, end_id);  pcl::io::savePCDFileBinary(fn, *floatc); }
	if (!stepc->empty()) { sprintf(fn, "%s/submap_%d_%d_step.pcd", step_dir.c_str(), start_id, end_id);  pcl::io::savePCDFileBinary(fn, *stepc); }
	if (!stepc_rejected->empty()) { sprintf(fn, "%s/submap_%d_%d_step_rejected.pcd", step_dir.c_str(), start_id, end_id);  pcl::io::savePCDFileBinary(fn, *stepc_rejected); } // 永井追加
	if (!promotedc->empty()) {
		sprintf(fn, "%s/submap_%d_%d_promoted.pcd", promoted_dir.c_str(), start_id, end_id);
		pcl::io::savePCDFileBinary(fn, *promotedc);
	}


	// 永井追加．段差の回数別ファイル
	//if (!stepc1->empty()) { sprintf(fn, "%s/submap_%d_%d_step_it1.pcd", step_dir.c_str(), start_id, end_id); pcl::io::savePCDFileBinary(fn, *stepc1); }
	//if (!stepc2->empty()) { sprintf(fn, "%s/submap_%d_%d_step_it2.pcd", step_dir.c_str(), start_id, end_id); pcl::io::savePCDFileBinary(fn, *stepc2); }
	//if (!stepc3->empty()) { sprintf(fn, "%s/submap_%d_%d_step_it3.pcd", step_dir.c_str(), start_id, end_id); pcl::io::savePCDFileBinary(fn, *stepc3); }
	//if (!stepc4->empty()) { sprintf(fn, "%s/submap_%d_%d_step_it4.pcd", step_dir.c_str(), start_id, end_id); pcl::io::savePCDFileBinary(fn, *stepc4); }
	//if (!stepc5p->empty()) { sprintf(fn, "%s/submap_%d_%d_step_it5p.pcd", step_dir.c_str(), start_id, end_id); pcl::io::savePCDFileBinary(fn, *stepc5p); }

	// GPF後の色分け地図保存
	{
		pcl::PointCloud<pcl::PointXYZRGB> all_colored;
		//if (!stepc->empty()) all_colored += *stepc;
		//if (!reject_by_trim->empty()) all_colored += *reject_by_trim;
		if (!road->empty())			  all_colored += *road;
		if (!stepc->empty())		  all_colored += *stepc;
		if (!reject_by_trim->empty()) all_colored += *reject_by_trim;
		if (!stepc_rejected->empty()) all_colored += *stepc_rejected; // リジェクト段差も表示
		if (!solid->empty())		  all_colored += *solid;
		if (!wallc->empty())		  all_colored += *wallc;
		if (!floatc->empty())		  all_colored += *floatc;
		//if (!step_bbox->empty())      all_colored += *step_bbox; // バウンディングボックス表示（デバッグ用）

		std::cout << "[DEBUG] all colored czm_boundaryc points: "
			<< (czm_boundaryc ? czm_boundaryc->size() : 0)
			<< std::endl;

		// デバッグモードが有効な場合のみCZMグリッド境界点をall_colored にも追加
		if (GS.params_.enable_czm_boundary && !czm_boundaryc->empty()) {
			std::cout << "[DEBUG] all colored czm_boundaryc points: "
				<< (czm_boundaryc ? czm_boundaryc->size() : 0)
				<< std::endl;
			all_colored += *czm_boundaryc;
		}
		//if (!stepc->empty()) all_colored += *stepc;

		//// センサ位置を示す直方体を追加
		//{
		//	// 直方体の中心とサイズ
		//	const float cx = 0.0f;
		//	const float cy = 0.0f;
		//	const float cz = 1.2f;   // 指定された中心高さ

		//	const float half_x = 0.12f;  // X方向半幅 [m]（少し太く）
		//	const float half_y = 0.12f;  // Y方向半幅 [m]（少し太く）
		//	const float half_z = 0.6f;   // Z方向半高さ [m] → 高さ 1.2m の直方体

		//	const float x_min = cx - half_x;
		//	const float x_max = cx + half_x;
		//	const float y_min = cy - half_y;
		//	const float y_max = cy + half_y;
		//	const float z_min = cz - half_z;
		//	const float z_max = cz + half_z;

		//	const float step = 0.02f;      // エッジを描く点の間隔 [m]（細かく）
		//	const float thick = 0.02f;     // 疑似的な線の太さ [m]

		//	pcl::PointXYZRGB pt;
		//	pt.r = 255;  // センサ用に赤色で描画
		//	pt.g = 0;
		//	pt.b = 0;

		//	int i;
		//	int j;
		//	int k;

		//	// 4本の縦エッジ
		//	for (i = 0; i < 2; i++) {
		//		float x;
		//		if (i == 0) {
		//			x = x_min;
		//		}
		//		else {
		//			x = x_max;
		//		}

		//		for (j = 0; j < 2; j++) {
		//			float y;
		//			if (j == 0) {
		//				y = y_min;
		//			}
		//			else {
		//				y = y_max;
		//			}

		//			float z = z_min;
		//			while (z <= z_max) {
		//				// 線を太く見せるために 3×3 点を配置
		//				int dx;
		//				int dy;
		//				for (dx = -1; dx <= 1; dx++) {
		//					for (dy = -1; dy <= 1; dy++) {
		//						pt.x = x + dx * thick;
		//						pt.y = y + dy * thick;
		//						pt.z = z;
		//						all_colored.push_back(pt);
		//					}
		//				}
		//				z += step;
		//			}
		//		}
		//	}

		//	// 上下面の枠 (x 方向)
		//	for (k = 0; k < 2; k++) {
		//		float z;
		//		if (k == 0) {
		//			z = z_min;
		//		}
		//		else {
		//			z = z_max;
		//		}

		//		for (j = 0; j < 2; j++) {
		//			float y;
		//			if (j == 0) {
		//				y = y_min;
		//			}
		//			else {
		//				y = y_max;
		//			}

		//			float x = x_min;
		//			while (x <= x_max) {
		//				int dx;
		//				int dy;
		//				for (dx = -1; dx <= 1; dx++) {
		//					for (dy = -1; dy <= 1; dy++) {
		//						pt.x = x + dx * thick;
		//						pt.y = y + dy * thick;
		//						pt.z = z;
		//						all_colored.push_back(pt);
		//					}
		//				}
		//				x += step;
		//			}
		//		}
		//	}

		//	// 上下面の枠 (y 方向)
		//	for (k = 0; k < 2; k++) {
		//		float z;
		//		if (k == 0) {
		//			z = z_min;
		//		}
		//		else {
		//			z = z_max;
		//		}

		//		for (i = 0; i < 2; i++) {
		//			float x;
		//			if (i == 0) {
		//				x = x_min;
		//			}
		//			else {
		//				x = x_max;
		//			}

		//			float y = y_min;
		//			while (y <= y_max) {
		//				int dx;
		//				int dy;
		//				for (dx = -1; dx <= 1; dx++) {
		//					for (dy = -1; dy <= 1; dy++) {
		//						pt.x = x + dx * thick;
		//						pt.y = y + dy * thick;
		//						pt.z = z;
		//						all_colored.push_back(pt);
		//					}
		//				}
		//				y += step;
		//			}
		//		}
		//	}
		//}


		// 色分けした段差を追加
		//if (!stepc1->empty()) all_colored += *stepc1;
		//if (!stepc2->empty()) all_colored += *stepc2;
		//if (!stepc3->empty()) all_colored += *stepc3;
		//if (!stepc4->empty()) all_colored += *stepc4;
		//if (!stepc5p->empty())all_colored += *stepc5p;

		char colored_filename[512];
		std::snprintf(colored_filename, sizeof(colored_filename), "%s/submap_%d_%d_all_colored.pcd", all_colored_dir.c_str(), start_id, end_id);

		if (!all_colored.empty()) {
			std::snprintf(fn, sizeof(fn), "%s/submap_%d_%d_all_colored.pcd", all_colored_dir.c_str(), start_id, end_id);
			pcl::io::savePCDFileBinary(fn, all_colored);
		}
		// 2. 局所地図作成用にメタデータをリストに追加
		// 実際に使用した回転行列と法線を確定させる
		Eigen::Matrix3f current_R_level = Eigen::Matrix3f::Identity();
		Eigen::Vector3f current_normal(0, 0, 1); // デフォルト

		if (ground_normal_valid || has_last_valid_R) {
			current_R_level = R_ground_to_horizontal;

			// R_ground_to_horizontal は 法線をZ軸(0,0,1)に向ける回転なので、
			// 逆算または保存しておいた値を使う必要があります。
			// ここではR_ground_to_horizontalを作ったときの法線ベクトルを復元します。
			// (R * n = z  =>  n = R_inv * z)
			current_normal = current_R_level.transpose() * Eigen::Vector3f(0, 0, 1);
		}

		SubmapMetadata meta;
		meta.pcd_path = std::string(colored_filename);
		meta.T_ws0 = T_ws0_;               // NDT姿勢 (Sensor -> World)
		meta.R_level = current_R_level;    // 水平化回転 (Sensor -> Leveled)
		meta.ground_normal = current_normal;
		meta.start_id = start_id;
		meta.end_id = end_id;

		submap_batch_list_.push_back(meta);

		// 3. 指定個数（100個）溜まったら統合処理を実行
		if (submap_batch_list_.size() >= batch_size_) {
			process_submap_batch();
		}
	}

	// ムービーへのフレーム追加
	//submap_movie::AppendFrameFromPCD(fn, start_id, end_id);
	{
		// 例: サブマップPCDの保存先と開始/終了スキャンIDが分かっている前提
		//     （自分の実装で使っている実際の変数名に合わせてください）
		std::string submap_pcd_path = fn; // 例: "Output_Submaps/submap_0123_0132.pcd"
		int start_id = submap_start_id_;                  // 例: 0123
		int end_id = submap_end_id_;                    // 例: 0132

		AppendFrameFromPCD(fn, start_id, end_id);
	}

	// 全体カテゴリ地図へ累積
	if (!road->empty())  road_map_accum += *road;
	if (!solid->empty()) solid_map_accum += *solid;
	if (!wallc->empty()) wall_map_accum += *wallc;
	if (!floatc->empty())float_map_accum += *floatc;

	// -----------------------------------------------------ログ出力用ブロック------------------------------------------------------

	// 保存類（PCD等）を出し終えた直後に JSON Lines を1本出す

	//const int start_id = submap_start_id_;
	//const int end_id = submap_end_id_;
	// 点数サマリ（すでに作った road/solid/wall/float/stepc から）

	const size_t n_ground = road->size();
	const size_t n_nonground = solid->size();
	const size_t n_wall = wallc->size();
	const size_t n_float = floatc->size();
	const size_t n_step = stepc->size();

	// 既に出力した CSV/PCD の想定パス

	//char pca_fn[512];
	std::snprintf(pca_fn, sizeof(pca_fn), "%s/submap_%d_%d_pca_results.csv", submap_output_base.c_str(), start_id, end_id);
	char step_planes_fn[512];
	std::snprintf(step_planes_fn, sizeof(step_planes_fn), "%s/submap_%d_%d_step_planes.csv", submap_output_base.c_str(), start_id, end_id);
	char colored_fn[512];
	std::snprintf(colored_fn, sizeof(colored_fn), "%s/submap_%d_%d_all_colored.pcd", submap_output_base.c_str(), start_id, end_id);

	JsonlLogger::instance().logEvent("submap_finalize", [&](JsonWriter& jw) {
		jw.key("submap").beginObject();
		jw.key("start_id").integer(start_id);
		jw.key("end_id").integer(end_id);
		jw.key("window").integer(submap_window_);
		jw.key("stride").integer(submap_stride_);
		jw.key("points").integer((long long)submap_active_.size());
		jw.endObject();

		// パラメータのスナップショット（主要のみ）

		jw.key("params").beginObject();
		jw.key("th_seeds").number(GS.params_.th_seeds);
		jw.key("th_dist").number(GS.params_.th_dist);
		jw.key("th_seeds_v").number(GS.params_.th_seeds_v);
		jw.key("th_dist_v").number(GS.params_.th_dist_v);
		jw.key("uprightness_thr").number(GS.params_.uprightness_thr);
		jw.key("uprightness_thr2").number(GS.params_.uprightness_thr2);
		jw.key("max_range").number(GS.params_.max_range);
		jw.key("min_range").number(GS.params_.min_range);
		jw.key("step_seed_radius").number(GS.params_.step_seed_radius);
		jw.key("step_plane_dist_thresh").number(GS.params_.step_plane_dist_thresh);
		jw.key("step_nz_thresh").number(GS.params_.step_nz_thresh);
		jw.key("step_planarity_thresh").number(GS.params_.step_planarity_thresh);
		jw.key("step_merge_angle_deg").number(GS.params_.step_merge_angle_deg);
		jw.key("step_merge_offset_thresh").number(GS.params_.step_merge_offset_thresh);
		jw.key("step_merge_xy_dist").number(GS.params_.step_merge_xy_dist);
		jw.key("step_local_xy_radius").number(GS.params_.step_local_xy_radius);
		jw.key("step_dz_min").number(GS.params_.step_dz_min);
		jw.key("step_dz_max").number(GS.params_.step_dz_max);
		jw.key("ransac_dist_thresh").number(GS.params_.ransac_dist_thresh);
		jw.key("ransac_max_iterations").integer(GS.params_.ransac_max_iterations);
		jw.key("ransac_uprightness_thr").number(GS.params_.ransac_uprightness_thr);
		jw.key("ransac_trim_ratio").number(GS.params_.ransac_trim_ratio);
		jw.key("ransac_weight_by_inliers").boolean(GS.params_.ransac_weight_by_inliers);
		jw.endObject();

		jw.key("counts").beginObject();
		jw.key("ground").integer((long long)n_ground);
		jw.key("nonground").integer((long long)n_nonground);
		jw.key("wall").integer((long long)n_wall);
		jw.key("float").integer((long long)n_float);
		jw.key("step").integer((long long)n_step);
		jw.endObject();

		jw.key("outputs").beginObject();
		jw.key("pca_csv").string(std::string(pca_fn));
		jw.key("step_planes_csv").string(std::string(step_planes_fn));
		jw.key("colored_pcd").string(std::string(colored_fn));
		jw.endObject();

		//jw.key("geometry").beginObject();
		//const ClusterGeometry& g = info->cluster.geometry;

		//// 踏面形状特徴量
		//jw.key("shape_features").beginObject();
		//jw.key("length").number(g.length, 6);
		//jw.key("width").number(g.width, 6);
		//jw.key("aspect_ratio").number(g.aspect_ratio, 6);
		//jw.key("fill_ratio").number(g.fill_ratio, 6);
		//jw.key("rectangularity").number(g.rectangularity, 6);
		//jw.endObject();

		//jw.key("bounding_box").beginObject();
		//jw.key("min_x").number(g.bbox_min_x, 6);
		//jw.key("max_x").number(g.bbox_max_x, 6);
		//jw.key("min_y").number(g.bbox_min_y, 6);
		//jw.key("max_y").number(g.bbox_max_y, 6);
		//jw.key("min_z").number(g.bbox_min_z, 6);
		//jw.key("max_z").number(g.bbox_max_z, 6);
		//jw.endObject();

		//jw.key("rect_area").number(g.rect_area, 6);
		//jw.key("actual_area").number(g.actual_area, 6);
		//jw.key("point_density").number(g.point_density, 6);

		//jw.endObject(); // geometry
	});

	// -----------------------------------------------------ログ出力用ブロック------------------------------------------------------

	// リセット
	submap_active_.clear();
	submap_count_ = 0;
	submap_start_id_ = -1;
	submap_end_id_ = -1;
	has_T_ws0_ = false; //永井追加．最初のセンサ座標用
}

// 毎スキャン呼び出し：NDT姿勢で世界座標にして加算
// 変更: 毎スキャン蓄積。per-scan ベクタに追加しつつ submap_active_ も更新
void Save::submap_add_scan(int scan_id,
	const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_robot,
	const Eigen::Matrix4f& T_wr) {
	if (!cloud_robot || cloud_robot->empty()) return;

	// 高さ制限
	// 高さ制限＋近距離(<=0.5m)除外フィルタ
	pcl::PointCloud<pcl::PointXYZI> filtered;
	filtered.clear();
	filtered.reserve(cloud_robot->size());
	const float MIN_RANGE2 = 0.25f * 0.25f; // 追加: 最小距離(0.5m)の二乗
	for (size_t i = 0; i < cloud_robot->points.size(); ++i) {
		const auto& pt = cloud_robot->points[i];

		// 追加（最小変更）: ローカル座標系で x と y が 0.5 以下の点は使用しない
		if (fabsf(pt.x) <= 0.5f && fabsf(pt.y) <= 0.3f) continue;

		float r2 = pt.x * pt.x + pt.y * pt.y + pt.z * pt.z; // 追加: センサ原点からの二乗距離
		if (pt.z <= (float)MAP_HEIGHT_MAX && r2 > MIN_RANGE2) {
			filtered.push_back(pt);
		}
	}


	// 世界座標へ変換（1スキャン分）
	pcl::PointCloud<pcl::PointXYZI> world;
	pcl::transformPointCloud(filtered, world, T_wr);

	// ベクタへ蓄積（オーバーラップ再構築に使う）
	submap_scans_.push_back(world);
	submap_scan_ids_.push_back(scan_id);
	submap_T_wr_.push_back(T_wr);

	// 連結ビューも更新
	submap_active_ += world;

	// 最初のスキャン
	if (submap_count_ == 0) {
		submap_start_id_ = scan_id;
		if (has_T_vs_) {
			T_ws0_ = T_wr * T_vs_;
			has_T_ws0_ = true;
		}
		else {
			has_T_ws0_ = false;
		}
	}
	submap_end_id_ = scan_id;
	++submap_count_;

	std::cout << "add scan OK\n";
}

// 変更: 窓幅に達したら finalize → stride 分先頭を捨てて重なり分を温存して再構築
void Save::submap_finalize_if_ready(
	patchwork::Ground_Segmentation& GS,
	pcl::PointCloud<pcl::PointXYZRGB>& road_map_accum,
	pcl::PointCloud<pcl::PointXYZRGB>& solid_map_accum,
	pcl::PointCloud<pcl::PointXYZRGB>& wall_map_accum,
	pcl::PointCloud<pcl::PointXYZRGB>& float_map_accum
) {
	if (submap_count_ >= submap_window_) {
		// まず現在のウィンドウを確定・出力
		finalize_active_submap_(GS, road_map_accum, solid_map_accum, wall_map_accum, float_map_accum);

		// finalize_active_submap_ は submap_active_ 等をリセットするが、
		// per-scan ベクタは保持されているため、ここで sliding する
		int s = std::max(1, std::min(submap_stride_, submap_window_)); // 安全にクランプ
		if ((int)submap_scans_.size() >= s) {
			// 先頭から stride 分削除
			submap_scans_.erase(submap_scans_.begin(), submap_scans_.begin() + s);
			submap_scan_ids_.erase(submap_scan_ids_.begin(), submap_scan_ids_.begin() + s);
			submap_T_wr_.erase(submap_T_wr_.begin(), submap_T_wr_.begin() + s);
		}
		else {
			submap_scans_.clear();
			submap_scan_ids_.clear();
			submap_T_wr_.clear();
		}

		// 残りで submap_active_ を再構築
		submap_active_.clear();
		for (const auto& c : submap_scans_) {
			submap_active_ += c;
		}

		submap_count_ = (int)submap_scans_.size();
		if (submap_count_ > 0) {
			submap_start_id_ = submap_scan_ids_.front();
			submap_end_id_ = submap_scan_ids_.back();
			if (has_T_vs_) {
				T_ws0_ = submap_T_wr_.front() * T_vs_;
				has_T_ws0_ = true;
			}
			else {
				has_T_ws0_ = false;
			}
		}
		else {
			submap_start_id_ = -1;
			submap_end_id_ = -1;
			has_T_ws0_ = false;
		}
	}
}

// 強制確定（アプリ終了時など）
void Save::submap_force_finalize(
	patchwork::Ground_Segmentation& GS,
	pcl::PointCloud<pcl::PointXYZRGB>& road_map_accum,
	pcl::PointCloud<pcl::PointXYZRGB>& solid_map_accum,
	pcl::PointCloud<pcl::PointXYZRGB>& wall_map_accum,
	pcl::PointCloud<pcl::PointXYZRGB>& float_map_accum
) {
	if (submap_count_ > 0) {
		finalize_active_submap_(GS, road_map_accum, solid_map_accum, wall_map_accum, float_map_accum);
	}
}