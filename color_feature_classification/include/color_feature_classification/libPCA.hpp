#ifndef MY_PCA_HPP
#define MY_PCA_HPP

#include <vector>
#include <Eigen3/Eigenvalues>

/**********************/
/* 主成分分析をするクラス */
/**********************/

class PCA{
public:
  PCA( bool _mean_flg = true ); // 自己相関行列からサンプル平均値をひかない場合はfalseに
  ~PCA(){}

  //* 特徴ベクトルを読み込みながら自己相関行列を計算していく
  void addData( std::vector<float> &feature );

  //* 主成分分析を行う
  void solve();
    
  //* 主成分軸のマトリックスの取得
  const Eigen3::MatrixXf &Axis() const { return axis; }

  //* 固有値を並べたベクトルの取得
  const Eigen3::VectorXf &Variance() const { return variance; }

  //* データの平均ベクトルの取得
  const Eigen3::VectorXf &Mean() const;
    
  //* PCAのデータをファイルから読み込み
  void read( const char *filename, bool ascii = false );

  //* PCAのデータをファイルに出力
  void write( const char *filename, bool ascii = false );
    
private:
  int dim;               // 特徴次元
  bool mean_flg;         // 自己相関行列から平均値をひくか否かのフラグ
  long long nsample;     // サンプル数
  Eigen3::VectorXf mean;     // 特徴量の平均
  Eigen3::MatrixXf correlation;    // 自己相関行列
  Eigen3::MatrixXf axis;           // 主成分軸
  Eigen3::VectorXf variance; // 固有値を並べたベクトル

//   //* 固有値の大きい順に固有値と固有ベクトルを並び替える
//   void sortVecAndVal( Eigen3::MatrixXf &vecs, Eigen3::VectorXf &vals );  
};

#endif
