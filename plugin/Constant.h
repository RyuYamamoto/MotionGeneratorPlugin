/*!
 * @file Constant.h
 * @brief グローバルな変数リスト
 * 全体に共通する定数はこのファイルに記述する
 *
 * 以下に加えることでロボットを切り替える
 * using namespace CitAccelite
 * using namespace CitDynamo (未実装)
 * using namespace CitXega (未実装)
 *
 * @date 2013.12.31
 * @author Yasuo Hayashibara
 */

#pragma once

#define CIT_DYNAMO

namespace Const {

// システム
const int ROBOT_TYPE_HR42	= 42;			//! Dynamo
const int ROBOT_TYPE_HR46	= 46;			//! Accelite

// 制御
const float ST				= 0.020f;		//! サンプリングタイム (s)

const int OFF				= 0;
const int ON				= 1;

const int NEUTRAL			= 0;
const int FOWARD			= 1;
const int BACKWARD			= 2;
const int RIGHT 			= 3;
const int LEFT				= 4;

const float GRAVITY			= 9.8f;			//! 重力加速度(m/s^2)

// Acceliteの固有の定数
#ifdef CIT_DYNAMO

/*
 * @class Acceliteの定数を記述する
 */
const int SERVO_NUM = 18;	//! サーボの個数 (Accelite)
enum {								//! サーボの番号
	FOOT_ROLL_L 	= 	0,			//! 左足首のロール軸
	KNEE_L1 		= 	1,			//! 左足膝下ピッチ軸（平行リンク）
	KNEE_L2 		= 	2,			//! 左足膝上ピッチ軸（平行リンク）
	LEG_PITCH_L 	= 	3,			//! 左股のピッチ軸（シリアルリンク，股のピッチ軸と合わせて足首の向きを変更）
	LEG_ROLL_L	 	= 	4,			//! 左股のロール軸
	LEG_YAW_L 		= 	5,			//! 左股のヨー軸
	FOOT_ROLL_R 	= 	6,			//! 右足首のロール軸
	KNEE_R1		 	= 	7,			//! 右足膝下ピッチ軸（平行リンク）
	KNEE_R2 		= 	8,			//! 右足膝上ピッチ軸（平行リンク）
	LEG_PITCH_R 	= 	9,			//! 右股のピッチ軸（シリアルリンク，股のピッチ軸と合わせて足首の向きを変更）
	LEG_ROLL_R	 	= 	10,			//! 右股のロール軸
	LEG_YAW_R 		= 	11,			//! 右股のヨー軸
	ARM_PITCH_L		= 	14,			//! 左腕のピッチ軸
	ARM_ROLL_L		= 	15,			//! 左腕のロール軸
	ARM_PITCH_R		= 	18,			//! 右腕のピッチ軸
	ARM_ROLL_R		= 	19,			//! 右腕のロール軸
	HEAD_YAW		= 	22,			//! 首のヨー軸
	HEAD_PITCH		= 	23,			//! 首のピッチ軸（Acceliteは無い）
	SERVO_MAX_ID
};

const int SERVO_NO[SERVO_NUM] =			//! サーボの番号列
{
	FOOT_ROLL_L ,
	KNEE_L1     ,
	KNEE_L2     ,
	LEG_PITCH_L ,
	LEG_ROLL_L  ,
	LEG_YAW_L   ,
	FOOT_ROLL_R ,
	KNEE_R1     ,
	KNEE_R2     ,
	LEG_PITCH_R ,
	LEG_ROLL_R  ,
	LEG_YAW_R   ,
	ARM_PITCH_L ,
	ARM_ROLL_L  ,
	ARM_PITCH_R ,
	ARM_ROLL_R  ,
	HEAD_YAW    ,
	HEAD_PITCH  ,
};

// リンク
enum {			// リンクの番号
	CAM	,		//! カメラのリンク（光軸を求めるため）
	HT	,		//! 頭のチルト軸からカメラ (Acceliteは固定)
	HP	,		//! 頭のパン軸からチルト軸
	SH	,		//! 肩中心から頭のパン軸
	CS	,		//! 腰から肩中心
	CC	,		//! 腰（基点）
	RY	,		//! 右股ヨー軸から右股ロール軸
	RR1	,		//! 右股ロール軸から右股ピッチ軸
	RP1	,		//! 右股ピッチ軸から右膝上ピッチ軸
	RP2	,		//! 右膝上ピッチ軸から右膝下ピッチ軸
	RP3	,		//1 右膝下ピッチ軸から右足首ピッチ軸
	RP4	,		//! 右足首ピッチ軸から右足首ロール軸
	RR2	,		//! 右足首ロール軸から右足先
	RF  ,		//! 左足先
	LY	,		//! 左股ヨー軸から左股ロール軸
	LR1	,		//! 左股ロール軸から左股ピッチ軸
	LP1	,		//! 左股ピッチ軸から左膝上ピッチ軸
	LP2	,		//! 左膝上ピッチ軸から左膝下ピッチ軸
	LP3	,		//! 左膝下ピッチ軸から左足首ピッチ軸
	LP4	,		//! 左足首ピッチ軸から左足首ロール軸
	LR2	,		//! 左足首ロール軸から左足先
	LF  ,       //! 左足先
	RSP ,		//! 右肩ピッチ軸からロール軸
	RSR	,		//! 右肩ロール軸から右腕先端
	RH  ,       //! 右腕先端
	LSP ,		//! 右肩ピッチ軸からロール軸
	LSR	,		//! 右肩ロール軸から右腕先端
	LH  ,       //! 左腕先端
	LINK_NUM	//! リンクの数	
};

const float HIP_WIDTH	= 0.088f;	//! 股の幅 (m)
const float L01			= 0.032f;	//! 足首から足先までの長さ (m)
const float L1			= 0.098f;	//! 足首から膝までの長さ (m)
const float L12			= 0.023f;	//! 膝間の距離 (m)
const float L2			= 0.098f;	//! 膝から股までの長さ (m)
const float L23			= 0.0f;		//! 膝から股までの長さ (m)
const float L3			= 0.043f;	//! 腰までの長さ (m)

const float Z3_LIMIT_H  = L01 + L1 + L12 + L2 + L23 + L3 ;
									//! 腰の高さの最大値 (m)
const float Z3_LIMIT_L	= 0.120f;	//! 腰の高さの最小値 (m)

// リンク
struct link_para_T {				//! リンクパラメータの構造体
	float p[3];
	float a[3];
};

const struct link_para_T LINK_PARA[LINK_NUM] = {
//	{{  前後,   左右,   上下},{前後軸,左右軸,上下軸}} (mm)
//	腰 -> 頭
	{{300.0f,   0.0f,   0.0f},{  0.0f,  0.0f,  0.0f}},	// CAM : カメラのリンク（光軸を求めるため）
	{{  0.0f,   0.0f,   0.0f},{  0.0f,  0.0f,  0.0f}},	// HT  : 頭のチルト軸からカメラ (Acceliteは固定)
	{{ 49.4f,   0.0f,  41.6f},{  0.0f,  1.0f,  0.0f}},	// HP  : 頭のパン軸からチルト軸
	{{-10.0f,   0.0f,  29.7f},{  0.0f,  0.0f,  1.0f}},	// SH  : 肩中心から頭のパン軸
	{{  0.0f,   0.0f, 126.5f},{  0.0f,  0.0f,  0.0f}},	// CS  : 腰から肩中心
	{{  0.0f,   0.0f,   0.0f},{  0.0f,  0.0f,  0.0f}},	// CC  : 腰（基点）
//	腰 -> 右足先
	{{  0.0f, -44.0f,   0.0f},{  0.0f,  0.0f,  1.0f}},	// RY  : 右股ヨー軸から右股ロール軸
	{{  0.0f,   0.0f, -42.8f},{  1.0f,  0.0f,  0.0f}},	// RR1 : 右股ロール軸から右股ピッチ軸
	{{ 15.0f,   0.0f,   0.0f},{  0.0f,  1.0f,  0.0f}},	// RP1 : 右股ピッチ軸から右膝上ピッチ軸
	{{  0.0f,   0.0f,-105.0f},{  0.0f,  1.0f,  0.0f}},	// RP2 : 右膝上ピッチ軸から右膝下ピッチ軸
	{{  0.0f,   0.0f, -41.0f},{  0.0f,  1.0f,  0.0f}},	// RP3 : 右膝下ピッチ軸から右足首ピッチ軸
	{{  0.0f,   0.0f,-105.0f},{  0.0f,  1.0f,  0.0f}},	// RP4 : 右足首ピッチ軸から右足首ロール軸
	{{  0.0f,   0.0f,   0.0f},{  1.0f,  0.0f,  0.0f}},	// RR2 : 右足首ロール軸から右足先
	{{  0.0f,   0.0f, -43.5f},{  0.0f,  0.0f,  0.0f}},	// RF  : 右足先
//	腰 -> 左足先
	{{  0.0f,  44.0f,   0.0f},{  0.0f,  0.0f,  1.0f}},	// LY  : 左股ヨー軸から左股ロール軸
	{{  0.0f,   0.0f, -42.8f},{  1.0f,  0.0f,  0.0f}},	// LR1 : 左股ロール軸から左股ピッチ軸
	{{ 15.0f,   0.0f,   0.0f},{  0.0f,  1.0f,  0.0f}},	// LP1 : 左股ピッチ軸から左膝上ピッチ軸
	{{  0.0f,   0.0f,-105.0f},{  0.0f,  1.0f,  0.0f}},	// LP2 : 左膝上ピッチ軸から左膝下ピッチ軸
	{{  0.0f,   0.0f, -41.0f},{  0.0f,  1.0f,  0.0f}},	// LP3 : 左膝下ピッチ軸から左足首ピッチ軸
	{{  0.0f,   0.0f,-105.0f},{  0.0f,  1.0f,  0.0f}},	// LP4 : 左足首ピッチ軸から左足首ロール軸
	{{  0.0f,   0.0f,   0.0f},{  1.0f,  0.0f,  0.0f}},	// LR2 : 左足首ロール軸から左足先
	{{  0.0f,   0.0f, -43.5f},{  0.0f,  0.0f,  0.0f}},	// LF  : 左足先
//	肩中心 -> 右手先
	{{  0.0f, -61.0f,   0.0f},{  0.0f,  1.0f,  0.0f}},	// RSP : 右肩ピッチ軸からロール軸
	{{  0.0f, -17.9f, -41.8f},{  1.0f,  0.0f,  0.0f}},	// RSR : 右肩ロール軸から右腕先端
	{{  0.0f, -30.0f,-247.6f},{  0.0f,  0.0f,  0.0f}},	// RH  : 右腕先端
//	肩中心 -> 左手先
	{{  0.0f,  61.0f,   0.0f},{  0.0f,  1.0f,  0.0f}},	// LSP : 左肩ピッチ軸からロール軸
	{{  0.0f,  17.9f, -41.8f},{  1.0f,  0.0f,  0.0f}},	// LSR : 左肩ロール軸から左腕先端
	{{  0.0f,  30.0f,-247.6f},{  0.0f,  0.0f,  0.0f}},	// LH  : 左腕先端
};

// リンクの接続

struct link_connect_T {
	int sister;
	int child;
	int mother;
};

const int NON = -1;

static const struct link_connect_T LINK_CONNECT[LINK_NUM] =
{
	{ NON, NON, HT  },	// CAM : カメラのリンク（光軸を求めるため）
	{ NON, CAM, HP  },	// HT  : 頭のチルト軸からカメラ (Acceliteは固定)
	{ NON, HT , SH  },	// HP  : 頭のパン軸からチルト軸
	{ NON, HP , CS  },	// SH  : 肩中心から頭のパン軸
	{ NON, SH , CC  },	// CS  : 腰から肩中心
	{ RY , CS , NON },	// CC  : 腰から肩中心
//	腰 -> 右足先
	{ LY , RR1, CC  },	// RY  : 右股ヨー軸から右股ロール軸
	{ NON, RP1, RY  },	// RR1 : 右股ロール軸から右股ピッチ軸
	{ NON, RP2, RR1 },	// RP1 : 右股ピッチ軸から右膝上ピッチ軸
	{ NON, RP3, RP1 },	// RP2 : 右膝上ピッチ軸から右膝下ピッチ軸
	{ NON, RP4, RP2 },	// RP3 : 右膝下ピッチ軸から右足首ピッチ軸
	{ NON, RR2, RP3 },	// RP4 : 右足首ピッチ軸から右足首ロール軸
	{ NON, RF , RP4 },	// RR2 : 右足首ロール軸から右足先
	{ NON, NON, RR2 },	// RF  : 右足先
//	腰 -> 左足先
	{ RSP, LR1, CC  },	// LY  : 左股ヨー軸から左股ロール軸
	{ NON, LP1, LY  },	// LR1 : 左股ロール軸から左股ピッチ軸
	{ NON, LP2, LR1 },	// LP1 : 左股ピッチ軸から左膝上ピッチ軸
	{ NON, LP3, LP1 },	// LP2 : 左膝上ピッチ軸から左膝下ピッチ軸
	{ NON, LP4, LP2 },	// LP3 : 左膝下ピッチ軸から左足首ピッチ軸
	{ NON, LR2, LP3 },	// LP4 : 左足首ピッチ軸から左足首ロール軸
	{ NON, LF , LP4 },	// LR2 : 左足首ロール軸から左足先
	{ NON, NON, LR2 },	// LF  : 左足先
//	肩中心 -> 右手先
	{ LSP, RSR, CS  },	// RSP : 右肩ピッチ軸からロール軸
	{ NON, RH , RSP },	// RSR : 右肩ロール軸から右腕先端
	{ NON, NON, RSR },	// RH  : 右腕先端
//	肩中心 -> 左手先
	{ NON, LSR, CS  },	// LSP : 右肩ピッチ軸からロール軸
	{ NON, LH , LSP },	// LSR : 右肩ロール軸から右腕先端
	{ NON, NON, LSR },	// LH  : 右腕先端
};

#endif

}
