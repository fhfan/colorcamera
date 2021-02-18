
// colorcameraDlg.h : 头文件
//

#pragma once
#include "afxwin.h"
#include "HalconCpp.h"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "resource.h"

#define WM_UPDATEDATA WM_USER+5
using namespace HalconCpp;
using namespace cv;
using namespace std;


// CcolorcameraDlg 对话框
class CcolorcameraDlg : public CDialogEx
{
// 构造
public:
	CcolorcameraDlg(CWnd* pParent = NULL);	// 标准构造函数

// 对话框数据
	enum { IDD = IDD_COLORCAMERA_DIALOG };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV 支持


// 实现
protected:
	HICON m_hIcon;

	// 生成的消息映射函数
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	DECLARE_MESSAGE_MAP()
public:
	HObject ho_Image;
	HObject ho_Rectangle;
	HObject ho_ImageReduced;
	HTuple hv_WindowID;
	bool StartImageState;
	void CreateImageWindow();
	Mat edge(HObject ho_Image);
	//Mat HObject2Mat(HObject Hobj);
	//HObject Mat2HObject(const Mat &image);
	//Mat label(HObject ho_Image, int upvalue, int lowvalue, int b, int g, int r);
	int delay;
	afx_msg void OnBnClickedCheck1();
	afx_msg void OnBnClickedButton1();
	static UINT StartCameraTest(LPVOID pParam);
	LRESULT OnUpdateData(WPARAM wParam, LPARAM lParam);
	long time;
	CButton save;
	afx_msg void OnBnClickedButton2();
	afx_msg void OnBnClickedButton3();
	CComboBox cameratype;
	afx_msg void OnCbnSelchangeCombo1();
	CButton edges;
	CButton re;
	CButton ge;
	CButton be;
	CButton he;
	CButton se;
	CButton ie;
	CButton edges1;
	double exp;
	CComboBox algorithm;
	afx_msg void OnCbnSelchangeCombo2();
};

//extern CcolorcameraDlg dlg;
