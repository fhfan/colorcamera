
// colorcameraDlg.cpp : 实现文件
//

#include "stdafx.h"
#include "colorcamera.h"
#include "colorcameraDlg.h"
#include "afxdialogex.h"
#include <Windows.h>
#include <ctime>
#include <vector>
#include <string>
#include <iostream>
#include "VisionFunLib.h"



#ifdef _DEBUG
#define new DEBUG_NEW
#endif

//CcolorcameraDlg dlg;

// 用于应用程序“关于”菜单项的 CAboutDlg 对话框

class CAboutDlg : public CDialogEx
{
public:
	CAboutDlg();

// 对话框数据
	enum { IDD = IDD_ABOUTBOX };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支持

// 实现
protected:
	DECLARE_MESSAGE_MAP()
};

CAboutDlg::CAboutDlg() : CDialogEx(CAboutDlg::IDD)
{
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialogEx)
END_MESSAGE_MAP()


// CcolorcameraDlg 对话框



CcolorcameraDlg::CcolorcameraDlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(CcolorcameraDlg::IDD, pParent)
	, delay(0)
	, time(0)
	, exp(0)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
}

void CcolorcameraDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Text(pDX, IDC_EDIT1, delay);
	DDX_Text(pDX, IDC_EDIT7, time);
	DDX_Control(pDX, IDC_CHECK1, save);
	DDX_Control(pDX, IDC_COMBO1, cameratype);
	DDX_Control(pDX, IDC_CHECK2, edges);
	DDX_Control(pDX, IDC_CHECK3, re);
	DDX_Control(pDX, IDC_CHECK4, ge);
	DDX_Control(pDX, IDC_CHECK5, be);
	DDX_Control(pDX, IDC_CHECK10, he);
	DDX_Control(pDX, IDC_CHECK11, se);
	DDX_Control(pDX, IDC_CHECK12, ie);
	DDX_Control(pDX, IDC_CHECK9, edges1);
	DDX_Text(pDX, IDC_EDIT8, exp);
	DDX_Control(pDX, IDC_COMBO2, algorithm);
}

BEGIN_MESSAGE_MAP(CcolorcameraDlg, CDialogEx)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_BN_CLICKED(IDC_CHECK1, &CcolorcameraDlg::OnBnClickedCheck1)
	ON_BN_CLICKED(IDC_BUTTON1, &CcolorcameraDlg::OnBnClickedButton1)
	ON_BN_CLICKED(IDC_BUTTON2, &CcolorcameraDlg::OnBnClickedButton2)
	ON_BN_CLICKED(IDC_BUTTON3, &CcolorcameraDlg::OnBnClickedButton3)
	ON_MESSAGE(WM_UPDATEDATA, OnUpdateData)
	ON_CBN_SELCHANGE(IDC_COMBO1, &CcolorcameraDlg::OnCbnSelchangeCombo1)
	ON_CBN_SELCHANGE(IDC_COMBO2, &CcolorcameraDlg::OnCbnSelchangeCombo2)
END_MESSAGE_MAP()


// CcolorcameraDlg 消息处理程序

BOOL CcolorcameraDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// 将“关于...”菜单项添加到系统菜单中。

	// IDM_ABOUTBOX 必须在系统命令范围内。
	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);

	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != NULL)
	{
		BOOL bNameValid;
		CString strAboutMenu;
		bNameValid = strAboutMenu.LoadString(IDS_ABOUTBOX);
		ASSERT(bNameValid);
		if (!strAboutMenu.IsEmpty())
		{
			pSysMenu->AppendMenu(MF_SEPARATOR);
			pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, strAboutMenu);
		}
	}

	// 设置此对话框的图标。  当应用程序主窗口不是对话框时，框架将自动
	//  执行此操作
	SetIcon(m_hIcon, TRUE);			// 设置大图标
	SetIcon(m_hIcon, FALSE);		// 设置小图标

	// TODO:  在此添加额外的初始化代码
	CreateImageWindow();
	cameratype.AddString(_T("大华彩色"));
	cameratype.AddString(_T("大华黑白"));
	algorithm.AddString(_T("频域算法"));
	algorithm.AddString(_T("毛刺算法"));
	algorithm.AddString(_T("褶皱算法"));
	return TRUE;  // 除非将焦点设置到控件，否则返回 TRUE
}

void CcolorcameraDlg::OnSysCommand(UINT nID, LPARAM lParam)
{
	if ((nID & 0xFFF0) == IDM_ABOUTBOX)
	{
		CAboutDlg dlgAbout;
		dlgAbout.DoModal();
	}
	else
	{
		CDialogEx::OnSysCommand(nID, lParam);
	}
}

// 如果向对话框添加最小化按钮，则需要下面的代码
//  来绘制该图标。  对于使用文档/视图模型的 MFC 应用程序，
//  这将由框架自动完成。

void CcolorcameraDlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this); // 用于绘制的设备上下文

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// 使图标在工作区矩形中居中
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// 绘制图标
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialogEx::OnPaint();
	}
}

//当用户拖动最小化窗口时系统调用此函数取得光标
//显示。
HCURSOR CcolorcameraDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}


void CcolorcameraDlg::CreateImageWindow()
{
	HTuple HWindowID;
	CRect Rect;
	CWnd* pWnd = GetDlgItem(IDC_PIC);
	HWindowID = (Hlong)pWnd->m_hWnd;//获取父窗口句柄
	pWnd->GetWindowRect(&Rect);
	OpenWindow(0, 0, Rect.Width(), Rect.Height(), HWindowID, "visible", "", &hv_WindowID);
}


Mat HObject2Mat(HObject Hobj)
{
	Mat Image;
	HTuple htCh;

	HString cType;

	ConvertImageType(Hobj, &Hobj, "byte");

	CountChannels(Hobj, &htCh);

	Hlong wid = 0;

	Hlong hgt = 0;

	if (htCh[0].I() == 1)

	{

		HImage hImg(Hobj);

		void *ptr = hImg.GetImagePointer1(&cType, &wid, &hgt);//GetImagePointer1(Hobj, &ptr, &cType, &wid, &hgt);

		int W = (HTuple)wid;

		int H = (HTuple)hgt;

		Image.create(H, W, CV_8UC1);

		unsigned char *pdata = static_cast<unsigned char *>(ptr);

		memcpy(Image.data, pdata, W*H);

	}

	else if (htCh[0].I() == 3)

	{

		void *Rptr;

		void *Gptr;

		void *Bptr;

		HImage hImg(Hobj);

		hImg.GetImagePointer3(&Rptr, &Gptr, &Bptr, &cType, &wid, &hgt);

		int W = (HTuple)wid;

		int H = (HTuple)hgt;

		Image.create(H, W, CV_8UC3);

		vector<Mat> VecM(3);

		VecM[0].create(H, W, CV_8UC1);

		VecM[1].create(H, W, CV_8UC1);

		VecM[2].create(H, W, CV_8UC1);

		unsigned char *R = (unsigned char *)Rptr;

		unsigned char *G = (unsigned char *)Gptr;

		unsigned char *B = (unsigned char *)Bptr;

		memcpy(VecM[2].data, R, W*H);

		memcpy(VecM[1].data, G, W*H);

		memcpy(VecM[0].data, B, W*H);

		merge(VecM, Image);

	}
	return Image;
}


HObject Mat2HObject(const Mat &image)
{
	HObject Hobj = HObject();
	int hgt = image.rows;
	int wid = image.cols;
	int i;
	//  CV_8UC3  
	if (image.type() == CV_8UC3)
	{
		vector<cv::Mat> imgchannel;
		split(image, imgchannel);
		cv::Mat imgB = imgchannel[0];
		cv::Mat imgG = imgchannel[1];
		cv::Mat imgR = imgchannel[2];
		uchar* dataR = new uchar[hgt*wid];
		uchar* dataG = new uchar[hgt*wid];
		uchar* dataB = new uchar[hgt*wid];
		for (i = 0; i < hgt; i++)
		{
			memcpy(dataR + wid * i, imgR.data + imgR.step*i, wid);
			memcpy(dataG + wid * i, imgG.data + imgG.step*i, wid);
			memcpy(dataB + wid * i, imgB.data + imgB.step*i, wid);
		}
		GenImage3(&Hobj, "byte", wid, hgt, (Hlong)dataR, (Hlong)dataG, (Hlong)dataB);
		delete[]dataR;
		delete[]dataG;
		delete[]dataB;
		dataR = NULL;
		dataG = NULL;
		dataB = NULL;
	}
	//  CV_8UC1  
	else if (image.type() == CV_8UC1)
	{
		uchar* data = new uchar[hgt*wid];
		for (i = 0; i < hgt; i++)
			memcpy(data + wid * i, image.data + image.step*i, wid);
		GenImage1(&Hobj, "byte", wid, hgt, (Hlong)data);
		delete[] data;
		data = NULL;
	}
	return Hobj;
}


Mat label(HObject ho_Image, int upvalue, int lowvalue, int b, int g, int r)
{
	Mat srcImage, colImage;
	int j, k;
	Mat grayImage;
	vector<Mat> channels(3);
	srcImage = HObject2Mat(ho_Image);
	if (srcImage.channels() == 1)
	{
		for (j = 0; j < 3; j++)
			channels[j] = srcImage;
		merge(channels, colImage);
		//channels.clear();
		for (j = 0; j < srcImage.rows; j++)
		{
			for (k = 0; k < srcImage.cols; k++)
			{
				if (srcImage.at<uchar>(j, k)>upvalue || srcImage.at<uchar>(j, k)<lowvalue)
				{
					colImage.at<Vec3b>(j, k)[0] = b;
					colImage.at<Vec3b>(j, k)[1] = g;
					colImage.at<Vec3b>(j, k)[2] = r;
				}

			}
		}
		return colImage;
	}
	else
	{
		cvtColor(srcImage, grayImage, CV_BGR2GRAY);   //RGB 012
		for (j = 0; j < grayImage.rows; j++)
		{
			for (k = 0; k < grayImage.cols; k++)
			{
				if (grayImage.at<uchar>(j, k)>upvalue || grayImage.at<uchar>(j, k)<lowvalue)
				{
					srcImage.at<Vec3b>(j, k)[0] = b;
					srcImage.at<Vec3b>(j, k)[1] = g;
					srcImage.at<Vec3b>(j, k)[2] = r;
				}
			}
		}
		return srcImage;
	}
}

//Mat CcolorcameraDlg::edge(HObject ho_Image)
//{
//	Mat srcImage;
//	vector<Mat> channels1, channels(3);
//	srcImage = HObject2Mat(ho_Image);
//	if (srcImage.channels() == 1)
//	{
//		Canny(srcImage, srcImage, 50, 100);
//	}
//	else
//	{
//		split(srcImage, channels1);
//		GetDlgItem(IDC_CHECK3)->EnableWindow(TRUE);
//		GetDlgItem(IDC_CHECK4)->EnableWindow(TRUE);
//		GetDlgItem(IDC_CHECK5)->EnableWindow(TRUE);
//		GetDlgItem(IDC_CHECK6)->EnableWindow(TRUE);
//		GetDlgItem(IDC_CHECK7)->EnableWindow(TRUE);
//		GetDlgItem(IDC_CHECK8)->EnableWindow(TRUE);
//		if (dlg.re.GetCheck() == 1)
//		{
//			GetDlgItem(IDC_CHECK6)->EnableWindow(FALSE);
//			GetDlgItem(IDC_CHECK7)->EnableWindow(FALSE);
//			GetDlgItem(IDC_CHECK8)->EnableWindow(FALSE);
//			Canny(channels1.at(2), channels1.at(2), 50, 100);    //R
//		}
//		if (dlg.ge.GetCheck() == 1)
//		{
//			GetDlgItem(IDC_CHECK6)->EnableWindow(FALSE);
//			GetDlgItem(IDC_CHECK7)->EnableWindow(FALSE);
//			GetDlgItem(IDC_CHECK8)->EnableWindow(FALSE);
//			Canny(channels1.at(1), channels1.at(1), 50, 100);   //G
//		}
//		if (dlg.be.GetCheck() == 1)
//		{
//			GetDlgItem(IDC_CHECK6)->EnableWindow(FALSE);
//			GetDlgItem(IDC_CHECK7)->EnableWindow(FALSE);
//			GetDlgItem(IDC_CHECK8)->EnableWindow(FALSE);
//			Canny(channels1.at(0), channels1.at(0), 50, 100);   //B
//		}
//		if (dlg.re.GetCheck() == 1 || dlg.ge.GetCheck() == 1 || dlg.be.GetCheck() == 1)
//		{
//			for (int i = 0; i < 3; i++)
//				channels[i] = channels1.at(i);
//			merge(channels, srcImage);
//		}
//		if (dlg.rs.GetCheck() == 1)
//		{
//			GetDlgItem(IDC_CHECK3)->EnableWindow(FALSE);
//			GetDlgItem(IDC_CHECK4)->EnableWindow(FALSE);
//			GetDlgItem(IDC_CHECK5)->EnableWindow(FALSE);
//			GetDlgItem(IDC_CHECK7)->EnableWindow(FALSE);
//			GetDlgItem(IDC_CHECK8)->EnableWindow(FALSE);
//			srcImage = channels1.at(2).clone();
//		}
//		if (dlg.gs.GetCheck() == 1)
//		{
//			GetDlgItem(IDC_CHECK3)->EnableWindow(FALSE);
//			GetDlgItem(IDC_CHECK4)->EnableWindow(FALSE);
//			GetDlgItem(IDC_CHECK5)->EnableWindow(FALSE);
//			GetDlgItem(IDC_CHECK6)->EnableWindow(FALSE);
//			GetDlgItem(IDC_CHECK8)->EnableWindow(FALSE);
//			srcImage = channels1.at(1).clone();
//		}
//		if (dlg.bs.GetCheck() == 1)
//		{
//			GetDlgItem(IDC_CHECK3)->EnableWindow(FALSE);
//			GetDlgItem(IDC_CHECK4)->EnableWindow(FALSE);
//			GetDlgItem(IDC_CHECK5)->EnableWindow(FALSE);
//			GetDlgItem(IDC_CHECK6)->EnableWindow(FALSE);
//			GetDlgItem(IDC_CHECK7)->EnableWindow(FALSE);
//			srcImage = channels1.at(0).clone();
//		}
//	}
//	return srcImage;
//}



void deletefiles(HTuple dir)
{
	HTuple  hv_Files, hv_Index, hv_strLen, hv_Pos;
	HTuple  hv_Pos1;

	ListFiles(dir, (HTuple("files").Append("directories")),
		&hv_Files);
	{
		HTuple end_val1 = (hv_Files.TupleLength()) - 1;
		HTuple step_val1 = 1;
		for (hv_Index = 0; hv_Index.Continue(end_val1, step_val1); hv_Index += step_val1)
		{
			hv_strLen = HTuple(hv_Files[hv_Index]).TupleStrlen();
			hv_Pos = HTuple(hv_Files[hv_Index]).TupleStrrchr(".");
			if (0 != (-1 == hv_Pos))
			{
				hv_Pos1 = HTuple(hv_Files[hv_Index]).TupleStrrchr("\\");
				if (0 != ((hv_Pos1 + 1) == hv_strLen))
				{
					deletefiles(hv_Files[hv_Index]);
				}
			}
			else
			{
				DeleteFile(HTuple(hv_Files[hv_Index]));
			}
		}
	}
}

HTuple hv_k = 1;
int n = 1;
CString name, name1;
vector<Mat> channels1, channels(3);
HTuple hv_Row1, hv_Column1, hv_Phi, hv_Length1, hv_Length2;
void CcolorcameraDlg::OnBnClickedButton1()
{
	// TODO:  在此添加控件通知处理程序代码
	AfxBeginThread(StartCameraTest, this, THREAD_PRIORITY_NORMAL, 0, 0, NULL);
}


UINT CcolorcameraDlg::StartCameraTest(LPVOID pParam)
{
	// Local control variables
	HTuple hv_AcqHandle, end_val36, step_val36, m_ImageWidth, m_ImageHeight, hv_Classes, hv_Information, hv_Values;
	clock_t st, et;
	bool judg = true;
	if (n == 1)
	{
		try
		{
			hv_Classes.Clear();
			hv_Classes[0] = "image";
			{
				end_val36 = (hv_Classes.TupleLength()) - 1;
				step_val36 = 1;
				for (hv_k = 0; hv_k.Continue(end_val36, step_val36); hv_k += step_val36)
				{
					MakeDir(HTuple(hv_Classes[hv_k]));
				}
			}
		}
		catch (...)
		{
			for (hv_k = 0; hv_k.Continue(end_val36, step_val36); hv_k += step_val36)
				deletefiles(HTuple(hv_Classes[hv_k]));
		}
		n = 0;
	}
	if (name == "大华彩色")
	{
		try
		{
			InfoFramegrabber("HMV3rdParty", "device", &hv_Information, &hv_Values);
			OpenFramegrabber("HMV3rdParty", 0, 0, 0, 0, 0, 0, "progressive", -1, "default", -1, "false", "default", hv_Values[0].S(), 0, -1, &hv_AcqHandle);
		}
		catch (...)
		{
			//CloseFramegrabber(hv_AcqHandle);
			return 0;
		}
	}
	else if (name == "大华黑白")
	{
		try
		{
			InfoFramegrabber("HMV3rdParty", "device", &hv_Information, &hv_Values);
			OpenFramegrabber("HMV3rdParty", 0, 0, 0, 0, 0, 0, "progressive", -1, "default", -1, "false", "default", hv_Values[0].S(), 0, -1, &hv_AcqHandle);
		}
		catch (...)
		{
			//CloseFramegrabber(hv_AcqHandle);
			return 0;
		}
	}
	/*OpenFramegrabber("GigEVision2", 0, 0, 0, 0, 0, 0, "progressive", -1, "default",
		-1, "false", "default", "08eded3f3f22_DahuaTechnology_A7500CG20", 0, -1, &hv_AcqHandle);*/
	//open_framegrabber('GigEVision2', 0, 0, 0, 0, 0, 0, 'progressive', -1, 'default', -1, 'false', 'default', '08eded3f3f22_DahuaTechnology_A7500CG20', 0, -1, AcqHandle)
	//SetFramegrabberParam(hv_AcqHandle, "TriggerMode", "Off") Face;
	CcolorcameraDlg *pDlg = (CcolorcameraDlg*)pParam;
	pDlg->SendMessage(WM_UPDATEDATA, TRUE);
	//相机参数
	SetFramegrabberParam(hv_AcqHandle, "ExposureTime", pDlg->exp);
	//SetFramegrabberParam(hv_AcqHandle, "GainRaw", pDlg->gain);
	//SetFramegrabberParam(hv_AcqHandle, "Gamma", pDlg->gamma);
	//GrabImageStart(hv_AcqHandle, -1);
	//ClearWindow(pDlg->hv_WindowID);
	//SetFramegrabberParam(hv_AcqHandle, "grab_timeout", -1);
	GrabImage(&pDlg->ho_Image, hv_AcqHandle);   //同步拉流
	GetImagePointer1(pDlg->ho_Image, NULL, NULL, &m_ImageWidth, &m_ImageHeight);     //和加载静态图像相同
	//Mat m_Image = HObject2Mat(pDlg->ho_Image);
	SetPart(pDlg->hv_WindowID, 0, 0, m_ImageHeight - 1, m_ImageWidth - 1);
	pDlg->StartImageState = true;
	SetColor(pDlg->hv_WindowID, "red");
	SetDraw(pDlg->hv_WindowID, "margin");
	if (name1 == "频域算法")
	{
		if (pDlg->edges.GetCheck() == 1)
		{
			while (pDlg->StartImageState)
			{
				st = clock();
				GrabImage(&pDlg->ho_Image, hv_AcqHandle);
				Mat srcImage = HObject2Mat(pDlg->ho_Image);
				DispObj(pDlg->ho_Image, pDlg->hv_WindowID);
				if (judg == true)
				{
					DrawRectangle2(pDlg->hv_WindowID, &hv_Row1, &hv_Column1, &hv_Phi, &hv_Length1, &hv_Length2);
					judg = false;
				}
				GenRectangle2(&pDlg->ho_Rectangle, hv_Row1, hv_Column1, hv_Phi, hv_Length1, hv_Length2);
				DispRectangle2(pDlg->hv_WindowID, hv_Row1, hv_Column1, hv_Phi, hv_Length1, hv_Length2);
				ReduceDomain(pDlg->ho_Image, pDlg->ho_Rectangle, &pDlg->ho_ImageReduced);
				////pDlg->GetDlgItem(IDC_CHECK9)->EnableWindow(FALSE);
				//pDlg->GetDlgItem(IDC_CHECK10)->EnableWindow(FALSE);
				//pDlg->GetDlgItem(IDC_CHECK11)->EnableWindow(FALSE);
				//pDlg->GetDlgItem(IDC_CHECK12)->EnableWindow(FALSE);
				////pDlg->GetDlgItem(IDC_CHECK2)->EnableWindow(TRUE);
				//pDlg->GetDlgItem(IDC_CHECK3)->EnableWindow(TRUE);
				//pDlg->GetDlgItem(IDC_CHECK4)->EnableWindow(TRUE);
				//pDlg->GetDlgItem(IDC_CHECK5)->EnableWindow(TRUE);
				if (srcImage.channels() == 1)
				{
					//Canny(srcImage, srcImage, 50, 100);
					method.FrequencyDomain(pDlg->ho_ImageReduced, hv_Length1, hv_Length2);
				}
				else
				{
					srcImage = HObject2Mat(pDlg->ho_ImageReduced);
					split(srcImage, channels1);
					if (pDlg->re.GetCheck() == 1)
					{
						//Canny(channels1.at(2), channels1.at(2), 50, 100);    //R
						pDlg->ho_ImageReduced = Mat2HObject(channels1.at(2));
						method.FrequencyDomain(pDlg->ho_ImageReduced, hv_Length1, hv_Length2);
					}
					if (pDlg->ge.GetCheck() == 1)
					{
						//Canny(channels1.at(1), channels1.at(1), 50, 100);   //G
						pDlg->ho_ImageReduced = Mat2HObject(channels1.at(1));
						method.FrequencyDomain(pDlg->ho_ImageReduced, hv_Length1, hv_Length2);
					}
					if (pDlg->be.GetCheck() == 1)
					{
						//Canny(channels1.at(0), channels1.at(0), 50, 100);   //B
						pDlg->ho_ImageReduced = Mat2HObject(channels1.at(0));
						method.FrequencyDomain(pDlg->ho_ImageReduced, hv_Length1, hv_Length2);
					}
				}
				//DispObj(pDlg->ho_Image = Mat2HObject(srcImage), pDlg->hv_WindowID);
				for (int i = 0; i < method.hv_Area.TupleLength(); i++)
				{
					DispCircle(pDlg->hv_WindowID, method.hv_Row[i], method.hv_Column[i], 5);
				}
				et = clock();
				pDlg->time = et - st;
				pDlg->SendMessage(WM_UPDATEDATA, FALSE);
				if (pDlg->save.GetCheck() == 1)
				{
					WriteImage(pDlg->ho_Image, "jpg", 0, "image/" + hv_k);
					++hv_k;
				}
				Sleep(pDlg->delay);
			}

		}
		//DispObj(pDlg->ho_Image = Mat2HObject(pDlg->edge(pDlg->ho_Image)), pDlg->hv_WindowID);
		else
		{
			if (pDlg->edges1.GetCheck() == 1)
			{
				while (pDlg->StartImageState)
				{
					st = clock();
					GrabImage(&pDlg->ho_Image, hv_AcqHandle);
					Mat srcImage = HObject2Mat(pDlg->ho_Image);
					DispObj(pDlg->ho_Image, pDlg->hv_WindowID);
					if (judg == true)
					{
						DrawRectangle2(pDlg->hv_WindowID, &hv_Row1, &hv_Column1, &hv_Phi, &hv_Length1, &hv_Length2);
						judg = false;
					}
					GenRectangle2(&pDlg->ho_Rectangle, hv_Row1, hv_Column1, hv_Phi, hv_Length1, hv_Length2);
					DispRectangle2(pDlg->hv_WindowID, hv_Row1, hv_Column1, hv_Phi, hv_Length1, hv_Length2);
					ReduceDomain(pDlg->ho_Image, pDlg->ho_Rectangle, &pDlg->ho_ImageReduced);
					////pDlg->GetDlgItem(IDC_CHECK2)->EnableWindow(FALSE);
					//pDlg->GetDlgItem(IDC_CHECK3)->EnableWindow(FALSE);
					//pDlg->GetDlgItem(IDC_CHECK4)->EnableWindow(FALSE);
					//pDlg->GetDlgItem(IDC_CHECK5)->EnableWindow(FALSE);
					////pDlg->GetDlgItem(IDC_CHECK9)->EnableWindow(TRUE);
					//pDlg->GetDlgItem(IDC_CHECK10)->EnableWindow(TRUE);
					//pDlg->GetDlgItem(IDC_CHECK11)->EnableWindow(TRUE);
					//pDlg->GetDlgItem(IDC_CHECK12)->EnableWindow(TRUE);
				
					if (srcImage.channels() == 1)
					{
						//Canny(srcImage, srcImage, 50, 100);
						method.FrequencyDomain(pDlg->ho_ImageReduced, hv_Length1, hv_Length2);
					}
					else
					{
						srcImage = HObject2Mat(pDlg->ho_ImageReduced);
						cvtColor(srcImage, srcImage, CV_BGR2HSV);
						split(srcImage, channels1);
						if (pDlg->he.GetCheck() == 1)
						{
							//Canny(channels1.at(0), channels1.at(0), 50, 100);    //H
							pDlg->ho_ImageReduced = Mat2HObject(channels1.at(0));
							method.FrequencyDomain(pDlg->ho_ImageReduced, hv_Length1, hv_Length2);
						}
						if (pDlg->se.GetCheck() == 1)
						{
							//Canny(channels1.at(1), channels1.at(1), 50, 100);   //S
							pDlg->ho_ImageReduced = Mat2HObject(channels1.at(1));
							method.FrequencyDomain(pDlg->ho_ImageReduced, hv_Length1, hv_Length2);
						}
						if (pDlg->ie.GetCheck() == 1)
						{
							//Canny(channels1.at(2), channels1.at(2), 50, 100);   //V=I
							pDlg->ho_ImageReduced = Mat2HObject(channels1.at(2));
							method.FrequencyDomain(pDlg->ho_ImageReduced, hv_Length1, hv_Length2);
						}
					}
					for (int i = 0; i < method.hv_Area.TupleLength(); i++)
					{
						DispCircle(pDlg->hv_WindowID, method.hv_Row[i], method.hv_Column[i], 5);
					}
					et = clock();
					pDlg->time = et - st;
					pDlg->SendMessage(WM_UPDATEDATA, FALSE);
					if (pDlg->save.GetCheck() == 1)
					{
						WriteImage(pDlg->ho_Image, "jpg", 0, "image/" + hv_k);
						++hv_k;
					}
					Sleep(pDlg->delay);
					//DispObj(pDlg->ho_Image = Mat2HObject(srcImage), pDlg->hv_WindowID);
				}
			}
		}
	}
	else if (name1 == "毛刺算法")
	{
		if (pDlg->edges.GetCheck() == 1)
		{
			while (pDlg->StartImageState)
			{
				st = clock();
				GrabImage(&pDlg->ho_Image, hv_AcqHandle);
				Mat srcImage = HObject2Mat(pDlg->ho_Image);
				DispObj(pDlg->ho_Image, pDlg->hv_WindowID);
				if (judg == true)
				{
					DrawRectangle2(pDlg->hv_WindowID, &hv_Row1, &hv_Column1, &hv_Phi, &hv_Length1, &hv_Length2);
					judg = false;
				}
				GenRectangle2(&pDlg->ho_Rectangle, hv_Row1, hv_Column1, hv_Phi, hv_Length1, hv_Length2);
				DispRectangle2(pDlg->hv_WindowID, hv_Row1, hv_Column1, hv_Phi, hv_Length1, hv_Length2);
				ReduceDomain(pDlg->ho_Image, pDlg->ho_Rectangle, &pDlg->ho_ImageReduced);
				////pDlg->GetDlgItem(IDC_CHECK9)->EnableWindow(FALSE);
				//pDlg->GetDlgItem(IDC_CHECK10)->EnableWindow(FALSE);
				//pDlg->GetDlgItem(IDC_CHECK11)->EnableWindow(FALSE);
				//pDlg->GetDlgItem(IDC_CHECK12)->EnableWindow(FALSE);
				////pDlg->GetDlgItem(IDC_CHECK2)->EnableWindow(TRUE);
				//pDlg->GetDlgItem(IDC_CHECK3)->EnableWindow(TRUE);
				//pDlg->GetDlgItem(IDC_CHECK4)->EnableWindow(TRUE);
				//pDlg->GetDlgItem(IDC_CHECK5)->EnableWindow(TRUE);
				if (srcImage.channels() == 1)
				{
					//Canny(srcImage, srcImage, 50, 100);
					method.GlitchDetect(pDlg->ho_ImageReduced, 2, 22);
				}
				else
				{
					srcImage = HObject2Mat(pDlg->ho_ImageReduced);
					split(srcImage, channels1);
					if (pDlg->re.GetCheck() == 1)
					{
						//Canny(channels1.at(2), channels1.at(2), 50, 100);    //R
						pDlg->ho_ImageReduced = Mat2HObject(channels1.at(2));
						method.GlitchDetect(pDlg->ho_ImageReduced, 2, 22);
					}
					if (pDlg->ge.GetCheck() == 1)
					{
						//Canny(channels1.at(1), channels1.at(1), 50, 100);   //G
						pDlg->ho_ImageReduced = Mat2HObject(channels1.at(1));
						method.GlitchDetect(pDlg->ho_ImageReduced, 2, 22);
					}
					if (pDlg->be.GetCheck() == 1)
					{
						//Canny(channels1.at(0), channels1.at(0), 50, 100);   //B
						pDlg->ho_ImageReduced = Mat2HObject(channels1.at(0));
						method.GlitchDetect(pDlg->ho_ImageReduced, 2, 22);
					}
				}
				//DispObj(pDlg->ho_Image = Mat2HObject(srcImage), pDlg->hv_WindowID);
				for (int i = 0; i < method.hv_GlitchRow.TupleLength(); i++)
				{
					DispCircle(pDlg->hv_WindowID, method.hv_GlitchRow[i], method.hv_GlitchColumn[i], 5);
				}
				et = clock();
				pDlg->time = et - st;
				pDlg->SendMessage(WM_UPDATEDATA, FALSE);
				if (pDlg->save.GetCheck() == 1)
				{
					WriteImage(pDlg->ho_Image, "jpg", 0, "image/" + hv_k);
					++hv_k;
				}
				Sleep(pDlg->delay);
			}

		}
		//DispObj(pDlg->ho_Image = Mat2HObject(pDlg->edge(pDlg->ho_Image)), pDlg->hv_WindowID);
		else
		{
			if (pDlg->edges1.GetCheck() == 1)
			{
				while (pDlg->StartImageState)
				{
					st = clock();
					GrabImage(&pDlg->ho_Image, hv_AcqHandle);
					Mat srcImage = HObject2Mat(pDlg->ho_Image);
					DispObj(pDlg->ho_Image, pDlg->hv_WindowID);
					if (judg == true)
					{
						DrawRectangle2(pDlg->hv_WindowID, &hv_Row1, &hv_Column1, &hv_Phi, &hv_Length1, &hv_Length2);
						judg = false;
					}
					GenRectangle2(&pDlg->ho_Rectangle, hv_Row1, hv_Column1, hv_Phi, hv_Length1, hv_Length2);
					DispRectangle2(pDlg->hv_WindowID, hv_Row1, hv_Column1, hv_Phi, hv_Length1, hv_Length2);
					ReduceDomain(pDlg->ho_Image, pDlg->ho_Rectangle, &pDlg->ho_ImageReduced);
					////pDlg->GetDlgItem(IDC_CHECK2)->EnableWindow(FALSE);
					//pDlg->GetDlgItem(IDC_CHECK3)->EnableWindow(FALSE);
					//pDlg->GetDlgItem(IDC_CHECK4)->EnableWindow(FALSE);
					//pDlg->GetDlgItem(IDC_CHECK5)->EnableWindow(FALSE);
					////pDlg->GetDlgItem(IDC_CHECK9)->EnableWindow(TRUE);
					//pDlg->GetDlgItem(IDC_CHECK10)->EnableWindow(TRUE);
					//pDlg->GetDlgItem(IDC_CHECK11)->EnableWindow(TRUE);
					//pDlg->GetDlgItem(IDC_CHECK12)->EnableWindow(TRUE);
					if (srcImage.channels() == 1)
					{
						//Canny(srcImage, srcImage, 50, 100);
						method.GlitchDetect(pDlg->ho_ImageReduced, 2, 22);
					}
					else
					{
						srcImage = HObject2Mat(pDlg->ho_ImageReduced);
						cvtColor(srcImage, srcImage, CV_BGR2HSV);
						split(srcImage, channels1);
						if (pDlg->he.GetCheck() == 1)
						{
							//Canny(channels1.at(0), channels1.at(0), 50, 100);    //H
							pDlg->ho_ImageReduced = Mat2HObject(channels1.at(0));
							method.GlitchDetect(pDlg->ho_ImageReduced, 2, 22);
						}
						if (pDlg->se.GetCheck() == 1)
						{
							//Canny(channels1.at(1), channels1.at(1), 50, 100);   //S
							pDlg->ho_ImageReduced = Mat2HObject(channels1.at(1));
							method.GlitchDetect(pDlg->ho_ImageReduced, 2, 22);
						}
						if (pDlg->ie.GetCheck() == 1)
						{
							//Canny(channels1.at(2), channels1.at(2), 50, 100);   //V=I
							pDlg->ho_ImageReduced = Mat2HObject(channels1.at(2));
							method.GlitchDetect(pDlg->ho_ImageReduced, 2, 22);
						}
					}
					for (int i = 0; i < method.hv_GlitchRow.TupleLength(); i++)
					{
						DispCircle(pDlg->hv_WindowID, method.hv_GlitchRow[i], method.hv_GlitchColumn[i], 5);
					}
					et = clock();
					pDlg->time = et - st;
					pDlg->SendMessage(WM_UPDATEDATA, FALSE);
					if (pDlg->save.GetCheck() == 1)
					{
						WriteImage(pDlg->ho_Image, "jpg", 0, "image/" + hv_k);
						++hv_k;
					}
					Sleep(pDlg->delay);
				}
			}
		}
	}
	else if (name1 == "褶皱算法")
	{
		if (pDlg->edges.GetCheck() == 1)
		{
			while (pDlg->StartImageState)
			{
				st = clock();
				GrabImage(&pDlg->ho_Image, hv_AcqHandle);
				Mat srcImage = HObject2Mat(pDlg->ho_Image);
				DispObj(pDlg->ho_Image, pDlg->hv_WindowID);
				if (judg == true)
				{
					DrawRectangle2(pDlg->hv_WindowID, &hv_Row1, &hv_Column1, &hv_Phi, &hv_Length1, &hv_Length2);
					judg = false;
				}
				GenRectangle2(&pDlg->ho_Rectangle, hv_Row1, hv_Column1, hv_Phi, hv_Length1, hv_Length2);
				DispRectangle2(pDlg->hv_WindowID, hv_Row1, hv_Column1, hv_Phi, hv_Length1, hv_Length2);
				ReduceDomain(pDlg->ho_Image, pDlg->ho_Rectangle, &pDlg->ho_ImageReduced);
				////pDlg->GetDlgItem(IDC_CHECK9)->EnableWindow(FALSE);
				//pDlg->GetDlgItem(IDC_CHECK10)->EnableWindow(FALSE);
				//pDlg->GetDlgItem(IDC_CHECK11)->EnableWindow(FALSE);
				//pDlg->GetDlgItem(IDC_CHECK12)->EnableWindow(FALSE);
				////pDlg->GetDlgItem(IDC_CHECK2)->EnableWindow(TRUE);
				//pDlg->GetDlgItem(IDC_CHECK3)->EnableWindow(TRUE);
				//pDlg->GetDlgItem(IDC_CHECK4)->EnableWindow(TRUE);
				//pDlg->GetDlgItem(IDC_CHECK5)->EnableWindow(TRUE);
				if (srcImage.channels() == 1)
				{
					//Canny(srcImage, srcImage, 50, 100);
					method.FoldDetect(pDlg->ho_ImageReduced);
				}
				else
				{
					srcImage = HObject2Mat(pDlg->ho_ImageReduced);
					split(srcImage, channels1);
					if (pDlg->re.GetCheck() == 1)
					{
						//Canny(channels1.at(2), channels1.at(2), 50, 100);    //R
						pDlg->ho_ImageReduced = Mat2HObject(channels1.at(2));
						method.FoldDetect(pDlg->ho_ImageReduced);
					}
					if (pDlg->ge.GetCheck() == 1)
					{
						//Canny(channels1.at(1), channels1.at(1), 50, 100);   //G
						pDlg->ho_ImageReduced = Mat2HObject(channels1.at(1));
						method.FoldDetect(pDlg->ho_ImageReduced);
					}
					if (pDlg->be.GetCheck() == 1)
					{
						//Canny(channels1.at(0), channels1.at(0), 50, 100);   //B
						pDlg->ho_ImageReduced = Mat2HObject(channels1.at(0));
						method.FoldDetect(pDlg->ho_ImageReduced);
					}
				}
				//DispObj(pDlg->ho_Image = Mat2HObject(srcImage), pDlg->hv_WindowID);
				for (int i = 0; i < method.hv_Area.TupleLength(); i++)
				{
					DispCircle(pDlg->hv_WindowID, method.hv_Row[i], method.hv_Column[i], 5);
				}
				et = clock();
				pDlg->time = et - st;
				pDlg->SendMessage(WM_UPDATEDATA, FALSE);
				if (pDlg->save.GetCheck() == 1)
				{
					WriteImage(pDlg->ho_Image, "jpg", 0, "image/" + hv_k);
					++hv_k;
				}
				Sleep(pDlg->delay);
			}

		}
		//DispObj(pDlg->ho_Image = Mat2HObject(pDlg->edge(pDlg->ho_Image)), pDlg->hv_WindowID);
		else
		{
			if (pDlg->edges1.GetCheck() == 1)
			{
			while (pDlg->StartImageState)
			{
				st = clock();
				GrabImage(&pDlg->ho_Image, hv_AcqHandle);
				Mat srcImage = HObject2Mat(pDlg->ho_Image);
				DispObj(pDlg->ho_Image, pDlg->hv_WindowID);
				if (judg == true)
				{
					DrawRectangle2(pDlg->hv_WindowID, &hv_Row1, &hv_Column1, &hv_Phi, &hv_Length1, &hv_Length2);
					judg = false;
				}
				GenRectangle2(&pDlg->ho_Rectangle, hv_Row1, hv_Column1, hv_Phi, hv_Length1, hv_Length2);
				DispRectangle2(pDlg->hv_WindowID, hv_Row1, hv_Column1, hv_Phi, hv_Length1, hv_Length2);
				ReduceDomain(pDlg->ho_Image, pDlg->ho_Rectangle, &pDlg->ho_ImageReduced);
				////pDlg->GetDlgItem(IDC_CHECK2)->EnableWindow(FALSE);
				//pDlg->GetDlgItem(IDC_CHECK3)->EnableWindow(FALSE);
				//pDlg->GetDlgItem(IDC_CHECK4)->EnableWindow(FALSE);
				//pDlg->GetDlgItem(IDC_CHECK5)->EnableWindow(FALSE);
				////pDlg->GetDlgItem(IDC_CHECK9)->EnableWindow(TRUE);
				//pDlg->GetDlgItem(IDC_CHECK10)->EnableWindow(TRUE);
				//pDlg->GetDlgItem(IDC_CHECK11)->EnableWindow(TRUE);
				//pDlg->GetDlgItem(IDC_CHECK12)->EnableWindow(TRUE);
					if (srcImage.channels() == 1)
					{
						//Canny(srcImage, srcImage, 50, 100);
						method.FoldDetect(pDlg->ho_ImageReduced);
					}
					else
					{
						srcImage = HObject2Mat(pDlg->ho_ImageReduced);
						cvtColor(srcImage, srcImage, CV_BGR2HSV);
						split(srcImage, channels1);
						if (pDlg->he.GetCheck() == 1)
						{
							//Canny(channels1.at(0), channels1.at(0), 50, 100);    //H
							pDlg->ho_ImageReduced = Mat2HObject(channels1.at(0));
							method.FoldDetect(pDlg->ho_ImageReduced);
						}
						if (pDlg->se.GetCheck() == 1)
						{
							//Canny(channels1.at(1), channels1.at(1), 50, 100);   //S
							pDlg->ho_ImageReduced = Mat2HObject(channels1.at(1));
							method.FoldDetect(pDlg->ho_ImageReduced);
						}
						if (pDlg->ie.GetCheck() == 1)
						{
							//Canny(channels1.at(2), channels1.at(2), 50, 100);   //V=I
							pDlg->ho_ImageReduced = Mat2HObject(channels1.at(2));
							method.FoldDetect(pDlg->ho_ImageReduced);
						}
					}
					for (int i = 0; i < method.hv_Area.TupleLength(); i++)
					{
						DispCircle(pDlg->hv_WindowID, method.hv_Row[i], method.hv_Column[i], 5);
					}
					et = clock();
					pDlg->time = et - st;
					pDlg->SendMessage(WM_UPDATEDATA, FALSE);
					if (pDlg->save.GetCheck() == 1)
					{
						WriteImage(pDlg->ho_Image, "jpg", 0, "image/" + hv_k);
						++hv_k;
					}
					Sleep(pDlg->delay);
				}
			}
		}
	}
	CloseFramegrabber(hv_AcqHandle); // 结束线程
	return 0;
}

LRESULT CcolorcameraDlg::OnUpdateData(WPARAM wParam, LPARAM lParam)
{
	UpdateData(wParam);
	return 0;
}


void CcolorcameraDlg::OnBnClickedCheck1()
{
	// TODO:  在此添加控件通知处理程序代码
	UpdateData(TRUE);
}


void CcolorcameraDlg::OnBnClickedButton2()
{
	// TODO:  在此添加控件通知处理程序代码
	/*GetDlgItem(IDC_CHECK3)->EnableWindow(FALSE);
	GetDlgItem(IDC_CHECK4)->EnableWindow(FALSE);
	GetDlgItem(IDC_CHECK5)->EnableWindow(FALSE);
	GetDlgItem(IDC_CHECK10)->EnableWindow(FALSE);
	GetDlgItem(IDC_CHECK11)->EnableWindow(FALSE);
	GetDlgItem(IDC_CHECK12)->EnableWindow(FALSE);*/
	StartImageState = false;
}


void CcolorcameraDlg::OnBnClickedButton3()
{
	// TODO:  在此添加控件通知处理程序代码
	StartImageState = false;
	//AfxGetMainWnd()->SendMessage(WM_CLOSE);
	HANDLE Process = GetCurrentProcess();
	TerminateProcess(Process, 0);
}


void CcolorcameraDlg::OnCbnSelchangeCombo1()
{
	// TODO:  在此添加控件通知处理程序代码
	int nIdx = cameratype.GetCurSel();
	cameratype.GetLBText(nIdx, name);
}


void CcolorcameraDlg::OnCbnSelchangeCombo2()
{
	// TODO:  在此添加控件通知处理程序代码
	int nIdx = algorithm.GetCurSel();
	algorithm.GetLBText(nIdx, name1);
}
