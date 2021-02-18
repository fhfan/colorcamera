#include "stdafx.h"
#include "VisionFunLib.h"

Method method;
void Method::FrequencyDomain(HObject ho_Image, HTuple hv_Width, HTuple hv_Height)
{
	HObject  ho_GaussFilter1, ho_GaussFilter2;
	HObject  ho_Filter, ho_ImageFFT, ho_ImageConvol, ho_ImageFiltered;
	HObject  ho_ImageResult, ho_RegionDynThresh, ho_ConnectedRegions;
	HObject  ho_SelectedRegions, ho_RegionUnion, ho_RegionClosing;
	HObject  ho_ConnectedRegions1, ho_SelectedRegions1, ho_ContCircle;

	HTuple  hv_Sigma1, hv_Sigma2;
	HTuple  hv_Min, hv_Max, hv_Range;
	HTuple  hv_Number;
	//GetImageSize(ho_Image, &hv_Width, &hv_Height);

	OptimizeRftSpeed(hv_Width, hv_Height, "standard");

	hv_Sigma1 = 10.0;
	hv_Sigma2 = 3.0;
	GenGaussFilter(&ho_GaussFilter1, hv_Sigma1, hv_Sigma1, 0.0, "none", "rft", hv_Width,
		hv_Height);
	GenGaussFilter(&ho_GaussFilter2, hv_Sigma2, hv_Sigma2, 0.0, "none", "rft", hv_Width,
		hv_Height);
	SubImage(ho_GaussFilter1, ho_GaussFilter2, &ho_Filter, 1, 0);

	//Rgb1ToGray(ho_Image, &ho_Image);

	RftGeneric(ho_Image, &ho_ImageFFT, "to_freq", "none", "complex", hv_Width);
	ConvolFft(ho_ImageFFT, ho_Filter, &ho_ImageConvol);
	RftGeneric(ho_ImageConvol, &ho_ImageFiltered, "from_freq", "n", "real", hv_Width);

	GrayRangeRect(ho_ImageFiltered, &ho_ImageResult, 5, 5);
	MinMaxGray(ho_ImageResult, ho_ImageResult, 0, &hv_Min, &hv_Max, &hv_Range);
	Threshold(ho_ImageResult, &ho_RegionDynThresh, (HalconCpp::HTuple(2).TupleConcat(hv_Max*0.6)).TupleMax(),
		255);
	HalconCpp::Connection(ho_RegionDynThresh, &ho_ConnectedRegions);
	SelectShape(ho_ConnectedRegions, &ho_SelectedRegions, "area", "and", 4, 99999);
	Union1(ho_SelectedRegions, &ho_RegionUnion);
	ClosingCircle(ho_RegionUnion, &ho_RegionClosing, 10);
	HalconCpp::Connection(ho_RegionClosing, &ho_ConnectedRegions1);
	SelectShape(ho_ConnectedRegions1, &ho_SelectedRegions1, "area", "and", 10, 99999);
	AreaCenter(ho_SelectedRegions1, &hv_Area, &hv_Row, &hv_Column);
}

void Method::GlitchDetect(HObject ho_Image, int objThresh, float areaThresh)
{
	HalconCpp::HObject  ho_Fin, ho_ROI_0, ho_ImageReduced, ho_Foreground;
	HalconCpp::HObject  ho_ForeConnected, ho_ForeSelected, ho_ForeRegionUnion;
	HalconCpp::HObject  ho_Background, ho_ClosedBackground, ho_RegionDifference;
	HalconCpp::HObject  ho_OpenRegion, ho_RegionIntersection, ho_InterConnectedRegions;
	HalconCpp::HObject  ho_GlitchRegions;

	// Local control variables
	HalconCpp::HTuple  hv_ROIArea, hv_ROIRow, hv_ROIColumn, hv_ROIWidth;
	HalconCpp::HTuple  hv_ROIHeight, hv_UsedThreshold, hv_ForeArea, hv_ForeRow;
	HalconCpp::HTuple  hv_ForeColumn, hv_ForeIndices, hv_ObjNum, hv_ForeNum;
	HalconCpp::HTuple  hv_InterArea, hv_InterRow, hv_InterColumn, hv_MinArea;

	HalconCpp::HTuple hv_GlitchArea, hv_GlitchCenterRow, hv_GlitchCenterColumn;

	BinaryThreshold(ho_Image, &ho_Foreground, "max_separability", "light", &hv_UsedThreshold);
	HalconCpp::Connection(ho_Foreground, &ho_ForeConnected);
	AreaCenter(ho_ForeConnected, &hv_ForeArea, &hv_ForeRow, &hv_ForeColumn);
	TupleSortIndex(hv_ForeArea, &hv_ForeIndices);
	//异常处理

	//hv_ObjNum = 2;
	hv_ObjNum = objThresh;
	hv_ForeNum = hv_ForeIndices.TupleLength();
	if (hv_ObjNum>hv_ForeNum | hv_ObjNum<1)
	{
		hv_ObjNum = hv_ForeNum;
	}
	SelectObj(ho_ForeConnected, &ho_ForeSelected, (hv_ForeIndices.TupleSelectRange(hv_ForeNum - hv_ObjNum, hv_ForeNum - 1)) + 1);
	//select_shape (ConnectedRegions, SelectedRegions, 'area', 'and', 31944.4, Area)
	//select_shape_std (ConnectedRegions, SelectedRegions, 'max_area', 20)
	Union1(ho_ForeSelected, &ho_ForeRegionUnion);

	//二值化分割图像，使出自动全局阈值分割单通道图像
	//cout << "LightDark_2: " << LightDark_2[0].S() << endl;
	BinaryThreshold(ho_Image, &ho_Background, "max_separability", "light", &hv_UsedThreshold);
	//闭运算，平滑背景边界
	/*cout << "closingCircleRadius is: " << closingCircleRadius[0].I() << endl;
	cout << "openingWidth is: " << openingWidth[0].I() << endl;
	cout << "openingHeight is: " << openingHeight[0].I() << endl;*/

	ClosingCircle(ho_Background, &ho_ClosedBackground, 3.5);
	//closing_rectangle1 (Background, ClosedBackground, 100, 100)
	//闭运算后的背景区域和原背景区域进行求补集
	Difference(ho_ClosedBackground, ho_Background, &ho_RegionDifference);
	//开运算，消除边界毛刺
	OpeningRectangle1(ho_RegionDifference, &ho_OpenRegion, 10, 10);

	//与已选择的obj取交集
	Intersection(ho_OpenRegion, ho_ForeRegionUnion, &ho_RegionIntersection);
	//获取毛刺的面积和坐标
	AreaCenter(ho_RegionIntersection, &hv_InterArea, &hv_InterRow, &hv_InterColumn);

	//异常处理
	//hv_MinArea = 22;
	hv_MinArea = areaThresh;
	if (0 != (hv_InterArea >= hv_MinArea))
	{
		//fin_index := find(FinArea[>]minArea,1)
		HalconCpp::Connection(ho_RegionIntersection, &ho_InterConnectedRegions);
		SelectShape(ho_InterConnectedRegions, &ho_GlitchRegions, "area", "and", hv_MinArea,
			hv_InterArea);
	}
	else
	{
		GenEmptyObj(&ho_GlitchRegions);
	}

	HalconCpp::HObject ho_GlitchRegionsUnion;
	//异常处理，将所有区域联合
	Union1(ho_GlitchRegions, &ho_GlitchRegionsUnion);
	AreaCenter(ho_GlitchRegionsUnion, &hv_GlitchArea, &hv_GlitchCenterRow, &hv_GlitchCenterColumn);
	if (0 != hv_GlitchArea)
	{
		GetRegionPoints(ho_GlitchRegionsUnion, &hv_GlitchRow, &hv_GlitchColumn);
	}
}

void Method::FoldDetect(HObject ho_Image)
{
	HalconCpp::HObject  ho_Region, ho_RegionClosing;
	HalconCpp::HObject  ho_RegionFillUp, ho_RegionErosion, ho_ImageReduced1, ho_ImageMedian1;
	HalconCpp::HObject  ho_RegionDynThresh2, ho_RegionOpening;
	HalconCpp::HObject  ho_RegionClosing2, ho_ConnectedRegions3, ho_SelectedRegions2;

	Threshold(ho_Image, &ho_Region, 0, 200);
	ClosingCircle(ho_Region, &ho_RegionClosing, 10);
	FillUp(ho_RegionClosing, &ho_RegionFillUp);
	ErosionCircle(ho_RegionFillUp, &ho_RegionErosion, 8);
	ReduceDomain(ho_Image, ho_RegionErosion, &ho_ImageReduced1);
	MedianImage(ho_ImageReduced1, &ho_ImageMedian1, "circle", 9, "mirrored");
	DynThreshold(ho_ImageReduced1, ho_ImageMedian1, &ho_RegionDynThresh2, 7, "dark");
	OpeningCircle(ho_RegionDynThresh2, &ho_RegionOpening, 1);
	ClosingCircle(ho_RegionOpening, &ho_RegionClosing2, 5);
	HalconCpp::Connection(ho_RegionClosing2, &ho_ConnectedRegions3);

	SelectShape(ho_ConnectedRegions3, &ho_SelectedRegions2, "area", "and", 10, 10000);
	AreaCenter(ho_SelectedRegions2, &hv_Area, &hv_Row, &hv_Column);
}
