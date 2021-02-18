#include "colorcameraDlg.h"




class Method
{
public:
	HTuple hv_Area, hv_Row, hv_Column;
	HTuple hv_GlitchRow, hv_GlitchColumn;

public:
	void FrequencyDomain(HObject ho_Image, HTuple hv_Width, HTuple hv_Height);
	void GlitchDetect(HObject ho_Image, int objThresh, float areaThresh);
	void FoldDetect(HObject ho_Image);
};

extern Method method;
