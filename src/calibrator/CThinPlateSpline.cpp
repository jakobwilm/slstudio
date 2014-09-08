/*
* CThinPlateSpline.cpp
*
*  Created on: 24.01.2010
*      Author: schmiedm
*/

#include <vector>
#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include "CThinPlateSpline.h"
using namespace cv;

CThinPlateSpline::CThinPlateSpline() {
	FLAG_MAPS_FORWARD_SET = false;
	FLAG_MAPS_BACK_WARP_SET = false;
	FLAG_COEFFS_BACK_WARP_SET = false;
	FLAG_COEFFS_FORWARD_WARP_SET = false;
}

CThinPlateSpline::CThinPlateSpline(const std::vector<Point2f>& pS, const std::vector<Point2f>& pD)
{
	if(pS.size() == pS.size())
	{
		pSrc = pS;
		pDst = pD;
	}

	FLAG_MAPS_FORWARD_SET = false;
	FLAG_MAPS_BACK_WARP_SET = false;
	FLAG_COEFFS_BACK_WARP_SET = false;
	FLAG_COEFFS_FORWARD_WARP_SET = false;
}

CThinPlateSpline::~CThinPlateSpline() {
}

void CThinPlateSpline::addCorrespondence(const Point2f& pS, const Point2f& pD)
{
	pSrc.push_back(pS);
	pDst.push_back(pD);

	// tell the class to recompute the coefficients if neccesarry 
	FLAG_COEFFS_BACK_WARP_SET = false;
	FLAG_COEFFS_FORWARD_WARP_SET = false;
	FLAG_MAPS_FORWARD_SET = false;
	FLAG_MAPS_BACK_WARP_SET = false;
	
}

void CThinPlateSpline::setCorrespondences(const std::vector<Point2f>& pS, const std::vector<Point2f>& pD)
{
	pSrc = pS;
	pDst = pD;

	// tell the class to recompute the coefficients if neccesarry 
	FLAG_COEFFS_BACK_WARP_SET = false;
	FLAG_COEFFS_FORWARD_WARP_SET = false;
	FLAG_MAPS_FORWARD_SET = false;
	FLAG_MAPS_BACK_WARP_SET = false;
}

double CThinPlateSpline::fktU(const Point2f& p1, const Point2f& p2)
{
	double r = pow(((double)p1.x - (double)p2.x), 2) + pow(((double)p1.y - (double)p2.y), 2);

	if (r == 0)
		return 0.0;
	else 
	{
		r = sqrt(r); // vector length
		double r2 = pow(r, 2);

		return (r2 * log(r2));
	}
}

void CThinPlateSpline::computeSplineCoeffs(std::vector<Point2f>& iPIn, std::vector<Point2f>& iiPIn, float lambda,const TPS_INTERPOLATION tpsInter)
{

    std::vector<Point2f>* iP = NULL;
    std::vector<Point2f>*	iiP = NULL;

	if(tpsInter == FORWARD_WARP)
	{
		iP = &iPIn;
		iiP = &iiPIn;

		// keep information which coefficients are set
		FLAG_COEFFS_BACK_WARP_SET = true;
		FLAG_COEFFS_FORWARD_WARP_SET = false;
	}
	else if(tpsInter == BACK_WARP)
	{
		iP = &iiPIn;
		iiP = &iPIn;

		// keep information which coefficients are set
		FLAG_COEFFS_BACK_WARP_SET = false;
		FLAG_COEFFS_FORWARD_WARP_SET = true;
	}

	//get number of corresponding points
	int dim = 2;
	int n = iP->size();

	//Initialize mathematical datastructures
	Mat_<float> V(dim,n+dim+1,0.0);
	Mat_<float> P(n,dim+1,1.0);
	Mat_<float> K = (K.eye(n,n)*lambda);
	Mat_<float> L(n+dim+1,n+dim+1,0.0);

	// fill up K und P matrix
    std::vector<Point2f>::iterator itY;
    std::vector<Point2f>::iterator itX;

	int y = 0;
	for (itY = iP->begin(); itY != iP->end(); ++itY, y++) {
		int x = 0;
		for (itX = iP->begin(); itX != iP->end(); ++itX, x++) {
			if (x != y) {
				K(y, x) = (float)fktU(*itY, *itX);
			}
		}
		P(y,1) = (float)itY->y;
		P(y,2) = (float)itY->x;
	}

	Mat Pt;
	transpose(P,Pt);

	// insert K into L
	Rect range = Rect(0, 0, n, n);
	Mat Lr(L,range);
	K.copyTo(Lr);


	// insert P into L
	range = Rect(n, 0, dim + 1, n);
	Lr = Mat(L,range);
	P.copyTo(Lr);

	// insert Pt into L
	range = Rect(0,n,n,dim+1);
	Lr = Mat(L,range);
	Pt.copyTo(Lr);

	// fill V array
    std::vector<Point2f>::iterator it;
	int u = 0;

	for(it = iiP->begin(); it != iiP->end(); ++it)
	{
		V(0,u) = (float)it->y;
		V(1,u) = (float)it->x;
		u++;
	}

	// transpose V
	Mat Vt;
	transpose(V,Vt);

	cMatrix = Mat_<float>(n+dim+1,dim,0.0);

	// invert L
	Mat invL;
	invert(L,invL,DECOMP_LU);

	//multiply(invL,Vt,cMatrix);
	cMatrix = invL * Vt;

	//compensate for rounding errors
	for(int row = 0; row < cMatrix.rows; row++)
	{
		for(int col = 0; col < cMatrix.cols; col++)
		{
			double v = cMatrix(row,col);
			if(v > (-1.0e-006) && v < (1.0e-006) )
			{
				cMatrix(row,col) = 0.0;
			}
		}
	}
}


Point2f CThinPlateSpline::interpolate_forward_(const Point2f& p)
{
    Point2f interP;
    std::vector<Point2f>* pList = &pSrc;

	int k1 = cMatrix.rows - 3;
	int kx = cMatrix.rows - 2;
	int ky = cMatrix.rows - 1;

	double a1 = 0, ax = 0, ay = 0, cTmp = 0, uTmp = 0, tmp_i = 0, tmp_ii = 0;

	for (int i = 0; i < 2; i++) {
		a1 = cMatrix(k1,i);
		ax = cMatrix(kx,i);
		ay = cMatrix(ky,i);

		tmp_i = a1 + ax * p.y + ay * p.x;
		tmp_ii = 0;

		for (int j = 0; j < (int)pSrc.size(); j++) {
			cTmp = cMatrix(j,i);
			uTmp = fktU( (*pList)[j], p);

			tmp_ii = tmp_ii + (cTmp * uTmp);
		}

		if (i == 0) {
			interP.y = (float)(tmp_i + tmp_ii);
		}
		if (i == 1) {
			interP.x = (float)(tmp_i + tmp_ii);
		}
	}

	return interP;
}
Point2f CThinPlateSpline::interpolate_back_(const Point2f& p)
{
    Point2f interP;
    std::vector<Point2f>* pList = &pDst;

	int k1 = cMatrix.rows - 3;
	int kx = cMatrix.rows - 2;
	int ky = cMatrix.rows - 1;

	double a1 = 0, ax = 0, ay = 0, cTmp = 0, uTmp = 0, tmp_i = 0, tmp_ii = 0;

	for (int i = 0; i < 2; i++) {
		a1 = cMatrix(k1,i);
		ax = cMatrix(kx,i);
		ay = cMatrix(ky,i);

		tmp_i = a1 + ax * p.y + ay * p.x;
		tmp_ii = 0;

		for (int j = 0; j < (int)pSrc.size(); j++) {
			cTmp = cMatrix(j,i);
			uTmp = fktU( (*pList)[j], p);

			tmp_ii = tmp_ii + (cTmp * uTmp);
		}

		if (i == 0) {
			interP.y = (float)(tmp_i + tmp_ii);
		}
		if (i == 1) {
			interP.x = (float)(tmp_i + tmp_ii);
		}
	}

	return interP;
}

Point2f CThinPlateSpline::interpolate(const Point2f& p, const TPS_INTERPOLATION tpsInter)
{

    // only compute the coefficients new if they weren't already computed
    // or there had been changes to the points
    if(tpsInter == BACK_WARP && !FLAG_COEFFS_BACK_WARP_SET)
    {
        computeSplineCoeffs(pSrc,pDst,1000,tpsInter);
    }
    else if(tpsInter == FORWARD_WARP && !FLAG_COEFFS_FORWARD_WARP_SET)
    {
        computeSplineCoeffs(pSrc,pDst,1000,tpsInter);
    }

	if(tpsInter == BACK_WARP)
	{
		return interpolate_back_(p);
	}
	else if(tpsInter == FORWARD_WARP)
	{
		return interpolate_forward_(p);
	}
	else
	{
		return interpolate_back_(p);
	}
	
}

void CThinPlateSpline::warpImage(const Mat& src, Mat& dst, float lambda, const int interpolation,const TPS_INTERPOLATION tpsInter)
{
	Size size = src.size();
	dst = Mat(size,src.type());

	// only compute the coefficients new if they weren't already computed
	// or there had been changes to the points
	if(tpsInter == BACK_WARP && !FLAG_COEFFS_BACK_WARP_SET)
	{
		computeSplineCoeffs(pSrc,pDst,lambda,tpsInter);
	}
	else if(tpsInter == FORWARD_WARP && !FLAG_COEFFS_FORWARD_WARP_SET)
	{
		computeSplineCoeffs(pSrc,pDst,lambda,tpsInter);
	}
	
	computeMaps(size,mapx,mapy);

	remap(src,dst,mapx,mapy,interpolation);
}

void CThinPlateSpline::getMaps(Mat& mx, Mat& my)
{
	mx = mapx;
	my = mapy;
}

void CThinPlateSpline::computeMaps(const Size& dstSize, Mat_<float>& mx, Mat_<float>& my,const TPS_INTERPOLATION tpsInter)
{
	mx = Mat_<float>(dstSize);
	my = Mat_<float>(dstSize);

    Point2f p(0, 0);
    Point_<float> intP(0, 0);
	
	for (int row = 0; row < dstSize.height; row++) {
		for (int col = 0; col < dstSize.width; col++) {
            p = Point2f(col, row);
			intP = interpolate(p,tpsInter);
			mx(row, col) = intP.x;
			my(row, col) = intP.y;
		}
	}

	if(tpsInter == BACK_WARP)
	{	
		FLAG_MAPS_BACK_WARP_SET = true;
		FLAG_MAPS_FORWARD_SET = false;
	}
	else if(tpsInter == FORWARD_WARP)
	{
		FLAG_MAPS_BACK_WARP_SET = false;
		FLAG_MAPS_FORWARD_SET = true;
	}
}

