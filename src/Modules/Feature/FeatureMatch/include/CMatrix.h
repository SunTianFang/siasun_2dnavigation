//                                         - MATRIX.H -
//
//   CMatrix类的接口定义。
//

#ifndef __CMatrix1
#define __CMatrix1

//////////////////////////////////////////////////////////////////////////////
//   CMatrix类的接口定义
class CMatrix
{
public:
	int m_nRow;
	int m_nCol;
	float** m_Array;

private:
	void Copy(const CMatrix& m);
	void Clear();

public:
	CMatrix(int nRow, int nCol);

	CMatrix();

	~CMatrix();

	// 生成矩阵
	bool Create(int nRow, int nCol);

	// 矩阵是否是一个方阵
	bool IsSquare();

	// 求矩阵的伴随矩阵
	CMatrix GetAccompany()const;

	// 求矩阵的行列式
	float Determinant() const;

	CMatrix GetRemainder(int nRow, int nCol)const;

	// 是否可以求逆
	bool CanContrary()const;

	// 求矩阵的转置矩阵
	CMatrix Transpose()const;

	// 求矩阵的逆矩阵
	CMatrix operator ~();

	CMatrix operator / (CMatrix & m);
	CMatrix operator / (float m);
	CMatrix operator * (const CMatrix & m);
	CMatrix operator * (float m);

	bool CanMutiply(const CMatrix & m)const;

	bool CanAddSub(const CMatrix & m)const;

	CMatrix(const CMatrix &);

	CMatrix operator + (const CMatrix & m);

	CMatrix operator - (const CMatrix & m);

	CMatrix & operator = (const CMatrix &m);

	CMatrix & operator = (float m);

	float GetAt(int nRow, int nCol);

	void SetAt(int nRow, int nCol, float fValue);

#ifdef _MSC_VER
	void Dump();
#endif
};
#endif
