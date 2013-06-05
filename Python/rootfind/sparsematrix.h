#ifndef MATH_SPARSE_MATRIX_H
#define MATH_SPARSE_MATRIX_H

#include "SparseMatrixTemplate.h"
#include "matrix.h"

namespace Math {

typedef SparseMatrixTemplate_RM<Real> SparseMatrix;

#if 0
struct SparseMatrix
{
  SparseMatrix();
  ~SparseMatrix();
  void init(int m, int n, int num_entries);
  void resize(int m, int n, int num_entries);
  void cleanup();

  bool Read(File&);
  bool Write(File&) const;
  void print(std::ostream&) const;

  void operator = (const SparseMatrix&);
  Real operator () (int i, int j) const;

  void setZero();
  void setMatrix(const Matrix&,Real zeroTol=Zero);
  void getMatrix(Matrix&) const;
  void makeSimilar(const SparseMatrix&);
  void copy(const SparseMatrix&);
  void scale(Real s);
  void scaleRows(const Vector& s);
  void scaleCols(const Vector& s);
  void mul(const SparseMatrix&, Real s);
  void mul(const Vector& w, Vector& v) const;		//vector multiply v = Mw
  void mul(const Matrix& w, Matrix& v) const;//dense matrix multiply v = Mw

  void mulTranspose(const Vector& w, Vector& v) const;
  Real dotRow(int i, const Vector&) const;
  Real dotCol(int j, const Vector&) const;
  //assumes lower triangle is filled, upper is transpose
  Real dotSymmL(int i, const Vector&) const;

  bool isValid() const;
  inline bool isEmpty() const { return m == 0 && n == 0; }
  inline bool hasDims(int M, int N) const { return m == M && n == N; }
  inline bool isSquare() const { return m == n; }
  inline bool isValidRow(int i) const { return i >= 0 && i < m; }
  inline bool isValidCol(int j) const { return j >= 0 && j < n; }

  inline int* rowIndices(int i) const { return col_indices + row_offsets[i]; }
  inline Real* rowValues(int i) const { return val_array + row_offsets[i]; }
  inline int numRowEntries(int i) const { return row_offsets[i+1]-row_offsets[i]; }

  /***************************************************************
   * Compressed row format:
   * row_offsets indexes into the column index array, by row, and is size m+1.
   * The number of entries per row can be determined by subtraction from
   *   the next value (the last value in row_offsets is num_entries)
   * The col_indices array marks the column of the corresponding
   *   element in the value array, and each column is sorted in
   *   increasing order.
   ****************************************************************/
  int* row_offsets;
  int* col_indices;
  Real* val_array;

  int m,n;
  int num_entries;

  static void self_test();
  static void self_test(int m, int n, int nnz);
};

class SparseMatrixBuilder
{
 public:
  SparseMatrixBuilder();
  ~SparseMatrixBuilder();
  void initialize(int m, int n);
  void resize(int m, int n);

  void print(std::ostream&) const;

  void setZero();
  void set(const SparseMatrix&);
  void zeroEntry(int i, int j);
  void insertEntry(int i, int j, Real val);
  void addEntry(int i, int j, Real val);

  void mul(const SparseMatrixBuilder& a, const SparseMatrixBuilder& b);
  void mul(const SparseMatrix& a, const SparseMatrix& b);

  Real* operator () (int i, int j);
  void buildMatrix(SparseMatrix& m) const;

  struct matrix_entry_t
  {
    int col;
    Real val;
  };

  typedef std::list<matrix_entry_t> RowT;
  typedef std::list<matrix_entry_t>::iterator RowIterator;

  RowT* rows;
  int m,n;
};

std::ostream& operator << (std::ostream& out, const SparseMatrix& m);
std::istream& operator >> (std::istream& out, SparseMatrix& m);

std::ostream& operator << (std::ostream& out, const SparseMatrixBuilder& m);
#endif //0

} //namespace Math

#endif
