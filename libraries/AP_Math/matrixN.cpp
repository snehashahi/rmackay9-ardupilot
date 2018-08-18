/*
 *  N dimensional matrix operations
 */

#pragma GCC optimize("O3")

#include "matrixN.h"


// multiply two vectors to give a matrix, in-place
template <typename T, uint8_t N>
void MatrixN<T,N>::mult(const VectorN<T,N> &A, const VectorN<T,N> &B)
{
    for (uint8_t i = 0; i < N; i++) {
        for (uint8_t j = 0; j < N; j++) {
            v[i][j] = A[i] * B[j];
        }
    }
}

// subtract B from the matrix
template <typename T, uint8_t N>
MatrixN<T,N> &MatrixN<T,N>::operator -=(const MatrixN<T,N> &B)
{
    for (uint8_t i = 0; i < N; i++) {
        for (uint8_t j = 0; j < N; j++) {
            v[i][j] -= B.v[i][j];
        }
    }
    return *this;
}

// add B to the matrix
template <typename T, uint8_t N>
MatrixN<T,N> &MatrixN<T,N>::operator +=(const MatrixN<T,N> &B)
{
    for (uint8_t i = 0; i < N; i++) {
        for (uint8_t j = 0; j < N; j++) {
            v[i][j] += B.v[i][j];
        }
    }
    return *this;
}

// allow a MatrixN to be used as an array of vectors, 0 indexed
template <typename T, uint8_t N>
VectorN<T,N> &MatrixN<T,N>::operator[](uint8_t i)
{
    VectorN<T,N> *_v = &v;
#if MATH_CHECK_INDEXES
    assert(i >= 0 && i < 3);
#endif
    return _v[i];
}

template <typename T, uint8_t N>
const VectorN<T,N> &MatrixN<T,N>::operator[](uint8_t i) const
{
    const VectorN<T,N> *_v = &v;
#if MATH_CHECK_INDEXES
    assert(i >= 0 && i < 3);
#endif
    return _v[i];
}

/*template <typename T, uint8_t N>
bool MatrixN<T,N>::invert()
{
    MatrixN<T,N> inv;
    bool success = inverse(inv);
    if (success) {
        *this = inv;
    }
    return success;
}
*/

// Matrix symmetry routine
template <typename T, uint8_t N>
void MatrixN<T,N>::force_symmetry(void)
{
    for (uint8_t i = 0; i < N; i++) {
        for (uint8_t j = 0; j < (i - 1); j++) {
            v[i][j] = (v[i][j] + v[j][i]) * 0.5;
            v[j][i] = v[i][j];
        }
    }
}

//template bool MatrixN<float,4>::inverse(MatrixN<float,4>& inv) const;

// matrix4f invert function
// calculate the inverse of this matrix
// results stored in inv
// returns true if this matrix is invertible, false otherwise and inv is unmodified
//template <typename T, uint8_t N>
//template bool MatrixN<float,4>::inverse(MatrixN<float,4>& inv) const;
//{
//    return true;
//}

/*
template <typename T, uint8_t N>
bool MatrixN<T,N>::inverse(MatrixN<T,N>& inv) const
{
    return true;
}
*/

template <typename T, uint8_t N>
bool MatrixN<float,4>::invert()
{
    const float A2323 = v[2][2] * v[3][3] - v[2][3] * v[3][2];
    const float A1323 = v[2][1] * v[3][3] - v[2][3] * v[3][1];
    const float A1223 = v[2][1] * v[3][2] - v[2][2] * v[3][1];
    const float A0323 = v[2][0] * v[3][3] - v[2][3] * v[3][0];
    const float A0223 = v[2][0] * v[3][2] - v[2][2] * v[3][0];
    const float A0123 = v[2][0] * v[3][1] - v[2][1] * v[3][0];
    const float A2313 = v[1][2] * v[3][3] - v[1][3] * v[3][2];
    const float A1313 = v[1][1] * v[3][3] - v[1][3] * v[3][1];
    const float A1213 = v[1][1] * v[3][2] - v[1][2] * v[3][1];
    const float A2312 = v[1][2] * v[2][3] - v[1][3] * v[2][2];
    const float A1312 = v[1][1] * v[2][3] - v[1][3] * v[2][1];
    const float A1212 = v[1][1] * v[2][2] - v[1][2] * v[2][1];
    const float A0313 = v[1][0] * v[3][3] - v[1][3] * v[3][0];
    const float A0213 = v[1][0] * v[3][2] - v[1][2] * v[3][0];
    const float A0312 = v[1][0] * v[2][3] - v[1][3] * v[2][0];
    const float A0212 = v[1][0] * v[2][2] - v[1][2] * v[2][0];
    const float A0113 = v[1][0] * v[3][1] - v[1][1] * v[3][0];
    const float A0112 = v[1][0] * v[2][1] - v[1][1] * v[2][0];

    const float det = v[0][0] * (v[1][1] * A2323 - v[1][2] * A1323 + v[1][3] * A1223)
                     -v[0][1] * (v[1][0] * A2323 - v[1][2] * A0323 + v[1][3] * A0223)
                     +v[0][2] * (v[1][0] * A1323 - v[1][1] * A0323 + v[1][3] * A0123)
                     -v[0][3] * (v[1][0] * A1223 - v[1][1] * A0223 + v[1][2] * A0123);
    det = 1 / det;

    // store original matrix
    MatrixN<float,4> temp_v;

    temp_v[00] = det *  (v[11] * A2323 - v[12] * A1323 + v[13] * A1223);
    temp_v[01] = det * -(v[01] * A2323 - v[02] * A1323 + v[03] * A1223);
    temp_v[02] = det *  (v[01] * A2313 - v[02] * A1313 + v[03] * A1213);
    temp_v[03] = det * -(v[01] * A2312 - v[02] * A1312 + v[03] * A1212);
    temp_v[10] = det * -(v[10] * A2323 - v[12] * A0323 + v[13] * A0223);
    temp_v[11] = det *  (v[00] * A2323 - v[02] * A0323 + v[03] * A0223);
    temp_v[12] = det * -(v[00] * A2313 - v[02] * A0313 + v[03] * A0213);
    temp_v[13] = det *  (v[00] * A2312 - v[02] * A0312 + v[03] * A0212);
    temp_v[20] = det *  (v[10] * A1323 - v[11] * A0323 + v[13] * A0123);
    temp_v[21] = det * -(v[00] * A1323 - v[01] * A0323 + v[03] * A0123);
    temp_v[22] = det *  (v[00] * A1313 - v[01] * A0313 + v[03] * A0113);
    temp_v[23] = det * -(v[00] * A1312 - v[01] * A0312 + v[03] * A0112);
    temp_v[30] = det * -(v[10] * A1223 - v[11] * A0223 + v[12] * A0123);
    temp_v[31] = det *  (v[00] * A1223 - v[01] * A0223 + v[02] * A0123);
    temp_v[32] = det * -(v[00] * A1213 - v[01] * A0213 + v[02] * A0113);
    temp_v[33] = det *  (v[00] * A1212 - v[01] * A0212 + v[02] * A0112);

    // update v
    v = temp_v;
    return true;
}

template void MatrixN<float,4>::mult(const VectorN<float,4> &A, const VectorN<float,4> &B);
template MatrixN<float,4> &MatrixN<float,4>::operator -=(const MatrixN<float,4> &B);
template MatrixN<float,4> &MatrixN<float,4>::operator +=(const MatrixN<float,4> &B);
template void MatrixN<float,4>::force_symmetry(void);
template bool MatrixN<float,4>::invert();
