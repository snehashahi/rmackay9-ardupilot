/*
 *  N dimensional matrix operations
 */

#pragma GCC optimize("O3")

#include "matrix4f.h"

/*
bool Matrix4f::inverse(Matrix4f& inv) const
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

    inv[00] = det *  (v[11] * A2323 - v[12] * A1323 + v[13] * A1223);
    inv[01] = det * -(v[01] * A2323 - v[02] * A1323 + v[03] * A1223);
    inv[02] = det *  (v[01] * A2313 - v[02] * A1313 + v[03] * A1213);
    inv[03] = det * -(v[01] * A2312 - v[02] * A1312 + v[03] * A1212);
    inv[10] = det * -(v[10] * A2323 - v[12] * A0323 + v[13] * A0223);
    inv[11] = det *  (v[00] * A2323 - v[02] * A0323 + v[03] * A0223);
    inv[12] = det * -(v[00] * A2313 - v[02] * A0313 + v[03] * A0213);
    inv[13] = det *  (v[00] * A2312 - v[02] * A0312 + v[03] * A0212);
    inv[20] = det *  (v[10] * A1323 - v[11] * A0323 + v[13] * A0123);
    inv[21] = det * -(v[00] * A1323 - v[01] * A0323 + v[03] * A0123);
    inv[22] = det *  (v[00] * A1313 - v[01] * A0313 + v[03] * A0113);
    inv[23] = det * -(v[00] * A1312 - v[01] * A0312 + v[03] * A0112);
    inv[30] = det * -(v[10] * A1223 - v[11] * A0223 + v[12] * A0123);
    inv[31] = det *  (v[00] * A1223 - v[01] * A0223 + v[02] * A0123);
    inv[32] = det * -(v[00] * A1213 - v[01] * A0213 + v[02] * A0113);
    inv[33] = det *  (v[00] * A1212 - v[01] * A0212 + v[02] * A0112);
    return true;
}
*/
