using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MatrixTool
{
    public static Matrix4x4 mAdd(Matrix4x4 m1, Matrix4x4 m2)
    {
        Matrix4x4 newMat = Matrix4x4.identity;
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                newMat[i, j] = m1[i, j] + m2[i, j];
            }
        }
        return newMat;
    }

    public static Matrix4x4 mMinus(Matrix4x4 m1, Matrix4x4 m2)
    {
        Matrix4x4 newMat = Matrix4x4.identity;
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                newMat[i, j] = m1[i, j] - m2[i, j];
            }
        }
        return newMat;
    }

    public static Matrix4x4 mMultiply(Matrix4x4 m1, float f)
    {
        Matrix4x4 newMat = Matrix4x4.identity;
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                newMat[i, j] = m1[i, j] * f;
            }
        }
        return newMat;
    }

    public static float Trace(Matrix4x4 m)
    {
        return m[0,0] + m[1,1] + m[2,2];
    }
}
