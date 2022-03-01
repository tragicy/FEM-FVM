using System.Collections;
using System.Collections.Generic;
using UnityEngine;
public class FEM_23 : MonoBehaviour
{
    Vector2[] V;
    Vector2[] X;
    Vector2[] Force;
    float[] Inv_mass;
    int[] triangles;
    Matrix4x4[] inv_Dm;
    float[] ARef;
    Vector3 g = new Vector3(0, -10.0f, 0);
    int number = 0;
    int tri_number = 0;
    float dt = 1.0f/120.0f;
    float stiffness_0 = 2000;
    float stiffness_1 = 500;

    // Start is called before the first frame update
    void Start()
    {
        //Application.targetFrameRate = 120;
        int n = 10;
        tri_number = (n - 1) * (n - 1) * 2;
        number = n * n;
        X = new Vector2[number];
        V = new Vector2[number];
        Force = new Vector2[number];
        ARef = new float[tri_number];
        Inv_mass = new float[number];
        //Force = new Vector3[number];
        X[0] = new Vector2(1, 0);
        X[1] = new Vector2(0, 0);
        X[2] = new Vector2(0, 1);
        triangles= new int[(n - 1) *(n - 1) * 6];
        triangles[0] = 0;
        triangles[1] = 1;
        triangles[2] = 2;


        Vector3[] vertices = new Vector3[number];
        for (int i = 0; i < number; i++)
        {
            Inv_mass[i] = 1;
        }
        //vertices[3] = new Vector3(1, 1, 0);


        Mesh mesh = GetComponent<MeshFilter>().mesh;
        //Resize the mesh.
        X = new Vector2[n * n];
        Vector2[] UV = new Vector2[n * n];
        int[] T = new int[(n - 1) * (n - 1) * 6];
        for (int j = 0; j < n; j++)
            for (int i = 0; i < n; i++)
            {
                X[j * n + i] = new Vector3(5 - 10.0f * i / (n - 1), 10 - 10.0f * j / (n - 1),0);
                UV[j * n + i] = new Vector3(i / (n - 1.0f), j / (n - 1.0f));
            }
        int t = 0;
        for (int j = 0; j < n - 1; j++)
            for (int i = 0; i < n - 1; i++)
            {
                T[t * 6 + 0] = j * n + i;
                T[t * 6 + 1] = j * n + i + 1;
                T[t * 6 + 2] = (j + 1) * n + i + 1;
                T[t * 6 + 3] = j * n + i;
                T[t * 6 + 4] = (j + 1) * n + i + 1;
                T[t * 6 + 5] = (j + 1) * n + i;
                t++;
            }
        triangles = T;
        for (int i = 0; i < number; i++)
        {
            vertices[i] = new Vector3(X[i].x, X[i].y, 0);
        }
        //mesh.vertices = X;
        //mesh.triangles = T;
        //mesh.uv = UV;
        //mesh.RecalculateNormals();
        Mesh Cube = new Mesh
        {
            vertices = vertices,
            uv = UV,
            triangles = triangles
        };
        GetComponent<MeshFilter>().mesh = Cube;
        Cube.RecalculateNormals();
        inv_Dm = new Matrix4x4[tri_number];
        for (int i = 0; i < tri_number; i++)
        {
            inv_Dm[i] = Build_Edge_Matrix(i);
        }
    }
    Matrix4x4 Build_Edge_Matrix(int tri)
    {
        Matrix4x4 ret = Matrix4x4.zero;
        Vector2 X10 = X[triangles[tri * 3 + 0]] - X[triangles[tri * 3 + 1]];
        Vector2 X20 = X[triangles[tri * 3 + 0]] - X[triangles[tri * 3 + 2]];
        ARef[tri] = 0.5f* Vector3.Magnitude(Vector3.Cross(X10, X20));
        ret[0, 0] = X10.x;
        ret[1, 0] = X10.y;
        ret[0, 1] = X20.x;
        ret[1, 1] = X20.y;
        ret[2,2] = 1;
        ret[3,3] = 1;
        ret = Matrix4x4.Inverse(ret);
        return ret;
    }
    // Update is called once per frame
    void Update()
    {
        //return;
        for (int tri = 0; tri < tri_number; tri++)
        {
            int x0 = triangles[3 * tri+0];
            int x1 = triangles[3 * tri + 1];
            int x2 = triangles[3 * tri + 2];
            Vector2 x10 = X[x0] - X[x1];
            Vector2 x20 = X[x0] - X[x2];
            Matrix4x4 ret = Matrix4x4.zero;
            ret[0, 0] = x10.x;
            ret[1, 0] = x10.y;
            ret[0, 1] = x20.x;
            ret[1, 1] = x20.y;
            ret[2,2] = 1;
            ret[3,3] = 1;
            Matrix4x4 F = ret * inv_Dm[tri];
            //Green Strain
            Matrix4x4 G = Matrix4x4.Transpose(F) * F;
            G = MatrixTool.mMinus2d(G, Matrix4x4.identity);
            G = MatrixTool.mMultiply2d(G, 0.5f);
            //Second PK Stress
            Matrix4x4 S1 = MatrixTool.mMultiply2d(G, 2 * stiffness_1);
            Matrix4x4 S2 = MatrixTool.mMultiply2d(Matrix4x4.identity, MatrixTool.Trace2d(G) * stiffness_0);
            Matrix4x4 S = MatrixTool.mAdd2d(S1, S2);
            Matrix4x4 P = F * S;
            //Force
            Matrix4x4 force = P * Matrix4x4.Transpose(inv_Dm[tri]);
            force = MatrixTool.mMultiply2d(force, ARef[tri]);
            Vector2 f1 = new Vector3(force[0,0],force[1,0]);
            Vector2 f2 = new Vector3(force[0,1],force[1,1]);
            //Debug.Log(f1);
            //Debug.Log(f2);
            Force[x1] += f1;
            Force[x2] += f2;
            Force[x0] += -1.0f * (f1 + f2);
        }
        for (int i = 0; i < number; i++)
        {
            //if (i == 0 || i == 9)
            //    continue;

            Vector2 a = g;
            a += Force[i] * Inv_mass[i];
            V[i] += a * dt;
            X[i] += V[i] * dt;
            V[i] *= 0.99f;

            //TODO: (Particle) collision with floor.
            if (X[i].y <= -3)
            {
                X[i].y = -3;
                V[i].y = -0.5f * V[i].y;
            }
            Force[i] = Vector3.zero;
        }

        Vector3[] vertices = new Vector3[number];
        for (int i = 0; i < number; i++)
        {
            vertices[i] = new Vector3(X[i].x, X[i].y, 0);
        }
        GetComponent<MeshFilter>().mesh.vertices = vertices;
    }
}
