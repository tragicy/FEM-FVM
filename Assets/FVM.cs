using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.IO;

public class FVM : MonoBehaviour
{
	float dt 			= 0.003f;
    float mass 			= 1;
	float stiffness_0	= 20000.0f;
    float stiffness_1 	= 5000.0f;
    float damp			= 0.999f;
    Vector3 G = new Vector3(0, -9.80f, 0);

	int[] 		Tet;
	int tet_number;			//The number of tetrahedra
 
	Vector3[] 	Force;
	Vector3[] 	V;
	Vector3[] 	X;
	int number;				//The number of vertices

	Matrix4x4[] inv_Dm;

	//For Laplacian smoothing.
	Vector3[]   V_sum;
	int[]		V_num;

    Dictionary<int, List<int>> nearPoints = new Dictionary<int, List<int>>();

	SVD svd = new SVD();
    Dictionary<Vector2, bool> edgeDic = new Dictionary<Vector2, bool>();

    List<int> touchedPointsIndex = new List<int>();
    float Un = 0.3f;
    float Ut = 0.9f;

    // Start is called before the first frame update
    void Start()
    {
        Application.targetFrameRate = 60;
    	// FILO IO: Read the house model from files.
    	// The model is from Jonathan Schewchuk's Stellar lib.
    	{
    		string fileContent = File.ReadAllText("Assets/house2.ele");
    		string[] Strings = fileContent.Split(new char[]{' ', '\t', '\r', '\n'}, StringSplitOptions.RemoveEmptyEntries);
    		
    		tet_number=int.Parse(Strings[0]);
        	Tet = new int[tet_number*4];

    		for(int tet=0; tet<tet_number; tet++)
    		{
				Tet[tet*4+0]=int.Parse(Strings[tet*5+4])-1;
				Tet[tet*4+1]=int.Parse(Strings[tet*5+5])-1;
				Tet[tet*4+2]=int.Parse(Strings[tet*5+6])-1;
				Tet[tet*4+3]=int.Parse(Strings[tet*5+7])-1;
			}
    	}
    	{
			string fileContent = File.ReadAllText("Assets/house2.node");
    		string[] Strings = fileContent.Split(new char[]{' ', '\t', '\r', '\n'}, StringSplitOptions.RemoveEmptyEntries);
    		number = int.Parse(Strings[0]);
    		X = new Vector3[number];
       		for(int i=0; i<number; i++)
       		{
       			X[i].x=float.Parse(Strings[i*5+5])*0.4f;
       			X[i].y=float.Parse(Strings[i*5+6])*0.4f;
       			X[i].z=float.Parse(Strings[i*5+7])*0.4f;
       		}
    		//Centralize the model.
	    	Vector3 center=Vector3.zero;
	    	for(int i=0; i<number; i++)		center+=X[i];
	    	center=center/number;
	    	for(int i=0; i<number; i++)
	    	{
	    		X[i]-=center;
	    		float temp=X[i].y;
	    		X[i].y=X[i].z;
	    		X[i].z=temp;
	    	}
		}
        //tet_number = 1;
        //Tet = new int[tet_number * 4];
        //Tet[0] = 0;
        //Tet[1] = 1;
        //Tet[2] = 2;
        //Tet[3] = 3;

        //number = 4;
        //X = new Vector3[number];
        //V = new Vector3[number];
        //Force = new Vector3[number];
        //X[0] = new Vector3(0, 0, 0);
        //X[1] = new Vector3(1, 0, 0);
        //X[2] = new Vector3(0, 1, 0);
        //X[3] = new Vector3(0, 0, 1);


        //Create triangle mesh.
        Vector3[] vertices = new Vector3[tet_number*12];
        int vertex_number=0;
        for(int tet=0; tet<tet_number; tet++)
        {
        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];
        	vertices[vertex_number++]=X[Tet[tet*4+1]];

        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];

        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+1]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];

        	vertices[vertex_number++]=X[Tet[tet*4+1]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];
        }

        int[] triangles = new int[tet_number*12];
        for(int t=0; t<tet_number*4; t++)
        {
        	triangles[t*3+0]=t*3+0;
        	triangles[t*3+1]=t*3+1;
        	triangles[t*3+2]=t*3+2;
        }

        Mesh mesh = GetComponent<MeshFilter> ().mesh;
		mesh.vertices  = vertices;
		mesh.triangles = triangles;
		mesh.RecalculateNormals ();


		V 	  = new Vector3[number];
        Force = new Vector3[number];
        V_sum = new Vector3[number];
        V_num = new int[number];

        //TODO: Need to allocate and assign inv_Dm
        inv_Dm = new Matrix4x4[tet_number];
        for (int i = 0; i < tet_number; i++)
        {
            inv_Dm[i] = Build_Edge_Matrix(i);
        }
        for (int i = 0; i < tet_number; i++)
        {
            int a = Tet[i * 4 + 0];
            int b = Tet[i * 4 + 1];
            int c = Tet[i * 4 + 2];
            int d = Tet[i * 4 + 3];
            SortAndAdd(a, b);
            SortAndAdd(a, c);
            SortAndAdd(a, d);
            SortAndAdd(b, c);
            SortAndAdd(b, d);
            SortAndAdd(c, b);
        }

        //Build edges
        List<Vector2> edgeList = new List<Vector2>();
        foreach (var item in edgeDic.Keys)
        {
            edgeList.Add(item);
        }
        edgeList.Sort((a,b)=>{
            if (a.x == b.x)
            {
                if (a.y < b.y)
                    return -1;
                else if (a.y == b.y)
                    return 0;
                else
                    return 1;
            }
            else if (a.x < b.x)
                return -1;
            else
                return 1;
        });
        for (int i = 0; i < edgeList.Count; i++)
        {
            fillNearPoints((int)edgeList[i].x, (int)edgeList[i].y);
            fillNearPoints((int)edgeList[i].y, (int)edgeList[i].x);
        }
        Debug.Log(nearPoints.Count);
    }
    void SortAndAdd(float a, float b)
    {
        if (a > b)
        {
            float c = b;
            b = a;
            a = c;
        }
        if (!edgeDic.ContainsKey(new Vector2(a, b)))
            edgeDic.Add(new Vector2(a, b), true);
    }
    void fillNearPoints(int a, int b)
    {
        if (nearPoints.ContainsKey(a))
        {
            nearPoints[a].Add(b);
        }
        else
        {
            var lb = new List<int>();
            lb.Add(b);
            nearPoints.Add(a, lb);
        }
    }
    Matrix4x4 Build_Edge_Matrix(int tet)
    {
    	Matrix4x4 ret=Matrix4x4.zero;
        //TODO: Need to build edge matrix here.
        Vector3 X10 = X[Tet[4 * tet + 0]] - X[Tet[4 * tet + 1]];
        ret[0, 0] = X10.x;
        ret[1, 0] = X10.y;
        ret[2, 0] = X10.z;

        Vector3 X20 = X[Tet[4 * tet + 0]] - X[Tet[4 * tet + 2]];
        ret[0, 1] = X20.x;
        ret[1, 1] = X20.y;
        ret[2, 1] = X20.z;

        Vector3 X30 = X[Tet[4 * tet + 0]] - X[Tet[4 * tet + 3]];
        ret[0, 2] = X30.x;
        ret[1, 2] = X30.y;
        ret[2, 2] = X30.z;

        ret[3, 3] = 1;
        ret = Matrix4x4.Inverse(ret);
		return ret;
    }
    void Collision_Impulse(Vector3 P, Vector3 N)
    {
        for (int i = 0; i < X.Length; i ++)
        {
            //Vector3 testVertex = X[i];
            //testVertex = transform.position + GetRx(testVertex, Matrix4x4.Rotate(transform.rotation));
            float PhiVertex = Vector3.Dot((X[i] - P), N);
            if (PhiVertex < 0.1f)
            {
                touchedPointsIndex.Add(i);
                X[i] -= (PhiVertex - 0.05f) * N;
            }
        }
        if (touchedPointsIndex.Count != 0)
        {
            for (int i = 0; i < touchedPointsIndex.Count; i++)
            {
                Vector3 v = V[touchedPointsIndex[i]];
                if (Vector3.Dot(v, N) < 0)
                {
                    Vector3 Vn = Vector3.Dot(v, N) * N;
                    Vector3 Vt = v - Vn;
                    Vector3 VNewN = -Un * Vn;

                    float a = 1 - Ut * (1 + Un) * Vn.magnitude / Vt.magnitude;
                    if (a < 0)
                        a = 0;
                    Vector3 VNewt = a * Vt;
                    Vector3 VNew = VNewt + VNewN;
                    V[touchedPointsIndex[i]] = VNew;
                }
            }
        }
        touchedPointsIndex.Clear();

    }
    
    void _Update()
    {
    	// Jump up.
		if(Input.GetKeyDown(KeyCode.Space))
    	{
    		for(int i=0; i<number; i++)
    			V[i].y+=0.5f;
    	}

    	for(int i=0 ;i<number; i++)
    	{
            Force[i] += G / mass;
    		//TODO: Add gravity to Force.
    	}

    	for(int tet=0; tet<tet_number; tet++)
    	{
            //TODO: Deformation Gradient
            int x0 = Tet[4 * tet + 0];
            int x1 = Tet[4 * tet + 1];
            int x2 = Tet[4 * tet + 2];
            int x3 = Tet[4 * tet + 3];

            Vector3 x10 = X[x0] - X[x1];
            Vector3 x20 = X[x0] - X[x2];
            Vector3 x30 = X[x0] - X[x3];

            Matrix4x4 ret = Matrix4x4.zero;
            ret[0, 0] = x10.x;
            ret[1, 0] = x10.y;
            ret[2, 0] = x10.z;

            ret[0, 1] = x20.x;
            ret[1, 1] = x20.y;
            ret[2, 1] = x20.z;

            ret[0, 2] = x30.x;
            ret[1, 2] = x30.y;
            ret[2, 2] = x30.z;
            ret[3, 3] = 1;
            Matrix4x4 F = ret * inv_Dm[tet];
            //TODO: Green Strain
            Matrix4x4 G = (Matrix4x4.Transpose(F) * F);
            G = MatrixTool.mMinus(G, Matrix4x4.identity);
            G = MatrixTool.mMultiply(G, 0.5f);
            //TODO: Second PK Stress
            Matrix4x4 S1 = MatrixTool.mMultiply(G, 2 * stiffness_1);
            Matrix4x4 S2 = MatrixTool.mMultiply(Matrix4x4.identity, MatrixTool.Trace(G) * stiffness_0);
            Matrix4x4 S = MatrixTool.mAdd(S1, S2);
            Matrix4x4 P = F * S;
            
            //svd
            //Matrix4x4 U = Matrix4x4.identity;
            //Matrix4x4 A = Matrix4x4.identity;
            //Matrix4x4 Vt = Matrix4x4.identity;
            //svd.svd(F, ref U, ref A, ref Vt);
            //Vt = Matrix4x4.Transpose(Vt);
            //Matrix4x4 diag = Matrix4x4.identity;
            //float I = A[0, 0] * A[0, 0] + A[1, 1] * A[1, 1] + A[2, 2] * A[2, 2];
            ////float II = A[0, 0] * A[0, 0] * A[0, 0] * A[0, 0] + A[1, 1] * A[1, 1] * A[1, 1] * A[1, 1] + A[2, 2] * A[2, 2] * A[2, 2] * A[2, 2];
            //for (int i = 0; i < 3; i++)
            //{
            //    float Ai = A[i, i];
            //    diag[i, i] = stiffness_0 * (I - 3) * Ai * 0.25f + 0.5f * stiffness_1 * Ai*(Ai*Ai -1);
            //}
            //Matrix4x4 P = U * diag * Vt;

            Matrix4x4 force = P * Matrix4x4.Transpose(inv_Dm[tet]);
            force = MatrixTool.mMultiply(force, 1.0f / (-6.0f * Matrix4x4.Determinant(inv_Dm[tet])));
            //TODO: Elastic Force
            Vector3 f1 = new Vector3(force[0, 0], force[1, 0], force[2, 0]);
            Vector3 f2 = new Vector3(force[0, 1], force[1, 1], force[2, 1]);
            Vector3 f3 = new Vector3(force[0, 2], force[1, 2], force[2, 2]);
            Force[x1] += f1;
            Force[x2] += f2;
            Force[x3] += f3;
            Force[x0] += -1.0f * (f1 + f2 + f3);
        }
        for (int i=0; i<number; i++)
    	{
            //TODO: Update X and V here.
            Vector3 a = Force[i] / mass;
            //Smooth
            for (int j = 0; j < nearPoints[i].Count; j++)
            {
                V[i] += V[nearPoints[i][j]];
            }
            V[i] /= (float)(nearPoints[i].Count + 1);

            V[i] += dt * a;
            X[i] += dt * V[i];
            V[i] *= damp;
            //TODO: (Particle) collision with floor.
            Force[i] = Vector3.zero;
        }
        Collision_Impulse(new Vector3(0, -3, 0), new Vector3(0, 1, 0));
        for (int i = 0; i < number; i++)
        {
        }

    }

    // Update is called once per frame
    void Update()
    {
    	//for(int l=0; l<10; l++)
    		 _Update();

    	// Dump the vertex array for rendering.
    	Vector3[] vertices = new Vector3[tet_number*12];
        int vertex_number=0;
        for(int tet=0; tet<tet_number; tet++)
        {
        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];
        	vertices[vertex_number++]=X[Tet[tet*4+1]];
        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];
        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+1]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];
        	vertices[vertex_number++]=X[Tet[tet*4+1]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];
        }
        Mesh mesh = GetComponent<MeshFilter> ().mesh;
		mesh.vertices  = vertices;
		mesh.RecalculateNormals ();
    }
}
