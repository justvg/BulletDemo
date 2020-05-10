#pragma once

#include <math.h>

#define PI 3.14159265358979323846f

#define Epsilon (1.19e-7f)
#define I32_MIN (-2147483647 - 1)
#define I32_MAX 2147483647
#define U32_MAX 0xFFFFFFFF
#define FLT_MAX 3.402823466e+38F

#define Max(A, B) (((A) > (B)) ? (A) : (B))
#define Min(A, B) (((A) < (B)) ? (A) : (B))

union vec2
{
    struct
    {
        r32 x, y;
    };

    struct
    {
        r32 u, v;
    };

    r32 E[2];

    vec2() {}
    vec2(r32 X, r32 Y) { x = X; y = Y; }
};

union vec3
{
    struct
    {
        r32 x, y, z;
    };

    struct
    {
        r32 u, v, w;
    };

    struct
    {
        r32 r, g, b;
    };

    struct 
    {
        vec2 xy;
        r32 Ignored_0;
    };
    struct 
    {
        r32 Ignored_1;
        vec2 yz;
    };
    struct 
    {
        vec2 uv;
        r32 Ignored_2;
    };
    struct 
    {
        r32 Ignored_3;
        vec2 vw;
    };

    r32 E[3];

    vec3() {}
    vec3(r32 X, r32 Y, r32 Z) { x = X; y = Y; z = Z; }
    vec3(vec2 XY, r32 Z) { x = XY.x; y = XY.y; z = Z; }
};

union vec4
{
    struct
    {
        r32 x, y, z, w;
    };

    struct
    {
        r32 r, g, b, a;
    };

    struct 
    {
        vec3 xyz;
        r32 Ignored_0;
    };
    struct
    {
        vec2 xy;
        r32 Ignored_1;
        r32 Ignored_2;
    };
    struct
    {
        r32 Ignored_3;
        vec2 yz;
        r32 Ignored_4;
    };
    struct
    {
        r32 Ignored_5;
        r32 Ignored_6;
        vec2 zw;
    };

    struct
    {
        vec3 rgb;
        r32 Ignored_7;
    };

    r32 E[4];

    vec4() {}
    vec4(r32 X, r32 Y, r32 Z, r32 W) { x = X; y = Y; z = Z; w = W; }
};

inline vec2
vec2i(i32 X, i32 Y)
{
    vec2 Result = vec2((r32)X, (r32)Y);

    return(Result);
}

inline vec2
vec2i(u32 X, u32 Y)
{
    vec2 Result = vec2((r32)X, (r32)Y);

    return(Result);
}

inline vec3
vec3i(i32 X, i32 Y, i32 Z)
{
    vec3 Result = vec3((r32)X, (r32)Y, (r32)Z);

    return(Result);
}

inline vec3
vec3i(u32 X, u32 Y, u32 Z)
{
    vec3 Result = vec3((r32)X, (r32)Y, (r32)Z);

    return(Result);
}

inline vec4
vec4i(i32 X, i32 Y, i32 Z, i32 W)
{
    vec4 Result = vec4((r32)X, (r32)Y, (r32)Z, (r32)W);

    return(Result);
}

inline vec4
vec4i(u32 X, u32 Y, u32 Z, u32 W)
{
    vec4 Result = vec4((r32)X, (r32)Y, (r32)Z, (r32)W);

    return(Result);
}

union mat3
{
    r32 E[9];
    struct 
    {
        r32 a11, a21, a31;
        r32 a12, a22, a32;
        r32 a13, a23, a33;
    };
};

union mat4
{
    r32 E[16];

    struct 
    {
        r32 a11, a21, a31, a41;
        r32 a12, a22, a32, a42;
        r32 a13, a23, a33, a43;
        r32 a14, a24, a34, a44;
    };
};

inline r32
Radians(r32 Angle)
{
    r32 Result = (Angle / 180.0f) * PI;;

    return(Result);
}

inline r32
Degrees(r32 Rad)
{
    r32 Result = (Rad / PI) * 180.0f;

    return(Result);
}

inline r32
Lerp(r32 A, r32 B, r32 t)
{
    r32 Result = A + (B - A)*t;

    return(Result);
}

// 
// NOTE(georgy): vec2
// 

inline vec2
operator+ (vec2 A, vec2 B)
{
    vec2 Result;

    Result.x = A.x + B.x;
    Result.y = A.y + B.y;

    return(Result);
}

inline vec2
operator- (vec2 A, vec2 B)
{
    vec2 Result;

    Result.x = A.x - B.x;
    Result.y = A.y - B.y;

    return(Result);
}

inline vec2
Hadamard(vec2 A, vec2 B)
{
    vec2 Result;

    Result.x = A.x * B.x;
    Result.y = A.y * B.y;

    return(Result);
}

inline vec2
operator* (vec2 A, r32 B)
{
    vec2 Result;

    Result.x = A.x * B;
    Result.y = A.y * B;

    return(Result);
}

inline vec2
operator* (r32 B, vec2 A)
{
    vec2 Result = A * B;

    return(Result);
}

inline vec2 &
operator+= (vec2 &A, vec2 B)
{
    A = A + B;
    
    return(A);
}

inline vec2 &
operator-= (vec2 &A, vec2 B)
{
    A = A - B;
    
    return(A);
}

inline vec2 &
operator*= (vec2 &A, r32 B)
{
    A = A * B;
    
    return(A);
}

inline vec2
operator- (vec2 A)
{
    vec2 Result;

    Result.x = -A.x;
    Result.y = -A.y;
    
    return(Result);
}

inline r32
Dot(vec2 A, vec2 B)
{
    r32 Result = A.x*B.x + A.y*B.y;

    return(Result);
}

inline r32
LengthSq(vec2 A)
{
    r32 Result = Dot(A, A);

    return(Result);
}

inline r32
Length(vec2 A)
{
    r32 Result = sqrtf(LengthSq(A));

    return(Result);
}

inline vec2
Normalize(vec2 A)
{
    vec2 Result = A * (1.0f / Length(A));

    return(Result);
}

inline vec2
Perp(vec2 A)
{
    vec2 Result = vec2(-A.y, A.x);
    
    return(Result);
}

inline r32
Cross2D(vec2 A, vec2 B)
{
    r32 Result = Dot(Perp(A), B);

    return(Result);
}

inline vec2
Lerp(vec2 A, vec2 B, r32 t)
{
    vec2 Result = A + (B - A)*t;

    return(Result);
}

// 
// NOTE(georgy): vec3
// 

inline vec3
operator+ (vec3 A, vec3 B)
{
    vec3 Result;

    Result.x = A.x + B.x;
    Result.y = A.y + B.y;
    Result.z = A.z + B.z;

    return(Result);
}

inline vec3
operator- (vec3 A, vec3 B)
{
    vec3 Result;

    Result.x = A.x - B.x;
    Result.y = A.y - B.y;
    Result.z = A.z - B.z;

    return(Result);
}

inline vec3
Hadamard(vec3 A, vec3 B)
{
    vec3 Result;

    Result.x = A.x * B.x;
    Result.y = A.y * B.y;
    Result.z = A.z * B.z;

    return(Result);
}

inline vec3
operator* (vec3 A, r32 B)
{
    vec3 Result;

    Result.x = A.x * B;
    Result.y = A.y * B;
    Result.z = A.z * B;

    return(Result);
}

inline vec3
operator* (r32 B, vec3 A)
{
    vec3 Result = A * B;

    return(Result);
}

inline vec3 &
operator+= (vec3 &A, vec3 B)
{
    A = A + B;
    
    return(A);
}

inline vec3 &
operator-= (vec3 &A, vec3 B)
{
    A = A - B;
    
    return(A);
}

inline vec3 &
operator*= (vec3 &A, r32 B)
{
    A = A * B;
    
    return(A);
}

inline vec3
operator- (vec3 A)
{
    vec3 Result;

    Result.x = -A.x;
    Result.y = -A.y;
    Result.z = -A.z;
    
    return(Result);
}

inline r32
Dot(vec3 A, vec3 B)
{
    r32 Result = A.x*B.x + A.y*B.y + A.z*B.z;

    return(Result);
}

inline r32
LengthSq(vec3 A)
{
    r32 Result = Dot(A, A);

    return(Result);
}

inline r32
Length(vec3 A)
{
    r32 Result = sqrtf(LengthSq(A));

    return(Result);
}

inline vec3
Normalize(vec3 A)
{
    vec3 Result = A * (1.0f / Length(A));

    return(Result);
}

inline vec3
NOZ(vec3 A)
{
    vec3 Result;

    r32 Len = Length(A);
    if(Len <= Epsilon)
    {
        Result = vec3(0.0f, 0.0f, 0.0f);
    }
    else
    {
        Result = A * (1.0f / Length(A));
    }

    return(Result);
}

inline vec3
Cross(vec3 A, vec3 B)
{
    vec3 Result;

    Result.x = A.y*B.z - B.y*A.z;
    Result.y = A.z*B.x - B.z*A.x;
    Result.z = A.x*B.y - B.x*A.y;

    return(Result);    
}

inline vec3
Lerp(vec3 A, vec3 B, r32 t)
{
    vec3 Result = A + (B - A)*t;

    return(Result);
}

static b32
PointInTriangle(vec3 P, vec3 A, vec3 B, vec3 C)
{
    A -= P; B -= P; C -= P;
    vec3 U = Cross(B, C);
    vec3 V = Cross(C, A);
    vec3 W = Cross(A, B);
    b32 Result = (Dot(U, V) >= 0.0f) && (Dot(U, W) >= 0.0f);

    return(Result);
}

static vec3
ClosestPointInSegment(vec3 P, vec3 A, vec3 B)
{
    r32 t = Dot(P - A, (B - A) * (1.0f / Length(B - A))) / Length(B - A);
    vec3 Result = A + t*(B - A);
    
    return(Result);
}

enum voronoi_region
{
    VoronoiRegion_Vertex,
    VoronoiRegion_Edge,
    VoronoiRegion_Triangle,
};

struct closest_voronoi_region_triangle_point
{
    voronoi_region VR;
    vec3 P;

    // NOTE(georgy): These are valid if VR = VoronoiRegion_Edge
    vec3 EdgeP0;
    vec3 EdgeP1;

    // NOTE(georgy): These are valid if VR = VoronoiRegion_Vertex or VR = VoronoiRegion_Edge
    u32 VertIndex0;
    u32 VertIndex1;
};

static closest_voronoi_region_triangle_point
ClosestPointInTriangleVR(vec3 P, vec3 A, vec3 B, vec3 C)
{
    closest_voronoi_region_triangle_point Result;

    vec3 AB = B - A;
    vec3 AC = C - A;
    vec3 BC = C - B;

    // NOTE(georgy): Parametric position s for projection P' of P on AB,
    // P' = A + s*AB, s = sNom / (sNom + sDenom)
    r32 sNom = Dot(P - A, AB);
    r32 sDenom = Dot(P - B, A - B); 

    // NOTE(georgy): Parametric position t for projection P' of P on AC,
    // P' = A + t*AC, s = tNom / (tNom + tDenom)
    r32 tNom = Dot(P - A, AC);
    r32 tDenom = Dot(P - C, A - C);

    if((sNom <= 0.0f) && (tNom <= 0.0f))
    {
        Result.VR = VoronoiRegion_Vertex;
        Result.P = A;
        Result.VertIndex0 = 0;
        return(Result);
    }

    // NOTE(georgy): Parametric position u for projection P' of P on BC,
    // P' = B + u*BC, u = uNom / (uNom + uDenom)
    r32 uNom = Dot(P - B, BC);
    r32 uDenom = Dot(P - C, B - C);

    if((sDenom <= 0.0f) && (uNom <= 0.0f))
    {
        Result.VR = VoronoiRegion_Vertex;
        Result.P = B;
        Result.VertIndex0 = 1;
        return(Result);
    }

    if((tDenom <= 0.0f) && (uDenom <= 0.0f))
    {
        Result.VR = VoronoiRegion_Vertex;
        Result.P = C;
        Result.VertIndex0 = 2;
        return(Result);
    }

    vec3 N = Cross(B - A, C - A);
    r32 RAB = Dot(N, Cross(A - P, B - P));
    if((RAB <= 0.0f) && (sNom >= 0.0f) && (sDenom >= 0.0f))
    {
        Result.VR = VoronoiRegion_Edge;
        Result.P = A + AB*(sNom / (sNom + sDenom));
        Result.EdgeP0 = A;
        Result.EdgeP1 = B;
        Result.VertIndex0 = 0;
        Result.VertIndex1 = 1;
        return(Result);
    }

    r32 RBC = Dot(N, Cross(B - P, C - P));
    if((RBC <= 0.0f) && (uNom >= 0.0f) && (uDenom >= 0.0f))
    {
        Result.VR = VoronoiRegion_Edge;
        Result.P = B + BC*(uNom / (uNom + uDenom));
        Result.EdgeP0 = B;
        Result.EdgeP1 = C;
        Result.VertIndex0 = 1;
        Result.VertIndex1 = 2;
        return(Result);
    }

    r32 RCA = Dot(N, Cross(C - P, A - P));
    if((RCA <= 0.0f) && (tNom >= 0.0f) && (tDenom >= 0.0f))
    {
        Result.VR = VoronoiRegion_Edge;
        Result.P = A + AC*(tNom / (tNom + tDenom));
        Result.EdgeP0 = C;
        Result.EdgeP1 = A;
        Result.VertIndex0 = 2;
        Result.VertIndex1 = 0;
        return(Result);
    }

    r32 u = RBC / (RAB + RBC + RCA);
    r32 v = RCA / (RAB + RBC + RCA);
    r32 w = 1.0f - u - v;
    Result.VR = VoronoiRegion_Triangle;
    Result.P = u*A + v*B + w*C;
    return(Result);

#if 0
    // NOTE(georgy): Slow but easy to understand way to do this

    b32 NegativeHalfspaceOfABPlane = (Dot(P - A, B - A) <= 0.0f);
    b32 NegativeHalfspaceOfACPlane = (Dot(P - A, C - A) <= 0.0f);
    if(NegativeHalfspaceOfABPlane && NegativeHalfspaceOfACPlane)
    {
        Result.VR = VoronoiRegion_Vertex;
        Result.P = A;
    }
    else
    {
        b32 NegativeHalfspaceOfBAPlane = (Dot(P - B, A - B) <= 0.0f);
        b32 NegativeHalfspaceOfBCPlane = (Dot(P - B, C - B) <= 0.0f);
        if(NegativeHalfspaceOfBAPlane && NegativeHalfspaceOfBCPlane)
        {
            Result.VR = VoronoiRegion_Vertex;
            Result.P = B;
        }
        else
        {
            b32 NegativeHalfspaceOfCAPlane = (Dot(P - C, A - C) <= 0.0f);
            b32 NegativeHalfspaceOfCBPlane = (Dot(P - C, B - C) <= 0.0f);
            if(NegativeHalfspaceOfCAPlane && NegativeHalfspaceOfCBPlane)
            {
                Result.VR = VoronoiRegion_Vertex;
                Result.P = C;
            }
            else
            {
                vec3 N = Cross(B - A, C - A);

                r32 RAB = Dot(N, Cross(A - P, B - P));
                if((RAB <= 0.0f) && !NegativeHalfspaceOfABPlane && !NegativeHalfspaceOfBAPlane)
                {
                    Result.VR = VoronoiRegion_Edge;
                    Result.P = ClosestPointInSegment(P, A, B);
                    Result.EdgeP0 = A;
                    Result.EdgeP1 = B;
                }
                else
                {
                    r32 RBC = Dot(N, Cross(B - P, C - P));
                    if((RBC <= 0.0f) && !NegativeHalfspaceOfBCPlane && !NegativeHalfspaceOfCBPlane)
                    {
                        Result.VR = VoronoiRegion_Edge;
                        Result.P = ClosestPointInSegment(P, B, C);
                        Result.EdgeP0 = B;
                        Result.EdgeP1 = C;
                    }
                    else
                    {
                        r32 RCA = Dot(N, Cross(C - P, A - P));
                        if((RCA <= 0.0f) && !NegativeHalfspaceOfCAPlane && !NegativeHalfspaceOfACPlane)
                        {
                            Result.VR = VoronoiRegion_Edge;
                            Result.P = ClosestPointInSegment(P, C, A);
                            Result.EdgeP0 = C;
                            Result.EdgeP1 = A; 
                        }
                        else
                        {
                            Result.VR = VoronoiRegion_Triangle;
                            Result.P = P - N*(Dot(P - A, N) / Dot(N, N));
                        }
                    }
                }
            }
        }
    }
#endif
    return(Result);
}

// 
// NOTE(georgy): vec4
// 

inline vec4
operator+ (vec4 A, vec4 B)
{
    vec4 Result;

    Result.x = A.x + B.x;
    Result.y = A.y + B.y;
    Result.z = A.z + B.z;
    Result.w = A.w + B.w;

    return(Result);
}

inline vec4
operator- (vec4 A, vec4 B)
{
    vec4 Result;

    Result.x = A.x - B.x;
    Result.y = A.y - B.y;
    Result.z = A.z - B.z;
    Result.w = A.w - B.w;

    return(Result);
}

inline vec4
Hadamard(vec4 A, vec4 B)
{
    vec4 Result;

    Result.x = A.x * B.x;
    Result.y = A.y * B.y;
    Result.z = A.z * B.z;
    Result.w = A.w * B.w;

    return(Result);
}

inline vec4
operator* (vec4 A, r32 B)
{
    vec4 Result;

    Result.x = A.x * B;
    Result.y = A.y * B;
    Result.z = A.z * B;
    Result.w = A.w * B;

    return(Result);
}

inline vec4
operator* (r32 B, vec4 A)
{
    vec4 Result = A * B;

    return(Result);
}

inline vec4 &
operator+= (vec4 &A, vec4 B)
{
    A = A + B;
    
    return(A);
}

inline vec4 &
operator-= (vec4 &A, vec4 B)
{
    A = A - B;
    
    return(A);
}

inline vec4 &
operator*= (vec4 &A, r32 B)
{
    A = A * B;
    
    return(A);
}

inline vec4
operator- (vec4 A)
{
    vec4 Result;

    Result.x = -A.x;
    Result.y = -A.y;
    Result.z = -A.z;
    Result.w = -A.w;
    
    return(Result);
}

inline r32
Dot(vec4 A, vec4 B)
{
    r32 Result = A.x*B.x + A.y*B.y + A.z*B.z + A.w*B.w;

    return(Result);
}

inline r32
LengthSq(vec4 A)
{
    r32 Result = Dot(A, A);

    return(Result);
}

inline r32
Length(vec4 A)
{
    r32 Result = sqrtf(LengthSq(A));

    return(Result);
}

inline vec4
Normalize(vec4 A)
{
    vec4 Result = A * (1.0f / Length(A));

    return(Result);
}

inline vec4
Lerp(vec4 A, vec4 B, r32 t)
{
    vec4 Result = A + (B - A)*t;

    return(Result);
}

// 
// NOTE(georgy): plane
// 

struct plane
{
    vec2 N;
    r32 D;
};

// 
// NOTE(georgy): mat3
// 

inline mat3
Identity3x3(r32 Diagonal = 1.0f)
{
    mat3 Result = {};

    Result.a11 = Diagonal;
    Result.a22 = Diagonal;
    Result.a33 = Diagonal;

    return(Result);
}

inline mat3
Scaling3x3(r32 Scale)
{
    mat3 Result = {};

    Result.a11 = Scale;
    Result.a22 = Scale;
    Result.a33 = Scale;

    return(Result);
}

inline mat3
Scaling3x3(vec3 Scale)
{
    mat3 Result = {};

    Result.a11 = Scale.x;
    Result.a22 = Scale.y;
    Result.a33 = Scale.z;

    return(Result);
}

static mat3
Rotation3x3(r32 Angle, vec3 Axis)
{
    mat3 Result;

    r32 Rad = Radians(Angle);
    r32 Sine = sinf(Rad);
    r32 Cosine = cosf(Rad);

    Axis = NOZ(Axis);
    if((Axis.x == 0.0f) && (Axis.y == 0.0f) && (Axis.z == 0.0f))
    {
        Result = Identity3x3();
    }
    else
    {
        Result.a11 = Axis.x*Axis.x*(1.0f - Cosine) + Cosine;
        Result.a21 = Axis.x*Axis.y*(1.0f - Cosine) + Axis.z*Sine;
        Result.a31 = Axis.x*Axis.z*(1.0f - Cosine) - Axis.y*Sine;

        Result.a12 = Axis.x*Axis.y*(1.0f - Cosine) - Axis.z*Sine;
        Result.a22 = Axis.y*Axis.y*(1.0f - Cosine) + Cosine;
        Result.a32 = Axis.y*Axis.z*(1.0f - Cosine) + Axis.x*Sine;

        Result.a13 = Axis.x*Axis.z*(1.0f - Cosine) + Axis.y*Sine;
        Result.a23 = Axis.y*Axis.z*(1.0f - Cosine) - Axis.x*Sine;
        Result.a33 = Axis.z*Axis.z*(1.0f - Cosine) + Cosine;
    }

    return(Result);
}

static mat3
Transpose3x3(const mat3 &M)
{
    mat3 Result;

    Result.a11 = M.a11;
	Result.a21 = M.a12;
	Result.a31 = M.a13;

	Result.a12 = M.a21;
	Result.a22 = M.a22;
	Result.a32 = M.a23;

	Result.a13 = M.a31;
	Result.a23 = M.a32;
	Result.a33 = M.a33;

    return(Result);
}

static mat3
Inverse3x3(const mat3 &M)
{
    mat3 InverseMatrix = {};
    
    r32 Determinant = M.a11*M.a22*M.a33 + M.a12*M.a23*M.a31 + M.a13*M.a21*M.a32 - 
                      (M.a31*M.a22*M.a13 + M.a32*M.a23*M.a11 + M.a33*M.a21*M.a12);
    if(Determinant > Epsilon)
    {
        r32 OneOverDeterminant = 1.0f / Determinant;

        InverseMatrix.a11 = (M.a22*M.a33 - M.a32*M.a23)*OneOverDeterminant;
        InverseMatrix.a12 = (-(M.a21*M.a33 - M.a31*M.a23))*OneOverDeterminant;
        InverseMatrix.a13 = (M.a21*M.a32 - M.a31*M.a22)*OneOverDeterminant;
        InverseMatrix.a21 = (-(M.a12*M.a33 - M.a32*M.a13))*OneOverDeterminant;
        InverseMatrix.a22 = (M.a11*M.a33 - M.a31*M.a13)*OneOverDeterminant;
        InverseMatrix.a23 = (-(M.a11*M.a32 - M.a31*M.a12))*OneOverDeterminant;
        InverseMatrix.a31 = (M.a12*M.a23 - M.a22*M.a13)*OneOverDeterminant;
        InverseMatrix.a32 = (-(M.a11*M.a23 - M.a21*M.a13))*OneOverDeterminant;
        InverseMatrix.a33 = (M.a11*M.a22 - M.a21*M.a12)*OneOverDeterminant;

        InverseMatrix = Transpose3x3(InverseMatrix);
    }

    return(InverseMatrix);
}

static mat3
operator*(mat3 A, mat3 B)
{
    mat3 Result;

    for(u32 Row = 0;
        Row < 3;
        Row++)
    {
        for(u32 Column = 0;
            Column < 3;
            Column++)
        {
            r32 Sum = 0.0f;
            for(u32 E = 0;
                E < 3;
                E++)
            {
                Sum += A.E[Row + E*3] * B.E[Column*3 + E];
            }
            Result.E[Row + Column*3] = Sum;
        }
    }

    return(Result);
}

static vec3
operator*(mat3 A, vec3 B)
{
    vec3 Result;

    Result.x = A.a11*B.x + A.a12*B.y + A.a13*B.z;
    Result.y = A.a21*B.x + A.a22*B.y + A.a23*B.z;
    Result.z = A.a31*B.x + A.a32*B.y + A.a33*B.z;

    return(Result);
}

// 
// NOTE(georgy): mat4
// 

inline mat4
Identity(r32 Diagonal = 1.0f)
{
    mat4 Result = {};

    Result.a11 = Diagonal;
    Result.a22 = Diagonal;
    Result.a33 = Diagonal;
    Result.a44 = Diagonal;

    return(Result);
}

inline mat4
Translation(vec3 Translate)
{
    mat4 Result = Identity(1.0f);

    Result.a14 = Translate.x;
    Result.a24 = Translate.y;
    Result.a34 = Translate.z;

    return(Result);
}

inline mat4
Scaling(r32 Scale)
{
    mat4 Result = {};

    Result.a11 = Scale;
    Result.a22 = Scale;
    Result.a33 = Scale;
    Result.a44 = 1.0f;

    return(Result);
}

inline mat4
Scaling(vec3 Scale)
{
    mat4 Result = {};

    Result.a11 = Scale.x;
    Result.a22 = Scale.y;
    Result.a33 = Scale.z;
    Result.a44 = 1.0f;

    return(Result);
}

static mat4
Rotation(r32 Angle, vec3 Axis)
{
    mat4 Result;

    r32 Rad = Radians(Angle);
    r32 Sine = sinf(Rad);
    r32 Cosine = cosf(Rad);

    Axis = Normalize(Axis);

    Result.a11 = Axis.x*Axis.x*(1.0f - Cosine) + Cosine;
	Result.a21 = Axis.x*Axis.y*(1.0f - Cosine) + Axis.z*Sine;
	Result.a31 = Axis.x*Axis.z*(1.0f - Cosine) - Axis.y*Sine;
	Result.a41 = 0.0f;

    Result.a12 = Axis.x*Axis.y*(1.0f - Cosine) - Axis.z*Sine;
	Result.a22 = Axis.y*Axis.y*(1.0f - Cosine) + Cosine;
	Result.a32 = Axis.y*Axis.z*(1.0f - Cosine) + Axis.x*Sine;
	Result.a42 = 0.0f;

    Result.a13 = Axis.x*Axis.z*(1.0f - Cosine) + Axis.y*Sine;
	Result.a23 = Axis.y*Axis.z*(1.0f - Cosine) - Axis.x*Sine;
	Result.a33 = Axis.z*Axis.z*(1.0f - Cosine) + Cosine;
	Result.a43 = 0.0f;

	Result.a14 = 0.0f;
	Result.a24 = 0.0f;
	Result.a34 = 0.0f;
	Result.a44 = 1.0f;

    return(Result);
}

static mat4
Mat4(const mat3 &M)
{
    mat4 Result;

    Result.a11 = M.a11;
    Result.a21 = M.a21;
    Result.a31 = M.a31;
    Result.a12 = M.a12;
    Result.a22 = M.a22;
    Result.a32 = M.a32;
    Result.a13 = M.a13;
    Result.a23 = M.a23;
    Result.a33 = M.a33;
    Result.a41 = Result.a42 = Result.a43 = Result.a14 = Result.a24 = Result.a34 = 0.0f;
    Result.a44 = 1.0f;

    return(Result);
}

static mat4
LookAt(vec3 From, vec3 Target, vec3 UpAxis = vec3(0.0f, 1.0f, 0.0f))
{
    vec3 Forward = Normalize(From - Target);
    vec3 Right = Normalize(Cross(UpAxis, Forward));
    vec3 Up = Cross(Forward, Right);

    mat4 Result;

    Result.a11 = Right.x;
    Result.a21 = Up.x;
    Result.a31 = Forward.x;
    Result.a41 = 0.0f;

    Result.a12 = Right.y;
    Result.a22 = Up.y;
    Result.a32 = Forward.y;
    Result.a42 = 0.0f;

    Result.a13 = Right.z;
    Result.a23 = Up.z;
    Result.a33 = Forward.z;
    Result.a43 = 0.0f;

    Result.a14 = -Dot(Right, From);
    Result.a24 = -Dot(Up, From);
    Result.a34 = -Dot(Forward, From);
    Result.a44 = 1.0f;

    return(Result);
}

static mat4
ViewRotationMatrixFromDirection(vec3 Dir, vec3 UpAxis = vec3(0.0f, 1.0f, 0.0f))
{
    vec3 Forward = Normalize(-Dir);
    vec3 Right = Normalize(Cross(UpAxis, Forward));
    vec3 Up = Cross(Forward, Right);

    mat4 Result;

    Result.a11 = Right.x;
    Result.a21 = Up.x;
    Result.a31 = Forward.x;
    Result.a41 = 0.0f;

    Result.a12 = Right.y;
    Result.a22 = Up.y;
    Result.a32 = Forward.y;
    Result.a42 = 0.0f;

    Result.a13 = Right.z;
    Result.a23 = Up.z;
    Result.a33 = Forward.z;
    Result.a43 = 0.0f;

    Result.a14 = 0.0f;
    Result.a24 = 0.0f;
    Result.a34 = 0.0f;
    Result.a44 = 1.0f;

    return(Result);
}

static mat4 
Ortho(r32 Bottom, r32 Top, r32 Left, r32 Right, r32 Near, r32 Far)
{
    mat4 Result = {};

    Result.a11 = 2.0f / (Right - Left);
    Result.a22 = 2.0f / (Top - Bottom);
    Result.a33 = -2.0f / (Far - Near);
    Result.a14 = -(Right + Left) / (Right - Left);
    Result.a24 = -(Top + Bottom) / (Top - Bottom);
	Result.a34 = -(Far + Near) / (Far - Near);
	Result.a44 = 1.0f;

    return(Result);
}

static mat4
Perspective(r32 FoV, r32 AspectRatio, r32 Near, r32 Far)
{
    r32 Scale = tanf(Radians(FoV) * 0.5f) * Near;
    r32 Top = Scale;
    r32 Bottom = -Top;
    r32 Right = Scale * AspectRatio;
    r32 Left = -Right;

    mat4 Result = {};

    Result.a11 = 2.0f * Near / (Right - Left);
	Result.a22 = 2.0f * Near / (Top - Bottom);
	Result.a13 = (Right + Left) / (Right - Left);
	Result.a23 = (Top + Bottom) / (Top - Bottom);
	Result.a33 = -(Far + Near) / (Far - Near);
	Result.a43 = -1.0f;
	Result.a34 = -(2.0f * Far*Near) / (Far - Near);

    return(Result);
}

static mat4
operator*(mat4 A, mat4 B)
{
    mat4 Result;

    for(u32 Row = 0;
        Row < 4;
        Row++)
    {
        for(u32 Column = 0;
            Column < 4;
            Column++)
        {
            r32 Sum = 0.0f;
            for(u32 E = 0;
                E < 4;
                E++)
            {
                Sum += A.E[Row + E*4] * B.E[Column*4 + E];
            }
            Result.E[Row + Column*4] = Sum;
        }
    }

    return(Result);
}
