#version 330 core
out vec4 FragColor;

// uniform vec3 Color;
in vec3 Color;
uniform vec3 CameraWorldP;

uniform sampler2D ShadowMap;
uniform vec2 SampleKernel[16];
uniform sampler2D ShadowNoiseTexture;

in vec3 N;
in vec3 FragWorldP;
in vec4 FragLightClipSpace;

const vec3 LightDir = normalize(vec3(-3.0, -6.0, -3.0));

float CalculateShadowFactor(vec3 Normal, vec3 LightDir)
{
    float ShadowFactor = 0.0;

    float CosAlpha = max(dot(Normal, LightDir), 0.0);
    float SinAlpha = sqrt(1.0 - CosAlpha*CosAlpha);
    float TanAlpha = SinAlpha / CosAlpha;
    float Bias = 0.005*TanAlpha;

    vec3 ProjectedCoords = FragLightClipSpace.xyz / FragLightClipSpace.w;
    ProjectedCoords = 0.5*ProjectedCoords + vec3(0.5, 0.5, 0.5);

    vec2 TexelSize = 1.0 / textureSize(ShadowMap, 0).xy;
    float CurrentFragDepth = ProjectedCoords.z - Bias;

    float NoiseScale = 1.0 / 16.0;
    vec2 RandomVec = texture(ShadowNoiseTexture, NoiseScale*gl_FragCoord.xy).rg;
    vec2 Perp = vec2(-RandomVec.y, RandomVec.x);
    mat2 ChangeOffsetMatrix = mat2(RandomVec, Perp);

    for(int SampleIndex = 0;
        SampleIndex < 16;
        SampleIndex++)
    {
        vec2 Offset = 1.25*TexelSize*(ChangeOffsetMatrix*SampleKernel[SampleIndex]);
        float Depth = texture(ShadowMap, ProjectedCoords.xy + Offset).r;
        ShadowFactor += (CurrentFragDepth <= Depth) ? 1.0 : 0.0;
    }

    ShadowFactor /= 16;

    return(ShadowFactor);
}

void main()
{
    vec3 Normal = normalize(N);

    vec3 Ambient = 0.3*Color;
    vec3 Diffuse = max(dot(Normal, -LightDir), 0.0)*Color;
    vec3 ViewDir = normalize(CameraWorldP - FragWorldP);
    vec3 ReflectedLightDir = reflect(LightDir, Normal);
    vec3 Specular = pow(max(dot(ViewDir, ReflectedLightDir), 0.0), 32)*Color;

    float ShadowFactor = CalculateShadowFactor(Normal, -LightDir);

    FragColor = vec4(Ambient + ShadowFactor*(Diffuse + Specular), 1.0);
}