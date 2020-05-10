#version 330 core
layout (location = 0) in vec3 aP;
layout (location = 1) in vec3 aN;
layout (location = 2) in vec3 aColor;
layout (location = 3) in mat4 instanceMatrix;
 
uniform mat4 Projection = mat4(1.0);
uniform mat4 View = mat4(1.0);

uniform mat4 LightViewProjection = mat4(1.0);

out vec3 N;
out vec3 FragWorldP;
out vec4 FragLightClipSpace;
out vec3 Color;

void main()
{
    Color = aColor;

    FragWorldP = vec3(instanceMatrix * vec4(aP, 1.0));
    FragLightClipSpace = LightViewProjection * vec4(FragWorldP, 1.0);
    N = mat3(transpose(inverse(instanceMatrix))) * aN;
    gl_Position = Projection * View * vec4(FragWorldP, 1.0);
}