#version 330 core
layout (location = 0) in vec3 aP;
layout (location = 3) in mat4 instanceMatrix;

uniform mat4 Projection = mat4(1.0);
uniform mat4 View = mat4(1.0);

void main()
{
    gl_Position = Projection * View * instanceMatrix * vec4(aP, 1.0);
}
