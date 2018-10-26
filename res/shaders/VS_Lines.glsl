#version 330

layout(location = 0) in vec3 position;
layout(location = 1) in vec3 color;

uniform mat4 modelToClip;

out vec3 col;

void main() {
    col = color;
    gl_Position = modelToClip * vec4(position, 1);
}
