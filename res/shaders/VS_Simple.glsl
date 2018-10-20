/**

    Automaatio- ja Systeemitekniikan killan robokäsiprojekti
    VS_Simple.glsl

    @version    0.1
    @author     Miika 'LehdaRi' Lehtimäki
    @date       2015-04-18

**/


#version 130

//layout(location = 0) in vec3 position;
//layout(location = 2) in vec3 normal;

in vec4 position;
in vec3 normal;

out vec4 pos;
out vec4 norm;
out vec3 col;

uniform mat4 MVP;
uniform vec3 Color;

void main() {
    pos = MVP * position;
    norm = MVP * vec4(normal, 0.0);
    col = Color;
    gl_Position = pos;
}
