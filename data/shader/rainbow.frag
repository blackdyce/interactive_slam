#version 330
uniform vec2 z_range;
uniform int z_clipping;
uniform int color_mode;

in vec4 frag_color;
flat in ivec4 frag_info;

layout (location=0) out vec4 color;
layout (location=1) out ivec4 info;

void main() {
    color = frag_color;
    info = frag_info;
}